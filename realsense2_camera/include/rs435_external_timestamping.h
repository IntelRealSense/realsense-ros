#ifndef REALSENSE2_CAMERA_RS435_EXTERNAL_TIMESTAMING_H
#define REALSENSE2_CAMERA_RS435_EXTERNAL_TIMESTAMPING_H

#include "ros/ros.h"
#include <std_msgs/Header.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <mutex>
#include <tuple>

namespace external_timestamping {
enum class sync_state {
    synced,
    not_initalized,
    wait_for_sync,
};

enum class inter_cam_sync_mode {
    none,
    master,
    slave
};

template<typename t_chanel_id>
class ExternalTimestamping {
    // template that holds the function used to publish restamped frames
    typedef boost::function<void(const t_chanel_id &channel,
                                 const ros::Time &stamp,
                                 const sensor_msgs::ImagePtr image,
                                 const sensor_msgs::CameraInfo info)> pub_frame_fn;

    typedef struct {
        uint32_t seq;
        ros::Time cam_stamp;
        ros::Time arrival_stamp;
        sensor_msgs::ImagePtr img;
        sensor_msgs::CameraInfo info;
        double exposure;
        void reset() {
            seq = 0;
        }
    } frame_buffer_type;

    typedef struct {
        uint32_t seq;
        ros::Time hw_stamp;
        ros::Time arrival_stamp;
        void reset() {
            seq = 0;
        }
    } hw_stamp_buffer_type;

 private:
    ros::NodeHandle nh_;

    const std::set<t_chanel_id> _channel_set;
    int _hw_stamp_seq_offset = 0;
    double _static_time_offset;
    int _inter_cam_sync_mode;
    int _frame_rate;

    ros::Subscriber _cam_imu_sub;
    
    std::mutex _mutex;

    pub_frame_fn _publish_frame_fn;
    sync_state _state;

    const std::string _log_prefix = "[HW timesync] ";
    std::map<t_chanel_id, hw_stamp_buffer_type> _hw_stamp_buffer;
    std::map<t_chanel_id, frame_buffer_type> _frame_buffer;

 public:

    ExternalTimestamping(const std::set<t_chanel_id> &channel_set) :
            _channel_set(channel_set),
            _state(sync_state::not_initalized) {
        ROS_DEBUG_STREAM(_log_prefix << " Initialized with " << _channel_set.size() << " channels.");
        for (t_chanel_id channel : _channel_set) {
            _hw_stamp_buffer[channel].reset();
        }
    }

    void setup(const pub_frame_fn &pub_function_ptr, int fps, 
               double static_time_offset, int inter_cam_sync_mode) {
        _static_time_offset = static_time_offset;
        _inter_cam_sync_mode = inter_cam_sync_mode;
        _frame_rate = fps;

        _publish_frame_fn = pub_function_ptr;
        _state = sync_state::not_initalized;

        _cam_imu_sub = nh_.subscribe("/hw_stamp", 100, &ExternalTimestamping::hwStampCallback, this);
    }

    void start() {
        if (!_publish_frame_fn) {
            ROS_ERROR_STREAM(_log_prefix << " No publish function set - discarding buffered images.");
        }

        for (t_chanel_id channel : _channel_set) {
            // clear buffers in case start() is invoked for re-initialization
            _hw_stamp_buffer[channel].reset();
            _frame_buffer[channel].img.reset();
        }

        _state = sync_state::wait_for_sync;
    }

    bool channelValid(const t_chanel_id &channel) const {
        return _channel_set.count(channel) == 1;
    }

    void bufferFrame(const t_chanel_id &channel, const uint32_t seq, const ros::Time &cam_stamp, 
                    double exposure, const sensor_msgs::ImagePtr img, const sensor_msgs::CameraInfo info) {
        std::lock_guard<std::mutex> lg(_mutex);

        if (!channelValid(channel)) {
            ROS_WARN_STREAM_ONCE(_log_prefix << "bufferFrame called for invalid channel.");
            return;
        }

        if (_frame_buffer[channel].img) {
            ROS_WARN_STREAM(_log_prefix << 
                "Overwriting image buffer! Make sure you are getting Timestamps from mavros.");
        }

        // set frame buffer
        _frame_buffer[channel].img = img;
        _frame_buffer[channel].info = info;
        _frame_buffer[channel].seq = seq;
        _frame_buffer[channel].arrival_stamp = ros::Time::now();
        _frame_buffer[channel].cam_stamp = cam_stamp; //store stamp that was constructed by ros-realsense
        _frame_buffer[channel].exposure = exposure;
        ROS_DEBUG_STREAM(_log_prefix << "Buffered frame, seq: " << seq);
    }

    bool syncSeqOffset(const t_chanel_id &channel, const uint32_t seq_frame, const double &delay) {
        // Get offset between first frame sequence and mavros
        _hw_stamp_seq_offset = _hw_stamp_buffer[channel].seq - static_cast<int32_t>(seq_frame);

        ROS_INFO_STREAM(_log_prefix << 
            "New seq offset determined by channel " << channel.first << ": " << _hw_stamp_seq_offset << 
            ", from " << _hw_stamp_buffer[channel].seq << 
            " to " << seq_frame);

        _state = sync_state::synced;
        return true;
    }

    bool lookupHardwareStamp(const t_chanel_id &channel, const uint32_t frame_seq,
                            const ros::Time &cam_stamp, double exposure, 
                            const sensor_msgs::ImagePtr img, const sensor_msgs::CameraInfo info) { 
        // Method to match an incoming frame to a buffered trigger
        // if a matching HW stamp was found we return true and publish the restamped frame
        // if no matching trigger is found we return false to buffer the frame
        std::lock_guard<std::mutex> lg(_mutex);

        if (!channelValid(channel)) {
            // looking up stamp for unsynced image stream
            return false;
        }

        ROS_DEBUG_STREAM(_log_prefix <<  std::setprecision(15) <<
            "Received frame, stamp: " << cam_stamp.toSec() << 
            ", seq nr: " << frame_seq);

        if (_state == sync_state::not_initalized) {
            return false;
        }

        if (_hw_stamp_buffer[channel].seq == 0) {
            // empty hw stamp buffer. return false to buffer frame
            return false;
        }

        // set max allowed age of buffered stamp to the interval between two frames
        const double kMaxStampAge = 1.0/_frame_rate;
        const double age_buffered_hw_stamp = cam_stamp.toSec() - _hw_stamp_buffer[channel].arrival_stamp.toSec();
        if (std::fabs(age_buffered_hw_stamp) > kMaxStampAge) {
            ROS_DEBUG_STREAM(_log_prefix << "Delay out of bounds: " << 
                            kMaxStampAge << " seconds. Clearing hw stamp buffer...");
            _hw_stamp_buffer[channel].reset();

            _state = sync_state::wait_for_sync;
            return false;
        }

        uint32_t expected_hw_stamp_seq = frame_seq + _hw_stamp_seq_offset;
        if (_state == sync_state::wait_for_sync || _hw_stamp_buffer[channel].seq != expected_hw_stamp_seq) {
            // there is a buffered stamp and it is within the expected time interval
            // however, we did not determine the sequence yet or the seq id did not match the one we expected 
            // call syncSeqOffset to set or reset the seq id offset between frames and stamps
            ROS_WARN_STREAM(_log_prefix << "Dropped stamp: could not find hw stamp with seq id: " <<  expected_hw_stamp_seq);
            syncSeqOffset(channel, frame_seq, age_buffered_hw_stamp);
        }

        // successful match: shift stamp and publish frame
        ros::Time hw_stamp = _hw_stamp_buffer[channel].hw_stamp;
        hw_stamp = shiftTimestampToMidExposure(hw_stamp, exposure);
        _publish_frame_fn(channel, hw_stamp, img, info);

        ROS_INFO_STREAM(_log_prefix <<  std::setprecision(15) << 
            "frame#" << frame_seq << " -> stamp#" << expected_hw_stamp_seq <<
            ", t_cam " << cam_stamp.toSec() << " -> t_hw " << hw_stamp.toSec() << 
            std::setprecision(7) << " delay: " << age_buffered_hw_stamp);
        
        _hw_stamp_buffer[channel].reset();
        return true;
    }


    bool lookupFrame(const t_chanel_id channel, const uint32_t hw_stamp_seq, 
                     ros::Time &hw_stamp, const ros::Time &arrival_stamp) {
        // Method to match an incoming trigger to a buffered frame
        // if a matching frame was found we return true and publish the restamped frame
        // if no matching frame is found we return false and the frame is buffered
        std::lock_guard<std::mutex> lg(_mutex);

        if (!_frame_buffer[channel].img) {
            // empty frame buffer. return false to buffer stamp
            return false;
        }

        // currently set to 0 as we expect hw stamps to arrive before the frame
        const double kMaxFrameAge = 0e-3; 
        const double age_buffered_frame = arrival_stamp.toSec() 
                                        - _frame_buffer[channel].arrival_stamp.toSec(); 
        if (std::fabs(age_buffered_frame) > kMaxFrameAge) {
            ROS_DEBUG_STREAM(_log_prefix << "Delay out of bounds:  "
                            << kMaxFrameAge << " seconds. Releasing buffered frame...");
            _frame_buffer[channel].img.reset();
            
            _state = sync_state::wait_for_sync;
            return false;
        }

        uint32_t expected_frame_seq = hw_stamp_seq - _hw_stamp_seq_offset;
        if (_state == sync_state::wait_for_sync || _frame_buffer[channel].seq != expected_frame_seq) {
            // buffered frame is within the expected delay 
            // however, the seq id offset is not set or it does not match the expected stamp seq id
            // call syncSeqOffset() to set or reset the offset between frame and stamp seq id
            syncSeqOffset(channel, _frame_buffer[channel].seq, age_buffered_frame);
            ROS_WARN_STREAM(_log_prefix << "Dropped frame: could not find frame with seq id: " << expected_frame_seq);
        }

        // successful match: shift stamp and publish frame
        hw_stamp = shiftTimestampToMidExposure(hw_stamp, _frame_buffer[channel].exposure);
        _publish_frame_fn(channel, hw_stamp, _frame_buffer[channel].img, _frame_buffer[channel].info);
        
        ROS_DEBUG_STREAM(_log_prefix << std::setprecision(15) << 
            "frame#" << expected_frame_seq << " -> stamp#" << hw_stamp_seq <<
            ", t_old " << _frame_buffer[channel].cam_stamp.toSec() << 
            " -> t_new " << hw_stamp.toSec() << 
            std::setprecision(7) << " delay: " << age_buffered_frame);
        
        _frame_buffer[channel].img.reset();
        return true;
    }

    ros::Time shiftTimestampToMidExposure(const ros::Time &stamp, double exposure_us) {
        ros::Time new_stamp = stamp
                            - ros::Duration(exposure_us * 1e-6 / 2.0)
                            + ros::Duration(_static_time_offset);
        ROS_DEBUG_STREAM(_log_prefix << "Shift timestamp: " << stamp.toSec() << " -> " << 
                         new_stamp.toSec() << " exposure: " << exposure_us * 1e-6);
        return new_stamp;
    }

    void hwStampCallback(const std_msgs::Header &header) {
        if (_state == sync_state::not_initalized) {
            // Do nothing before triggering is setup and initialized
            return;
        }

        ros::Time arrival_stamp = ros::Time::now();
        ros::Time hw_stamp = header.stamp;
        uint32_t hw_stamp_seq  = header.seq;

        ROS_DEBUG_STREAM(_log_prefix << "Received hw stamp   : " <<
                std::setprecision(15) <<
                hw_stamp.toSec() <<
                ", seq nr: " << hw_stamp_seq <<
                " (synced frame_seq: " << hw_stamp_seq-_hw_stamp_seq_offset << ")");

        for (auto channel : _channel_set) {
            if (!lookupFrame(channel, hw_stamp_seq, hw_stamp, arrival_stamp)) {
                // buffer hw_stamp if lookupFrame returns false
                _frame_buffer[channel].img.reset();
                _hw_stamp_buffer[channel].seq = hw_stamp_seq;
                _hw_stamp_buffer[channel].arrival_stamp = arrival_stamp;
                _hw_stamp_buffer[channel].hw_stamp = hw_stamp;
            }
        }
    }


};

}

#endif //REALSENSE2_CAMERA_RS435_EXTERNAL_TIMESTAMPING_H
