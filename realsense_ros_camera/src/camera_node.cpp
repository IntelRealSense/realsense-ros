// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2016 Intel Corporation. All Rights Reserved

#include <iostream>
#include <functional>
#include <iomanip>
#include <map>
#include <atomic>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <librealsense/rs.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/version.hpp>
#include <sensor_msgs/CameraInfo.h>
#include <realsense_ros_camera/Extrinsics.h>
#include <realsense_ros_camera/IMUInfo.h>
#include <realsense_ros_camera/constants.h>
#include <sensor_msgs/Imu.h>

const int STREAM_COUNT = 5;
ros::Publisher feInfo_publisher_,
    colorInfo_publisher_,
    depthInfo_publisher_,
    accelInfo_publisher_,
    gyroInfo_publisher_,
    fe2imu_publisher_,
    fe2depth_publisher_;
image_transport::Publisher image_publishers_[STREAM_COUNT] = {};
ros::Publisher imu_publishers_[2] = {};//accel 0 gyro 1
int seq_motion[2]= {0,0};
cv::Mat image_[STREAM_COUNT] = {};
sensor_msgs::CameraInfo camera_info_[STREAM_COUNT] = {};
sensor_msgs::ImagePtr image_msg_[STREAM_COUNT] = {};
int seq[STREAM_COUNT]= {0,0,0,0,0};

std::string encoding_[STREAM_COUNT] = {
    sensor_msgs::image_encodings::TYPE_16UC1,
    sensor_msgs::image_encodings::RGB8,
    sensor_msgs::image_encodings::TYPE_8UC1,
    sensor_msgs::image_encodings::TYPE_8UC1,
    sensor_msgs::image_encodings::TYPE_8UC1
};

int unit_step_size_[STREAM_COUNT] = {
    sizeof(uint16_t),
    sizeof(unsigned char) * 3,
    sizeof(unsigned char),
    sizeof(unsigned char),
    sizeof(unsigned char)
};

int step_[STREAM_COUNT] = {};
std::string optical_frame_id_[STREAM_COUNT] = {
    "camera_depth_optical_frame",
    "camera_rgb_optical_frame",
    "camera_ir_optical_frame",
    "camera_ir2_optical_frame",
    "camera_fisheye_optical_frame"
};

std::string optical_imu_id_[2] = {"imu_accel_frame_id","imu_gyro_frame_id"};
bool isZR300 = false;
std::string serial_no;

namespace realsense_ros_camera
{

void getImuInfo(rs::device* device, IMUInfo &accelInfo, IMUInfo &gyroInfo)
{
    rs::motion_intrinsics imuIntrinsics = device->get_motion_intrinsics();
    
    accelInfo.header.frame_id = "imu_accel";  
    int index = 0;
    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 4; ++j)
        {
            accelInfo.data[index] = imuIntrinsics.acc.data[i][j];
            ++index;
        }
        accelInfo.noise_variances[i] = imuIntrinsics.acc.noise_variances[i];
        accelInfo.bias_variances[i] = imuIntrinsics.acc.bias_variances[i];
    }
    
    gyroInfo.header.frame_id = "imu_gyro";
    index = 0;
    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 4; ++j)
        {
            gyroInfo.data[index] = imuIntrinsics.gyro.data[i][j];
            ++index;
        }
        gyroInfo.noise_variances[i] = imuIntrinsics.gyro.noise_variances[i];
        gyroInfo.bias_variances[i] = imuIntrinsics.gyro.bias_variances[i];
    }
}

Extrinsics rsExtrinsicsToMsg(rs::extrinsics rsExtrinsics)
{
    Extrinsics extrinsicsMsg;
    
    for (int i = 0; i < 9; ++i) 
    {
        extrinsicsMsg.rotation[i] = rsExtrinsics.rotation[i];
        if (i < 3) extrinsicsMsg.translation[i] = rsExtrinsics.translation[i];
    }
    
    return extrinsicsMsg;
}

Extrinsics getFisheye2ImuExtrinsicsMsg(rs::device* device)
{    
    Extrinsics extrinsicsMsg = rsExtrinsicsToMsg(device->get_motion_extrinsics_from(rs::stream::fisheye));
    extrinsicsMsg.header.frame_id = "fisheye2imu_extrinsics";
    return extrinsicsMsg;
}

Extrinsics getFisheye2DepthExtrinsicsMsg(rs::device* device)
{
    Extrinsics extrinsicsMsg =  rsExtrinsicsToMsg(device->get_extrinsics(rs::stream::depth, rs::stream::fisheye));
    extrinsicsMsg.header.frame_id = "fisheye2depth_extrinsics";
    return extrinsicsMsg;
}

class NodeletCamera:public nodelet::Nodelet
{
public:
    NodeletCamera() {}
    ros::NodeHandle node_handle, pnh_;
    rs::device *device;

   std::string serial_no_;
   std::string usb_port_id_;
   std::string camera_type_;

   std::map<rs::stream,int> width_;
   std::map<rs::stream,int> height_;
   std::map<rs::stream,int> fps_;

private:
    void getStreamCalibData(rs::stream stream_index);
    int getDatas();
 
 
    void getParameters()
    {
      pnh_ = getPrivateNodeHandle();

      pnh_.param("depth_width", width_[rs::stream::depth], DEPTH_WIDTH);
      pnh_.param("depth_height", height_[rs::stream::depth], DEPTH_HEIGHT);
      pnh_.param("depth_fps", fps_[rs::stream::depth], DEPTH_FPS);

      pnh_.param("color_width", width_[rs::stream::color], COLOR_WIDTH);
      pnh_.param("color_height", height_[rs::stream::color], COLOR_HEIGHT);
      pnh_.param("color_fps", fps_[rs::stream::color], COLOR_FPS);

      pnh_.param("fisheye_width", width_[rs::stream::fisheye], FISHEYE_WIDTH);
      pnh_.param("fisheye_height", height_[rs::stream::fisheye], FISHEYE_HEIGHT);
      pnh_.param("fisheye_fps", fps_[rs::stream::fisheye], FISHEYE_FPS);
    }

    virtual void onInit()
    {
        getParameters();

        node_handle = getNodeHandle();
        image_transport::ImageTransport image_transport(node_handle);
        
        ros::NodeHandle pnh = getPrivateNodeHandle();
        node_handle.param<std::string>("serial_no", serial_no, "");
        std::unique_ptr< rs::context > ctx(new rs::context());
        int num_of_cams = ctx -> get_device_count();
        if (num_of_cams == 0)
        {
            ROS_ERROR("error : can't find devices");
            exit(0);
        }
        rs::device *detected_dev;
        for (int i = 0; i < num_of_cams; i++)
        {
            detected_dev = ctx -> get_device(i);
            detected_dev -> get_serial();
            serial_no.empty();
            if(serial_no.empty() || (serial_no == std::string(detected_dev -> get_serial())))
            {
                device = detected_dev;
                ROS_INFO_STREAM("device serial No.: "<<std::string(device -> get_serial()));
                break;
            }
        }
        if (device == nullptr)
        {
            ROS_ERROR_STREAM("Couldn't find camera to connect with serial_no = " << serial_no);
            return;
        }
        auto device_name = device -> get_name();
        ROS_INFO_STREAM(device_name);
        if (std::string(device_name).find("ZR300") == std::string::npos) {
            isZR300 = false;
            if ((std::string(device_name).find("R200") == std::string::npos) && (std::string(device_name).find("SR300") == std::string::npos))
            {
                ROS_INFO_STREAM("Please use ZR300, LR200 or SR300");
                return;
            }
        }
        else {
            isZR300 = true;
        }        
        
        // Stream publishers
        image_publishers_[(int32_t)rs::stream::color] = image_transport.advertise("camera/color/image_raw", 1);
        image_publishers_[(int32_t)rs::stream::depth] = image_transport.advertise("camera/depth/image_raw", 1);

        // Latched topics            
        colorInfo_publisher_ = node_handle.advertise< sensor_msgs::CameraInfo >("camera/color/camera_info", 1, true);
        depthInfo_publisher_ = node_handle.advertise< sensor_msgs::CameraInfo >("camera/depth/camera_info", 1, true);

        if (isZR300)
        {
            // Stream publishers
            image_publishers_[(int32_t)rs::stream::fisheye] = image_transport.advertise("camera/fisheye/image_raw", 1);
            imu_publishers_[RS_EVENT_IMU_GYRO] = node_handle.advertise< sensor_msgs::Imu >("camera/gyro/sample", 100); // TODO: review queue size
            imu_publishers_[RS_EVENT_IMU_ACCEL] = node_handle.advertise< sensor_msgs::Imu >("camera/accel/sample", 100); // TODO: review queue size
            
            // Latched topics            
            feInfo_publisher_ = node_handle.advertise< sensor_msgs::CameraInfo >("camera/fisheye/camera_info", 1, true);
            fe2imu_publisher_ = node_handle.advertise< Extrinsics >("camera/extrinsics/fisheye2imu", 1, true);
            fe2depth_publisher_ = node_handle.advertise< Extrinsics >("camera/extrinsics/fisheye2depth", 1, true);
            accelInfo_publisher_ = node_handle.advertise< IMUInfo >("camera/accel/imu_info", 1, true);
            gyroInfo_publisher_ = node_handle.advertise< IMUInfo >("camera/gyro/imu_info", 1, true);
        }
        // end publishers and services
        
        int ret = getDatas();
        ROS_INFO_STREAM("end of onInit " << ret);
    }//end onInit
};//end class
PLUGINLIB_DECLARE_CLASS(realsense_ros_camera, NodeletCamera, realsense_ros_camera::NodeletCamera, nodelet::Nodelet);

int NodeletCamera::getDatas()
{
    std::map< rs::stream, std::function< void (rs::frame) > > stream_callback_per_stream;
    const rs::stream All[] = { rs::stream::depth, rs::stream::color, rs::stream::fisheye };

    for ( const auto stream : All )
    {
        if (stream == rs::stream::depth)
        {
            device->enable_stream(stream, width_[stream], height_[stream], rs::format::z16, fps_[stream]);
        }
        else if (stream == rs::stream::fisheye && isZR300)
        {
            device->enable_stream(stream, width_[stream], height_[stream], rs::format::raw8, fps_[stream]);
        }
        else if (stream == rs::stream::color)
        {
	    device->enable_stream(stream, width_[stream], height_[stream], rs::format::rgb8, fps_[stream]);
        }

        stream_callback_per_stream[stream] = [stream](rs::frame frame)
        {
            if (isZR300)
            {
                const auto timestampDomain = frame.get_frame_timestamp_domain();
                if (rs::timestamp_domain::microcontroller != timestampDomain)
                {
                    ROS_ERROR_STREAM("error: Junk time stamp in stream:" << (int)(stream) <<
                                     "\twith frame counter:" << frame.get_frame_number());
                    return ;
                }
            }
            int32_t stream_nb = (int32_t) stream;
            image_[stream_nb].data = (unsigned char *) frame.get_data();
            image_msg_[stream_nb] = cv_bridge::CvImage(std_msgs::Header(), encoding_[stream_nb], image_[stream_nb]).toImageMsg();
            image_msg_[stream_nb]->header.frame_id = optical_frame_id_[stream_nb];
            image_msg_[stream_nb]->header.stamp = ros::Time(frame.get_timestamp());
            image_msg_[stream_nb]->header.seq = seq[stream_nb];
            image_msg_[stream_nb]->width = image_[stream_nb].cols;
            image_msg_[stream_nb]->height = image_[stream_nb].rows;
            image_msg_[stream_nb]->is_bigendian = false;
            image_msg_[stream_nb]->step = image_[stream_nb].cols * unit_step_size_[stream_nb];
            seq[stream_nb] += 1;

            image_publishers_[stream_nb].publish(image_msg_[stream_nb]);
        };
        device->set_frame_callback(stream, stream_callback_per_stream[stream]);
    }//end for
    
    if(isZR300) {
        device->set_option(rs::option::fisheye_strobe, 1);
        //define callback to the motion events and set it.
        std::function<void(rs::motion_data)> motion_callback;
        std::function<void(rs::timestamp_data)> timestamp_callback;
        motion_callback = [](rs::motion_data entry)
        {
            if ((entry.timestamp_data.source_id != RS_EVENT_IMU_GYRO) && (entry.timestamp_data.source_id != RS_EVENT_IMU_ACCEL))
                return;
            rs_event_source motionType = entry.timestamp_data.source_id;
            sensor_msgs::Imu imu_msg = sensor_msgs::Imu();
            imu_msg.header.stamp = ros::Time::now();
            imu_msg.header.frame_id = optical_imu_id_[motionType];
            imu_msg.orientation.x = 0.0;
            imu_msg.orientation.y = 0.0;
            imu_msg.orientation.z = 0.0;
            imu_msg.orientation.w = 0.0;
            imu_msg.orientation_covariance = {-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
            if (motionType == RS_EVENT_IMU_GYRO) {
                imu_msg.angular_velocity.x = entry.axes[0];
                imu_msg.angular_velocity.y = entry.axes[1];
                imu_msg.angular_velocity.z = entry.axes[2];
            }
            if (motionType == RS_EVENT_IMU_ACCEL) {
                imu_msg.linear_acceleration.x = entry.axes[0];
                imu_msg.linear_acceleration.y = entry.axes[1];
                imu_msg.linear_acceleration.z = entry.axes[2];
            }
            seq_motion[motionType] += 1;
            imu_msg.header.seq = seq_motion[motionType];
            imu_msg.header.stamp = ros::Time(entry.timestamp_data.timestamp);
            imu_publishers_[motionType].publish(imu_msg);
        };
        timestamp_callback = [](rs::timestamp_data entry) {};
        device->enable_motion_tracking(motion_callback, timestamp_callback);
        getStreamCalibData(rs::stream::fisheye);
        image_[(int32_t)rs::stream::fisheye] = cv::Mat(camera_info_[(int32_t)rs::stream::fisheye].height, camera_info_[(int32_t)rs::stream::fisheye].width, CV_8UC1, cv::Scalar(0,0,0));
                
        // publish ZR300-specific intrinsics/extrinsics
        IMUInfo accelInfo, gyroInfo;
        getImuInfo(device, accelInfo, gyroInfo);
        feInfo_publisher_.publish(camera_info_[(int32_t)rs::stream::fisheye]);
        fe2imu_publisher_.publish(getFisheye2ImuExtrinsicsMsg(device));
        fe2depth_publisher_.publish(getFisheye2DepthExtrinsicsMsg(device));
        accelInfo_publisher_.publish(accelInfo);
        gyroInfo_publisher_.publish(gyroInfo);
    }
    
    // get camera intrinsics
    getStreamCalibData(rs::stream::depth);
    getStreamCalibData(rs::stream::color);
    
    // publish camera intrinsics/extrinsics        
    colorInfo_publisher_.publish(camera_info_[(int32_t)rs::stream::color]);
    depthInfo_publisher_.publish(camera_info_[(int32_t)rs::stream::depth]);
    
    image_[(int32_t)rs::stream::depth]   = cv::Mat(camera_info_[(int32_t)rs::stream::depth].height, camera_info_[(int32_t)rs::stream::depth].width, CV_16UC1, cv::Scalar(0,0,0));
    image_[(int32_t)rs::stream::color]   = cv::Mat(camera_info_[(int32_t)rs::stream::color].height, camera_info_[(int32_t)rs::stream::color].width, CV_8UC3, cv::Scalar(0,0,0));

    if(isZR300)
        device->start(rs::source::all_sources);
    else
        device->start();

    ROS_INFO_STREAM("RealSense camera started streaming");
    
    ros::Rate loop_rate(30);
    while (ros::ok())
    {
        loop_rate.sleep();
    }
    
    if(isZR300)
        device->stop(rs::source::all_sources);
    else
        device->stop();

    return 0;
}

void NodeletCamera::getStreamCalibData(rs::stream stream_index)
{
    rs::intrinsics intrinsic = device->get_stream_intrinsics(stream_index);
    int32_t stream = (int32_t)stream_index;
    camera_info_[stream].header.frame_id = optical_frame_id_[stream];
    camera_info_[stream].width = intrinsic.width;
    camera_info_[stream].height = intrinsic.height;

    camera_info_[stream].K.at(0) = intrinsic.fx;
    camera_info_[stream].K.at(2) = intrinsic.ppx;
    camera_info_[stream].K.at(4) = intrinsic.fy;
    camera_info_[stream].K.at(5) = intrinsic.ppy;
    camera_info_[stream].K.at(8) = 1;

    camera_info_[stream].P.at(0) = camera_info_->K.at(0);
    camera_info_[stream].P.at(1) = 0;
    camera_info_[stream].P.at(2) = camera_info_->K.at(2);
    camera_info_[stream].P.at(3) = 0;
    camera_info_[stream].P.at(4) = 0;
    camera_info_[stream].P.at(5) = camera_info_->K.at(4);
    camera_info_[stream].P.at(6) = camera_info_->K.at(5);
    camera_info_[stream].P.at(7) = 0;
    camera_info_[stream].P.at(8) = 0;
    camera_info_[stream].P.at(9) = 0;
    camera_info_[stream].P.at(10) = 1;
    camera_info_[stream].P.at(11) = 0;
    
    if (stream_index == rs::stream::depth)
    {
        // set depth to color translation values in Projection matrix (P)
        rs::extrinsics extrinsic = device->get_extrinsics(rs::stream::depth, rs::stream::color);
        camera_info_[stream].P.at(3) = extrinsic.translation[0];     // Tx
        camera_info_[stream].P.at(7) = extrinsic.translation[1];     // Ty
        camera_info_[stream].P.at(11) = extrinsic.translation[2];    // Tz

        for (int i = 0; i < 9; i++)
            camera_info_[stream].R.at(i) = extrinsic.rotation[i];
    }
    
    ROS_INFO_STREAM(" model "<<(int32_t)intrinsic.model()<<" stream index "<<stream_index);
    
    switch((int32_t)intrinsic.model())
    {
    case 0:
        camera_info_[stream].distortion_model = "none";
        break;
    case 1:
        camera_info_[stream].distortion_model = "modified_brown_conrady";
        break;
    case 2:
        camera_info_[stream].distortion_model = "inverse_brown_conrady";
        break;
    case 3:
        camera_info_[stream].distortion_model = "distortion_ftheta";
        break;
    default:
        camera_info_[stream].distortion_model = "others";
        break;
    }
    
    // set R (rotation matrix) values to identity matrix
    if (stream_index != rs::stream::depth)
    {
        camera_info_[stream].R.at(0) = 1.0;
        camera_info_[stream].R.at(1) = 0.0;
        camera_info_[stream].R.at(2) = 0.0;
        camera_info_[stream].R.at(3) = 0.0;
        camera_info_[stream].R.at(4) = 1.0;
        camera_info_[stream].R.at(5) = 0.0;
        camera_info_[stream].R.at(6) = 0.0;
        camera_info_[stream].R.at(7) = 0.0;
        camera_info_[stream].R.at(8) = 1.0;
    }
    
    for (int i = 0; i < 5; i++)
    {
        camera_info_[stream].D.push_back(intrinsic.coeffs[i]);
    }
}
}//end namespace

