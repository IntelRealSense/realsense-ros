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


// R200 and ZR300 types
std::map<rs::stream,image_transport::Publisher> image_publishers_;
std::map<rs::stream,int> image_format_;
std::map<rs::stream,rs::format> format_;
std::map<rs::stream,ros::Publisher> info_publisher_;
std::map<rs::stream,cv::Mat> image_;
std::map<rs::stream,std::string> encoding_;
std::map<rs::stream,std::string> optical_frame_id_;
std::string optical_frame_id_accel;
std::string optical_frame_id_gyro;
std::map<rs::stream,int> seq_;
std::map<rs::stream,int> unit_step_size_;
std::map<rs::stream,std::function<void(rs::frame)>> stream_callback_per_stream; 
std::map<rs::stream,sensor_msgs::CameraInfo> camera_info_;

// ZR300 specific types
bool isZR300 = false;
ros::Publisher accelInfo_publisher_, gyroInfo_publisher_, fe2imu_publisher_, fe2depth_publisher_;
ros::Publisher imu_publishers_[2] = {};//accel 0 gyro 1
int seq_motion[2]= {0,0};
std::string optical_imu_id_[2] = {"imu_accel_frame_id","imu_gyro_frame_id"};


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
        NodeletCamera() :
            cameraStarted(false)
        {
            // libRealsense format types for stream
            format_[rs::stream::depth] = rs::format::z16;
            format_[rs::stream::color] = rs::format::rgb8;
            format_[rs::stream::fisheye] = rs::format::raw8;

            // CVBridge native image types
            image_format_[rs::stream::depth] = CV_16UC1;
            image_format_[rs::stream::color] = CV_8UC3;
            image_format_[rs::stream::fisheye] = CV_8UC1;

            // ROS sensor_msgs::ImagePtr encoding types
            encoding_[rs::stream::depth] = sensor_msgs::image_encodings::TYPE_16UC1;
            encoding_[rs::stream::color] = sensor_msgs::image_encodings::RGB8;
            encoding_[rs::stream::fisheye] = sensor_msgs::image_encodings::TYPE_8UC1;

            // ROS sensor_msgs::ImagePtr row step sizes
            unit_step_size_[rs::stream::depth] = sizeof(uint16_t);
            unit_step_size_[rs::stream::color] = sizeof(unsigned char) * 3;
            unit_step_size_[rs::stream::fisheye] = sizeof(unsigned char);

            stream_name_[rs::stream::depth] = "depth";
            stream_name_[rs::stream::color] = "color";
            stream_name_[rs::stream::fisheye] = "fisheye";
        }

        virtual ~NodeletCamera()
        {        
            if(cameraStarted)
            {
                if(isZR300)
                    device->stop(rs::source::all_sources);
                else
                    device->stop();
            }      
            ctx.reset();
        }

    private:
        virtual void onInit()
        {
            getParameters();
            
            if(false == setupDevice())
                return;

            setupPublishers();        
            setupStreams();
        }//end onInit


        void getParameters()
        {
          pnh_ = getPrivateNodeHandle();

          pnh_.param("serial_no", serial_no, DEFAULT_SERIAL_NO);

          pnh_.param("depth_width", width_[rs::stream::depth], DEPTH_WIDTH);
          pnh_.param("depth_height", height_[rs::stream::depth], DEPTH_HEIGHT);
          pnh_.param("depth_fps", fps_[rs::stream::depth], DEPTH_FPS);
          pnh_.param("enable_depth", enable_[rs::stream::depth], true);

          pnh_.param("color_width", width_[rs::stream::color], COLOR_WIDTH);
          pnh_.param("color_height", height_[rs::stream::color], COLOR_HEIGHT);
          pnh_.param("color_fps", fps_[rs::stream::color], COLOR_FPS);
          pnh_.param("enable_color", enable_[rs::stream::color], false);

          pnh_.param("fisheye_width", width_[rs::stream::fisheye], FISHEYE_WIDTH);
          pnh_.param("fisheye_height", height_[rs::stream::fisheye], FISHEYE_HEIGHT);
          pnh_.param("fisheye_fps", fps_[rs::stream::fisheye], FISHEYE_FPS);
          pnh_.param("enable_fisheye", enable_[rs::stream::fisheye], false);

          pnh_.param("depth_optical_frame_id", optical_frame_id_[rs::stream::depth], DEFAULT_DEPTH_OPTICAL_FRAME_ID);
          pnh_.param("color_optical_frame_id", optical_frame_id_[rs::stream::color], DEFAULT_COLOR_OPTICAL_FRAME_ID);
          pnh_.param("fisheye_optical_frame_id", optical_frame_id_[rs::stream::fisheye], DEFAULT_FISHEYE_OPTICAL_FRAME_ID);
          pnh_.param("accel_optical_frame_id", optical_frame_id_accel, DEFAULT_ACCEL_OPTICAL_FRAME_ID);
          pnh_.param("gyro_optical_frame_id", optical_frame_id_gyro, DEFAULT_GYRO_OPTICAL_FRAME_ID);
        }//end getParameters


        bool setupDevice()
        {
            ctx.reset(new rs::context());
            int num_of_cams = ctx -> get_device_count();
            if (num_of_cams == 0)
            {
                ROS_ERROR("error : no RealSense R200, LR200, or ZR300 devices found.");
                ctx.reset();
                return false;
            }

            rs::device *detected_dev;
            for (int i = 0; i < num_of_cams; i++)
            {
                detected_dev = ctx->get_device(i);
                detected_dev->get_serial();
                if(serial_no.empty() || (serial_no == std::string(detected_dev->get_serial())))
                {
                    device = detected_dev;
                    break;
                }
            }

            if (device == nullptr)
            {
                ROS_ERROR_STREAM("error: No RealSense device with serial_no = " << serial_no << " found.");
                ctx.reset();
                return false;
            }

            auto device_name = device -> get_name();
            ROS_INFO_STREAM(device_name << ", serial_no: " << std::string(device ->get_serial()));
            if (std::string(device_name).find("ZR300") == std::string::npos) 
            {
                isZR300 = false;
                enable_[rs::stream::fisheye] = false;

                if ((std::string(device_name).find("R200") == std::string::npos) && (std::string(device_name).find("LR200") == std::string::npos))
                {
                    ROS_ERROR_STREAM("error: This ROS node supports R200, LR200, and ZR300 only.  See https://github.com/intel-ros/realsense for F200 and SR300 support.");
                    ctx.reset();
                    return false;
                }
            }
            else 
            {
                // Only enable ZR300 functionality if fisheye stream is enabled.  Accel/Gyro automatically enabled when fisheye requested
                if( true == enable_[rs::stream::fisheye])
                    isZR300 = true;
            }   

            return true; 
        }//end setupDevice


        void setupPublishers()
        {
            image_transport::ImageTransport image_transport(getNodeHandle());

            // Stream publishers and latched topics
            if(true == enable_[rs::stream::color])
            {
                image_publishers_[rs::stream::color] = image_transport.advertise("camera/color/image_raw", 1);
                info_publisher_[rs::stream::color] = node_handle.advertise< sensor_msgs::CameraInfo >("camera/color/camera_info", 1);
            }

            if(true == enable_[rs::stream::depth])
            {
                image_publishers_[rs::stream::depth] = image_transport.advertise("camera/depth/image_raw", 1);
                info_publisher_[rs::stream::depth] = node_handle.advertise< sensor_msgs::CameraInfo >("camera/depth/camera_info", 1);
            }
           
            if (isZR300)
            {
                // Stream publishers
                image_publishers_[rs::stream::fisheye] = image_transport.advertise("camera/fisheye/image_raw", 1);
                info_publisher_[rs::stream::fisheye] = node_handle.advertise< sensor_msgs::CameraInfo >("camera/fisheye/camera_info", 1);

                imu_publishers_[RS_EVENT_IMU_GYRO] = node_handle.advertise< sensor_msgs::Imu >("camera/gyro/sample", 100); // TODO: review queue size
                imu_publishers_[RS_EVENT_IMU_ACCEL] = node_handle.advertise< sensor_msgs::Imu >("camera/accel/sample", 100); // TODO: review queue size
                
                // Latched topics            
                fe2imu_publisher_ = node_handle.advertise< Extrinsics >("camera/extrinsics/fisheye2imu", 1, true);
                fe2depth_publisher_ = node_handle.advertise< Extrinsics >("camera/extrinsics/fisheye2depth", 1, true);
                accelInfo_publisher_ = node_handle.advertise< IMUInfo >("camera/accel/imu_info", 1, true);
                gyroInfo_publisher_ = node_handle.advertise< IMUInfo >("camera/gyro/imu_info", 1, true);
            }
        }//end setupPublishers


        void setupStreams()
        {
            device->set_option(rs::option::r200_lr_auto_exposure_enabled, 1);

            const rs::stream All[] = { rs::stream::depth, rs::stream::color, rs::stream::fisheye };
            for ( const auto stream : All )
            {
                if( false == enable_[stream])
                    continue;

                // Define lambda callback for receiving stream data
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

                    image_[stream].data = (unsigned char *) frame.get_data();

                    sensor_msgs::ImagePtr img;
                    img = cv_bridge::CvImage(std_msgs::Header(), encoding_[stream], image_[stream]).toImageMsg();
                    img->width = image_[stream].cols;
                    img->height = image_[stream].rows;
                    img->is_bigendian = false;
                    img->step = image_[stream].cols * unit_step_size_[stream];
                    seq_[stream] += 1;

                    img->header.frame_id = optical_frame_id_[stream];
                    img->header.stamp = ros::Time(frame.get_timestamp());
                    img->header.seq = seq_[stream];
                    image_publishers_[stream].publish(img);

                    camera_info_[stream].header.stamp = img->header.stamp;
                    camera_info_[stream].header.seq = img->header.seq;
                    info_publisher_[stream].publish(camera_info_[stream]);
                };

                // Enable the stream
                device->enable_stream(stream, width_[stream], height_[stream], format_[stream], fps_[stream]);

                // Publish info about the stream
                getStreamCalibData(stream);

                // Setup stream callback for stream
                image_[stream] = cv::Mat(camera_info_[stream].height, camera_info_[stream].width, image_format_[stream], cv::Scalar(0,0,0));
                device->set_frame_callback(stream, stream_callback_per_stream[stream]);

                ROS_INFO_STREAM("  enabled " << stream_name_[stream] << " stream, width: " << camera_info_[stream].width << " height: " << camera_info_[stream].height << " fps: " << fps_[stream]);
            }//end for
            

            if(isZR300) {
                device->set_option(rs::option::fisheye_strobe, 1); // Needed to align image timestamps to common clock-domain with the motion events
                device->set_option(rs::option::fisheye_external_trigger, 1); // This option causes the fisheye image to be aquired in-sync with the depth image.
                device->set_option(rs::option::fisheye_color_auto_exposure, 1);
                
                //define callback to the motion events and set it.
                std::function<void(rs::motion_data)> motion_callback;
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
                
                std::function<void(rs::timestamp_data)> timestamp_callback;
                timestamp_callback = [](rs::timestamp_data entry) {};

                device->enable_motion_tracking(motion_callback, timestamp_callback);
                ROS_INFO_STREAM("  enabled accel and gyro stream" );

                // publish ZR300-specific intrinsics/extrinsics
                IMUInfo accelInfo, gyroInfo;
                getImuInfo(device, accelInfo, gyroInfo);
                
                fe2imu_publisher_.publish(getFisheye2ImuExtrinsicsMsg(device));
                fe2depth_publisher_.publish(getFisheye2DepthExtrinsicsMsg(device));
                accelInfo_publisher_.publish(accelInfo);
                gyroInfo_publisher_.publish(gyroInfo);
            }
                        
            if(isZR300)
                device->start(rs::source::all_sources);
            else
                device->start();
            cameraStarted = true;            
        }//end setupStreams


        void getStreamCalibData(rs::stream stream)
        {
            rs::intrinsics intrinsic = device->get_stream_intrinsics(stream);

            camera_info_[stream].header.frame_id = optical_frame_id_[stream];
            camera_info_[stream].width = intrinsic.width;
            camera_info_[stream].height = intrinsic.height;

            camera_info_[stream].K.at(0) = intrinsic.fx;
            camera_info_[stream].K.at(2) = intrinsic.ppx;
            camera_info_[stream].K.at(4) = intrinsic.fy;
            camera_info_[stream].K.at(5) = intrinsic.ppy;
            camera_info_[stream].K.at(8) = 1;

            camera_info_[stream].P.at(0) = camera_info_[stream].K.at(0);
            camera_info_[stream].P.at(1) = 0;
            camera_info_[stream].P.at(2) = camera_info_[stream].K.at(2);
            camera_info_[stream].P.at(3) = 0;
            camera_info_[stream].P.at(4) = 0;
            camera_info_[stream].P.at(5) = camera_info_[stream].K.at(4);
            camera_info_[stream].P.at(6) = camera_info_[stream].K.at(5);
            camera_info_[stream].P.at(7) = 0;
            camera_info_[stream].P.at(8) = 0;
            camera_info_[stream].P.at(9) = 0;
            camera_info_[stream].P.at(10) = 1;
            camera_info_[stream].P.at(11) = 0;
            
            if (stream == rs::stream::depth)
            {
                // set depth to color translation values in Projection matrix (P)
                rs::extrinsics extrinsic = device->get_extrinsics(rs::stream::depth, rs::stream::color);
                camera_info_[stream].P.at(3) = extrinsic.translation[0];     // Tx
                camera_info_[stream].P.at(7) = extrinsic.translation[1];     // Ty
                camera_info_[stream].P.at(11) = extrinsic.translation[2];    // Tz

                for (int i = 0; i < 9; i++)
                    camera_info_[stream].R.at(i) = extrinsic.rotation[i];
            }
            
            switch((int32_t)intrinsic.model())
            {
            case 0:
                camera_info_[stream].distortion_model = "none";
                break;
            case 1:
                camera_info_[stream].distortion_model = "plumb_bob"; // This is the same as "modified_brown_conrady", but used by ROS
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
            if (stream != rs::stream::depth)
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
        }//end getStreamCalibData

    private:
        ros::NodeHandle node_handle, pnh_;
        bool cameraStarted;
        std::unique_ptr< rs::context > ctx; 
        rs::device *device;

        std::string serial_no;
        std::string usb_port_id_;
        std::string camera_type_;

        std::map<rs::stream,int> width_;
        std::map<rs::stream,int> height_;
        std::map<rs::stream,int> fps_;
        std::map<rs::stream,bool> enable_; 
        std::map<rs::stream,std::string> stream_name_;
    };//end class

PLUGINLIB_DECLARE_CLASS(realsense_ros_camera, NodeletCamera, realsense_ros_camera::NodeletCamera, nodelet::Nodelet);

}//end namespace

