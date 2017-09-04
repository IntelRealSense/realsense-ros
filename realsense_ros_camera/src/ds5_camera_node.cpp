// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved

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
#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/version.hpp>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <constants.h>
#include <realsense_ros_camera/Extrinsics.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <sensor_msgs/Imu.h>
#include <realsense_ros_camera/IMUInfo.h>
#include <csignal>

namespace realsense_ros_camera
{
    using stream_index_pair = std::pair<rs2_stream, int>;

    const stream_index_pair COLOR{RS2_STREAM_COLOR, 0};
    const stream_index_pair DEPTH{RS2_STREAM_DEPTH, 0};
    const stream_index_pair INFRA1{RS2_STREAM_INFRARED, 1};
    const stream_index_pair INFRA2{RS2_STREAM_INFRARED, 2};
    const stream_index_pair FISHEYE{RS2_STREAM_FISHEYE, 0};
    const stream_index_pair GYRO{RS2_STREAM_GYRO, 0};
    const stream_index_pair ACCEL{RS2_STREAM_ACCEL, 0};

    const std::vector<std::vector<stream_index_pair>> IMAGE_STREAMS = {{{DEPTH, INFRA1, INFRA2},
                                                                        {COLOR},
                                                                        {FISHEYE}}};

    const std::vector<std::vector<stream_index_pair>> HID_STREAMS = {{GYRO, ACCEL}};

    inline void signalHandler(int signum)
    {
        std::cout << "\n\"Ctrl+C\" is being pressed! Terminate RealSense Node...\n";
        ros::shutdown();
        exit(signum);
    }

    class DS5NodeletCamera: public nodelet::Nodelet
    {
    public:
        DS5NodeletCamera() :
            _cameraStarted(false),
            _base_frame_id(""),
            _intialize_time_base(false)
        {
            signal(SIGINT, signalHandler);
            rs2::log_to_console(rs2_log_severity::RS2_LOG_SEVERITY_ERROR);
            ROS_INFO("Running with LibRealSense v%s", RS2_API_VERSION_STR);

            // Types for depth stream
            _format[DEPTH] = RS2_FORMAT_Z16;   // libRS type
            _image_format[DEPTH] = CV_16UC1;    // CVBridge type
            _encoding[DEPTH] = sensor_msgs::image_encodings::TYPE_16UC1; // ROS message type
            _unit_step_size[DEPTH] = sizeof(uint16_t); // sensor_msgs::ImagePtr row step size
            _stream_name[DEPTH] = "depth";

            // Infrared stream - Left
            _format[INFRA1] = RS2_FORMAT_Y8;   // libRS type
            _image_format[INFRA1] = CV_8UC1;    // CVBridge type
            _encoding[INFRA1] = sensor_msgs::image_encodings::TYPE_8UC1; // ROS message type
            _unit_step_size[INFRA1] = sizeof(uint8_t); // sensor_msgs::ImagePtr row step size
            _stream_name[INFRA1] = "infra1";

            // Infrared stream - Right
            _format[INFRA2] = RS2_FORMAT_Y8;   // libRS type
            _image_format[INFRA2] = CV_8UC1;    // CVBridge type
            _encoding[INFRA2] = sensor_msgs::image_encodings::TYPE_8UC1; // ROS message type
            _unit_step_size[INFRA2] = sizeof(uint8_t); // sensor_msgs::ImagePtr row step size
            _stream_name[INFRA2] = "infra2";

            // Types for color stream
            _format[COLOR] = RS2_FORMAT_RGB8;   // libRS type
            _image_format[COLOR] = CV_8UC3;    // CVBridge type
            _encoding[COLOR] = sensor_msgs::image_encodings::TYPE_8UC3; // ROS message type
            _unit_step_size[COLOR] = 3; // sensor_msgs::ImagePtr row step size
            _stream_name[COLOR] = "color";

            // Types for fisheye stream
            _format[FISHEYE] = RS2_FORMAT_RAW8;   // libRS type
            _image_format[FISHEYE] = CV_8UC1;    // CVBridge type
            _encoding[FISHEYE] = sensor_msgs::image_encodings::TYPE_8UC1; // ROS message type
            _unit_step_size[FISHEYE] = sizeof(uint8_t); // sensor_msgs::ImagePtr row step size
            _stream_name[FISHEYE] = "fisheye";

            // Types for Motion-Module streams
            _format[GYRO] = RS2_FORMAT_MOTION_XYZ32F;   // libRS type
            _image_format[GYRO] = CV_8UC1;    // CVBridge type
            _encoding[GYRO] = sensor_msgs::image_encodings::TYPE_8UC1; // ROS message type
            _unit_step_size[GYRO] = sizeof(uint8_t); // sensor_msgs::ImagePtr row step size
            _stream_name[GYRO] = "gyro";

            _format[ACCEL] = RS2_FORMAT_MOTION_XYZ32F;   // libRS type
            _image_format[ACCEL] = CV_8UC1;    // CVBridge type
            _encoding[ACCEL] = sensor_msgs::image_encodings::TYPE_8UC1; // ROS message type
            _unit_step_size[ACCEL] = sizeof(uint8_t); // sensor_msgs::ImagePtr row step size
            _stream_name[ACCEL] = "accel";
        }

        virtual ~DS5NodeletCamera()
        {
            try{
                ROS_INFO("~DS5NodeletCamera()");
                if (_cameraStarted)
                {
                    bool is_imu_stopped = false;
                    for (auto& kvp: _sensors)
                    {
                        if (false == _enable[kvp.first])
                            continue;

                        if (is_imu_stopped &&
                            ((ACCEL == kvp.first) ||
                             (GYRO == kvp.first)))
                        {
                            continue;
                        }
                        else if ((ACCEL == kvp.first) ||
                                 (GYRO == kvp.first))
                        {
                            is_imu_stopped = true;
                        }

                        if (is_imu_stopped &&
                            ((DEPTH == kvp.first) ||
                             (INFRA1 == kvp.first) ||
                             (INFRA2 == kvp.first)))
                        {
                            continue;
                        }

                        kvp.second->stop();
                        kvp.second->close();
                    }
                }
                usleep(250);
                _ctx.reset();
            }
            catch(const std::exception& ex)
            {
                ROS_ERROR_STREAM("An exception has been thrown: " << ex.what());
            }
            catch(...)
            {
                ROS_ERROR_STREAM("An unknown error has occurred!");
            }
        }

    private:
        virtual void onInit()
        {
            getParameters();
            setupDevice();
            setupPublishers();
            setupStreams();
            publishStaticTransforms();
            ROS_INFO_STREAM("RealSense Node Is Up!");
        }

        void getParameters()
        {
            ROS_INFO("getParameters...");

            _pnh = getPrivateNodeHandle();

            _pnh.param("serial_no", _serial_no, DEFAULT_SERIAL_NO);

            _pnh.param("depth_width", _width[DEPTH], DEPTH_WIDTH);
            _pnh.param("depth_height", _height[DEPTH], DEPTH_HEIGHT);
            _pnh.param("depth_fps", _fps[DEPTH], DEPTH_FPS);
            _pnh.param("enable_depth", _enable[DEPTH], ENABLE_DEPTH);

            _pnh.param("infra1_width", _width[INFRA1], INFRA1_WIDTH);
            _pnh.param("infra1_height", _height[INFRA1], INFRA1_HEIGHT);
            _pnh.param("infra1_fps", _fps[INFRA1], INFRA1_FPS);
            _pnh.param("enable_infra1", _enable[INFRA1], ENABLE_INFRA1);

            _pnh.param("infra2_width", _width[INFRA2], INFRA2_WIDTH);
            _pnh.param("infra2_height", _height[INFRA2], INFRA2_HEIGHT);
            _pnh.param("infra2_fps", _fps[INFRA2], INFRA2_FPS);
            _pnh.param("enable_infra2", _enable[INFRA2], ENABLE_INFRA2);

            _pnh.param("color_width", _width[COLOR], COLOR_WIDTH);
            _pnh.param("color_height", _height[COLOR], COLOR_HEIGHT);
            _pnh.param("color_fps", _fps[COLOR], COLOR_FPS);
            _pnh.param("enable_color", _enable[COLOR], ENABLE_COLOR);

            _pnh.param("fisheye_width", _width[FISHEYE], FISHEYE_WIDTH);
            _pnh.param("fisheye_height", _height[FISHEYE], FISHEYE_HEIGHT);
            _pnh.param("fisheye_fps", _fps[FISHEYE], FISHEYE_FPS);
            _pnh.param("enable_fisheye", _enable[FISHEYE], ENABLE_FISHEYE);

            _pnh.param("gyro_fps", _fps[GYRO], GYRO_FPS);
            _pnh.param("accel_fps", _fps[ACCEL], ACCEL_FPS);
            _pnh.param("enable_imu", _enable[GYRO], ENABLE_IMU);
            _pnh.param("enable_imu", _enable[ACCEL], ENABLE_IMU);

            _pnh.param("base_frame_id", _base_frame_id, DEFAULT_BASE_FRAME_ID);
            _pnh.param("depth_frame_id", _frame_id[DEPTH], DEFAULT_DEPTH_FRAME_ID);
            _pnh.param("infra1_frame_id", _frame_id[INFRA1], DEFAULT_INFRA1_FRAME_ID);
            _pnh.param("infra2_frame_id", _frame_id[INFRA2], DEFAULT_INFRA2_FRAME_ID);
            _pnh.param("color_frame_id", _frame_id[COLOR], DEFAULT_COLOR_FRAME_ID);
            _pnh.param("fisheye_frame_id", _frame_id[FISHEYE], DEFAULT_FISHEYE_FRAME_ID);
            _pnh.param("imu_gyro_frame_id", _frame_id[GYRO], DEFAULT_IMU_FRAME_ID);
            _pnh.param("imu_accel_frame_id", _frame_id[ACCEL], DEFAULT_IMU_FRAME_ID);

            _pnh.param("depth_optical_frame_id", _optical_frame_id[DEPTH], DEFAULT_DEPTH_OPTICAL_FRAME_ID);
            _pnh.param("infra1_optical_frame_id", _optical_frame_id[INFRA1], DEFAULT_INFRA1_OPTICAL_FRAME_ID);
            _pnh.param("infra2_optical_frame_id", _optical_frame_id[INFRA2], DEFAULT_INFRA2_OPTICAL_FRAME_ID);
            _pnh.param("color_optical_frame_id", _optical_frame_id[COLOR], DEFAULT_COLOR_OPTICAL_FRAME_ID);
            _pnh.param("fisheye_optical_frame_id", _optical_frame_id[FISHEYE], DEFAULT_FISHEYE_OPTICAL_FRAME_ID);
            _pnh.param("gyro_optical_frame_id", _optical_frame_id[GYRO], DEFAULT_GYRO_OPTICAL_FRAME_ID);
            _pnh.param("accel_optical_frame_id", _optical_frame_id[ACCEL], DEFAULT_ACCEL_OPTICAL_FRAME_ID);
        }

        void setupDevice()
        {
            ROS_INFO("setupDevice...");
            try{
                _ctx.reset(new rs2::context());

                auto list = _ctx->query_devices();
                if (0 == list.size())
                {
                    _ctx.reset();
                    ROS_ERROR("No RealSense devices were found!");
                    ros::shutdown();
                    return;
                }

                // Take the first device in the list.
                // TODO: Add an ability to get the specific device to work with from outside
                _dev = list[0];
                _ctx->set_devices_changed_callback([this](rs2::event_information& info)
                {
                    if (info.was_removed(_dev))
                    {
                        ROS_FATAL("The device has been disconnected!");
                        ros::shutdown();
                    }
                });

                _serial_no = _dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
                ROS_INFO_STREAM("Device Serial No: " << _serial_no);
                auto dev_sensors = _dev.query_sensors();

                ROS_INFO_STREAM("Device Sensors: ");
                for(auto&& elem : dev_sensors)
                {
                    std::string module_name = elem.get_info(RS2_CAMERA_INFO_NAME);
                    if ("Stereo Module" == module_name)
                    {
                        auto sen = new rs2::sensor(elem);
                        _sensors[DEPTH] = std::unique_ptr<rs2::sensor>(sen);
                        _sensors[INFRA1] = std::unique_ptr<rs2::sensor>(sen);
                        _sensors[INFRA2] = std::unique_ptr<rs2::sensor>(sen);
                    }
                    else if ("RGB Camera" == module_name)
                    {
                        _sensors[COLOR] = std::unique_ptr<rs2::sensor>(new rs2::sensor(elem));
                    }
                    else if ("Wide FOV Camera" == module_name)
                    {
                        _sensors[FISHEYE] = std::unique_ptr<rs2::sensor>(new rs2::sensor(elem));
                    }
                    else if ("Motion Module" == module_name)
                    {
                        auto hid_sensor = new rs2::sensor(elem);
                        _sensors[GYRO] = std::unique_ptr<rs2::sensor>(hid_sensor);
                        _sensors[ACCEL] = std::unique_ptr<rs2::sensor>(hid_sensor);
                    }
                    else
                    {
                        ROS_ERROR_STREAM("Module Name \"" << module_name << "\" isn't supported by LibRealSense!");
                        ros::shutdown();
                        return;
                    }
                    ROS_INFO_STREAM(std::string(elem.get_info(RS2_CAMERA_INFO_NAME)) << " was found.");
                }

                // Update "enable" map
                std::vector<std::vector<stream_index_pair>> streams(IMAGE_STREAMS);
                streams.insert(streams.end(), HID_STREAMS.begin(), HID_STREAMS.end());
                for (auto elem : streams)
                {
                    for (auto& stream_index : elem)
                    {
                        if (true == _enable[stream_index] && _sensors.find(stream_index) == _sensors.end()) // check if device supports the enabled stream
                        {
                            ROS_INFO_STREAM(rs2_stream_to_string(stream_index.first) << " stream isn't supported by current device! -- Skipping...");
                            _enable[stream_index] = false;
                        }
                    }
                }
            }
            catch(const std::exception& ex)
            {
                ROS_ERROR_STREAM("An exception has been thrown: " << ex.what());
                throw;
            }
        }

        void setupPublishers()
        {
            ROS_INFO("setupPublishers...");
            image_transport::ImageTransport image_transport(getNodeHandle());

            if (true == _enable[DEPTH])
            {
                _image_publishers[DEPTH] = image_transport.advertise("camera/depth/image_raw", 1);
                _info_publisher[DEPTH] = _node_handle.advertise<sensor_msgs::CameraInfo>("camera/depth/camera_info", 1);
                _pointcloud_publisher = _node_handle.advertise<sensor_msgs::PointCloud2>("/camera/points", 1);
            }

            if (true == _enable[INFRA1])
            {
                _image_publishers[INFRA1] = image_transport.advertise("camera/infra1/image_raw", 1);
                _info_publisher[INFRA1] = _node_handle.advertise<sensor_msgs::CameraInfo>("camera/infra1/camera_info", 1);
                _pointcloud_publisher = _node_handle.advertise<sensor_msgs::PointCloud2>("/camera/points", 1);
            }

            if (true == _enable[INFRA2])
            {
                _image_publishers[INFRA2] = image_transport.advertise("camera/infra2/image_raw", 1);
                _info_publisher[INFRA2] = _node_handle.advertise<sensor_msgs::CameraInfo>("camera/infra2/camera_info", 1);
                _pointcloud_publisher = _node_handle.advertise<sensor_msgs::PointCloud2>("/camera/points", 1);
            }

            if (true == _enable[COLOR])
            {
                _image_publishers[COLOR] = image_transport.advertise("camera/color/image_raw", 1);
                _info_publisher[COLOR] = _node_handle.advertise<sensor_msgs::CameraInfo>("camera/color/camera_info", 1);
            }

            if (true == _enable[FISHEYE] &&
                    true == _enable[DEPTH])
            {
                _image_publishers[FISHEYE] = image_transport.advertise("camera/fisheye/image_raw", 1);
                _info_publisher[FISHEYE] = _node_handle.advertise<sensor_msgs::CameraInfo>("camera/fisheye/camera_info", 1);
                _fe_to_depth_publisher = _node_handle.advertise<Extrinsics>("camera/extrinsics/fisheye2depth", 1, true);
            }

            if (true == _enable[GYRO])
            {
                _imu_publishers[GYRO] = _node_handle.advertise<sensor_msgs::Imu>("camera/gyro/sample", 100);
                _info_publisher[GYRO] = _node_handle.advertise<IMUInfo>("camera/gyro/imu_info", 1, true);
            }

            if (true == _enable[ACCEL])
            {
                _imu_publishers[ACCEL] = _node_handle.advertise<sensor_msgs::Imu>("camera/accel/sample", 100);
                _info_publisher[ACCEL] = _node_handle.advertise<IMUInfo>("camera/accel/imu_info", 1, true);
            }

            if (true == _enable[FISHEYE] &&
                (true == _enable[GYRO] ||
                 true == _enable[ACCEL]))
            {
                _fe_to_imu_publisher = _node_handle.advertise<Extrinsics>("camera/extrinsics/fisheye2imu", 1, true);
            }
        }

        void setupStreams()
        {
            ROS_INFO("setupStreams...");
            try{
                for (auto& streams : IMAGE_STREAMS)
                {
                    for (auto& elem : streams)
                    {
                        if (true == _enable[elem])
                        {
                            auto& sens = _sensors[elem];
                            auto profiles = sens->get_stream_profiles();
                            for (rs2::stream_profile& profile : profiles)
                            {
                                auto video_profile = profile.as<rs2::video_stream_profile>();
                                if (video_profile.format() == _format[elem] &&
                                    video_profile.width()  == _width[elem] &&
                                    video_profile.height() == _height[elem] &&
                                    video_profile.fps()    == _fps[elem] &&
                                    video_profile.stream_index() == elem.second)
                                {
                                    _enabled_profiles[elem].push_back(profile);
                                    // Publish info about the stream
                                    updateStreamCalibData(video_profile);

                                    // Setup stream callback for stream
                                    _image[elem] = cv::Mat(_width[elem], _height[elem], _image_format[elem], cv::Scalar(0, 0, 0));
                                    ROS_INFO_STREAM(_stream_name[elem] << " stream is enabled - width: " << _width[elem] << ", height: " << _height[elem] << ", fps: " << _fps[elem]);
                                    break;
                                }
                            }
                        }
                    }
                }

                // Streaming IMAGES
                for (auto& streams : IMAGE_STREAMS)
                {
                    std::vector<rs2::stream_profile> profiles;
                    for (auto& elem : streams)
                    {
                        if (!_enabled_profiles[elem].empty())
                        {
                            profiles.insert(profiles.begin(),
                                            _enabled_profiles[elem].begin(),
                                            _enabled_profiles[elem].end());
                        }
                    }

                    if (!profiles.empty())
                    {
                        auto stream = streams.front();
                        auto& sens = _sensors[stream];
                        // Enable the stream
                        sens->open(profiles);

                        // Start device with the following callback
                        sens->start([this](rs2::frame frame)
                        {
                            // We compute a ROS timestamp which is based on an initial ROS time at point of first frame,
                            // and the incremental timestamp from the camera.
                            if (false == _intialize_time_base)
                            {
                                _intialize_time_base = true;
                                _ros_time_base = ros::Time::now();
                                _camera_time_base = frame.get_timestamp();
                            }
                            double elapsed_camera_ms = (/*ms*/ frame.get_timestamp() - /*ms*/ _camera_time_base) / /*ms to seconds*/ 1000;
                            ros::Time t(_ros_time_base.toSec() + elapsed_camera_ms);

                            ROS_DEBUG("Frame arrived: stream: %s ; index: %d", rs2_stream_to_string(frame.get_profile().stream_type()), frame.get_profile().stream_index());
                            stream_index_pair stream{frame.get_profile().stream_type(), frame.get_profile().stream_index()};
                            _image[stream].data = (uint8_t*)frame.get_data();
                            auto width = 0;
                            auto height = 0;
                            auto bpp = 1;
                            if (frame.is<rs2::video_frame>())
                            {
                                auto image = frame.as<rs2::video_frame>();
                                width = image.get_width();
                                height = image.get_height();
                                bpp = image.get_bytes_per_pixel();
                            }

                            if((DEPTH == stream) && (0 != _pointcloud_publisher.getNumSubscribers()))
                                publishPCTopic(t, frame.get_profile());

                            _seq[stream] += 1;
                            if(0 != _info_publisher[stream].getNumSubscribers() ||
                               0 != _image_publishers[stream].getNumSubscribers())
                            {
                                sensor_msgs::ImagePtr img;
                                img = cv_bridge::CvImage(std_msgs::Header(), _encoding[stream], _image[stream]).toImageMsg();
                                img->width = width;
                                img->height = height;
                                img->is_bigendian = false;
                                img->step = width * bpp;
                                img->header.frame_id = _optical_frame_id[stream];
                                img->header.stamp = t;
                                img->header.seq = _seq[stream];

                                _camera_info[stream].header.stamp = t;
                                _camera_info[stream].header.seq = _seq[stream];
                                _info_publisher[stream].publish(_camera_info[stream]);

                                _image_publishers[stream].publish(img);
                                ROS_DEBUG("Publish stream: %s", rs2_stream_to_string(frame.get_profile().stream_type()));
                            }
                        });
                    }
                }//end for


                // Streaming HID
                for (const auto streams : HID_STREAMS)
                {
                    for (auto& elem : streams)
                    {
                        if (true == _enable[elem])
                        {
                            auto& sens = _sensors[elem];
                            auto profiles = sens->get_stream_profiles();
                            for (rs2::stream_profile& profile : profiles)
                            {
                                if (profile.fps() == _fps[elem] &&
                                        profile.format() == _format[elem])
                                {
                                    _enabled_profiles[elem].push_back(profile);
                                    break;
                                }
                            }
                        }
                    }
                }

                auto gyro_profile = _enabled_profiles.find(GYRO);
                auto accel_profile = _enabled_profiles.find(ACCEL);

                if (gyro_profile != _enabled_profiles.end() &&
                    accel_profile != _enabled_profiles.end())
                {
                    std::vector<rs2::stream_profile> profiles;
                    profiles.insert(profiles.begin(), gyro_profile->second.begin(), gyro_profile->second.end());
                    profiles.insert(profiles.begin(), accel_profile->second.begin(), accel_profile->second.end());
                    auto& sens = _sensors[GYRO];
                    sens->open(profiles);

                    sens->start([this](rs2::frame frame){
                        auto stream = frame.get_profile().stream_type();
                        if (false == _intialize_time_base)
                            return;

                        ROS_DEBUG("Frame arrived: stream: %s ; index: %d", rs2_stream_to_string(frame.get_profile().stream_type()), frame.get_profile().stream_index());

                        auto stream_index = (stream == GYRO.first)?GYRO:ACCEL;
                        if (0 != _info_publisher[stream_index].getNumSubscribers() ||
                                0 != _imu_publishers[stream_index].getNumSubscribers())
                        {
                            double elapsed_camera_ms = (/*ms*/ frame.get_timestamp() - /*ms*/ _camera_time_base) / /*ms to seconds*/ 1000;
                            ros::Time t(_ros_time_base.toSec() + elapsed_camera_ms);

                            auto imu_msg = sensor_msgs::Imu();
                            imu_msg.header.frame_id = _optical_frame_id[stream_index];
                            imu_msg.orientation.x = 0.0;
                            imu_msg.orientation.y = 0.0;
                            imu_msg.orientation.z = 0.0;
                            imu_msg.orientation.w = 0.0;
                            imu_msg.orientation_covariance = { -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

                            auto axes = *(reinterpret_cast<const float3*>(frame.get_data()));
                            if (GYRO == stream_index)
                            {
                                imu_msg.angular_velocity.x = axes.x;
                                imu_msg.angular_velocity.y = axes.y;
                                imu_msg.angular_velocity.z = axes.z;
                            }
                            else if (ACCEL == stream_index)
                            {
                                imu_msg.linear_acceleration.x = axes.x;
                                imu_msg.linear_acceleration.y = axes.y;
                                imu_msg.linear_acceleration.z = axes.z;
                            }
                            _seq[stream_index] += 1;
                            imu_msg.header.seq = _seq[stream_index];
                            imu_msg.header.stamp = t;
                            _imu_publishers[stream_index].publish(imu_msg);
                            ROS_DEBUG("Publish stream: %s", rs2_stream_to_string(frame.get_profile().stream_type()));
                        }
                    });

                    if (true == _enable[GYRO])
                    {
                        ROS_INFO_STREAM(_stream_name[GYRO] << " stream is enabled - " << "fps: " << _fps[GYRO]);
                        auto gyroInfo = getImuInfo(GYRO);
                        _info_publisher[GYRO].publish(gyroInfo);
                    }

                    if (true == _enable[ACCEL])
                    {
                        ROS_INFO_STREAM(_stream_name[ACCEL] << " stream is enabled - " << "fps: " << _fps[ACCEL]);
                        auto accelInfo = getImuInfo(ACCEL);
                        _info_publisher[ACCEL].publish(accelInfo);
                    }
                }


                if (true == _enable[DEPTH] &&
                    true == _enable[FISHEYE])
                {
                    auto ex = getFisheye2DepthExtrinsicsMsg();
                    _fe_to_depth_publisher.publish(ex);
                }

                if (true == _enable[FISHEYE] &&
                    (_enable[GYRO] || _enable[ACCEL]))
                {
                    auto ex = getFisheye2ImuExtrinsicsMsg();
                    _fe_to_imu_publisher.publish(ex);
                }

                _cameraStarted = true;
            }
            catch(const std::exception& ex)
            {
                ROS_ERROR_STREAM("An exception has been thrown: " << ex.what());
                throw;
            }

        }

        void updateStreamCalibData(const rs2::video_stream_profile& video_profile)
        {
            stream_index_pair stream_index{video_profile.stream_type(), video_profile.stream_index()};
            auto intrinsic = video_profile.get_intrinsics();

            _camera_info[stream_index].width = intrinsic.width;
            _camera_info[stream_index].height = intrinsic.height;
            _camera_info[stream_index].header.frame_id = _optical_frame_id[stream_index];

            _camera_info[stream_index].K.at(0) = intrinsic.fx;
            _camera_info[stream_index].K.at(2) = intrinsic.ppx;
            _camera_info[stream_index].K.at(4) = intrinsic.fy;
            _camera_info[stream_index].K.at(5) = intrinsic.ppy;
            _camera_info[stream_index].K.at(8) = 1;

            _camera_info[stream_index].P.at(0) = _camera_info[stream_index].K.at(0);
            _camera_info[stream_index].P.at(1) = 0;
            _camera_info[stream_index].P.at(2) = _camera_info[stream_index].K.at(2);
            _camera_info[stream_index].P.at(3) = 0;
            _camera_info[stream_index].P.at(4) = 0;
            _camera_info[stream_index].P.at(5) = _camera_info[stream_index].K.at(4);
            _camera_info[stream_index].P.at(6) = _camera_info[stream_index].K.at(5);
            _camera_info[stream_index].P.at(7) = 0;
            _camera_info[stream_index].P.at(8) = 0;
            _camera_info[stream_index].P.at(9) = 0;
            _camera_info[stream_index].P.at(10) = 1;
            _camera_info[stream_index].P.at(11) = 0;

            _camera_info[stream_index].distortion_model = "plumb_bob";

            // set R (rotation matrix) values to identity matrix
            if (stream_index != DEPTH)
            {
                _camera_info[stream_index].R.at(0) = 1.0;
                _camera_info[stream_index].R.at(1) = 0.0;
                _camera_info[stream_index].R.at(2) = 0.0;
                _camera_info[stream_index].R.at(3) = 0.0;
                _camera_info[stream_index].R.at(4) = 1.0;
                _camera_info[stream_index].R.at(5) = 0.0;
                _camera_info[stream_index].R.at(6) = 0.0;
                _camera_info[stream_index].R.at(7) = 0.0;
                _camera_info[stream_index].R.at(8) = 1.0;
            }

            for (int i = 0; i < 5; i++)
            {
                _camera_info[stream_index].D.push_back(intrinsic.coeffs[i]);
            }
        }

        void publishStaticTransforms()
        {
            ROS_INFO("publishStaticTransforms...");
            // Publish transforms for the cameras
            tf::Quaternion q_c2co;
            tf::Quaternion q_d2do;
            geometry_msgs::TransformStamped b2c_msg;
            geometry_msgs::TransformStamped c2co_msg;
            geometry_msgs::TransformStamped b2d_msg;
            geometry_msgs::TransformStamped d2do_msg;

            // Get the current timestamp for all static transforms
            ros::Time transform_ts_ = ros::Time::now();

            if (true == _enable[COLOR])
            {
                // Transform base frame to depth frame
                b2c_msg.header.stamp = transform_ts_;
                b2c_msg.header.frame_id = _base_frame_id;
                b2c_msg.child_frame_id = _frame_id[COLOR];
                b2c_msg.transform.translation.x = 0;
                b2c_msg.transform.translation.y = 0;
                b2c_msg.transform.translation.z = 0;
                b2c_msg.transform.rotation.x = 0;
                b2c_msg.transform.rotation.y = 0;
                b2c_msg.transform.rotation.z = 0;
                b2c_msg.transform.rotation.w = 1;
                _static_tf_broadcaster.sendTransform(b2c_msg);

                // Transform depth frame to depth optical frame
                q_c2co.setRPY(-M_PI / 2, 0.0, -M_PI / 2);
                c2co_msg.header.stamp = transform_ts_;
                c2co_msg.header.frame_id = _frame_id[COLOR];
                c2co_msg.child_frame_id = _optical_frame_id[COLOR];
                c2co_msg.transform.translation.x = 0;
                c2co_msg.transform.translation.y = 0;
                c2co_msg.transform.translation.z = 0;
                c2co_msg.transform.rotation.x = q_c2co.getX();
                c2co_msg.transform.rotation.y = q_c2co.getY();
                c2co_msg.transform.rotation.z = q_c2co.getZ();
                c2co_msg.transform.rotation.w = q_c2co.getW();
                _static_tf_broadcaster.sendTransform(c2co_msg);
            }

            if (true == _enable[DEPTH])
            {
                // Transform base frame to depth frame
                b2d_msg.header.stamp = transform_ts_;
                b2d_msg.header.frame_id = _base_frame_id;
                b2d_msg.child_frame_id = _frame_id[DEPTH];
                b2d_msg.transform.translation.x = 0;
                b2d_msg.transform.translation.y = 0;
                b2d_msg.transform.translation.z = 0;
                b2d_msg.transform.rotation.x = 0;
                b2d_msg.transform.rotation.y = 0;
                b2d_msg.transform.rotation.z = 0;
                b2d_msg.transform.rotation.w = 1;
                _static_tf_broadcaster.sendTransform(b2d_msg);

                // Transform depth frame to depth optical frame
                q_d2do.setRPY(-M_PI / 2, 0.0, -M_PI / 2);
                d2do_msg.header.stamp = transform_ts_;
                d2do_msg.header.frame_id = _frame_id[DEPTH];
                d2do_msg.child_frame_id = _optical_frame_id[DEPTH];
                d2do_msg.transform.translation.x = 0;
                d2do_msg.transform.translation.y = 0;
                d2do_msg.transform.translation.z = 0;
                d2do_msg.transform.rotation.x = q_d2do.getX();
                d2do_msg.transform.rotation.y = q_d2do.getY();
                d2do_msg.transform.rotation.z = q_d2do.getZ();
                d2do_msg.transform.rotation.w = q_d2do.getW();
                _static_tf_broadcaster.sendTransform(d2do_msg);
            }
        }

        void publishPCTopic(const ros::Time& t, const rs2::stream_profile& profile)
        {
            auto depth_sensor = _sensors[DEPTH]->as<rs2::depth_sensor>();
            auto video_profile = profile.as<rs2::video_stream_profile>();
            auto depth_intrinsic = video_profile.get_intrinsics();
            auto depth_scale_meters = depth_sensor.get_depth_scale();

            sensor_msgs::PointCloud2 msg_pointcloud;
            msg_pointcloud.header.stamp = t;
            msg_pointcloud.header.frame_id = _optical_frame_id[DEPTH];
            msg_pointcloud.width = depth_intrinsic.width;
            msg_pointcloud.height = depth_intrinsic.height;
            msg_pointcloud.is_dense = true;

            sensor_msgs::PointCloud2Modifier modifier(msg_pointcloud);
            modifier.setPointCloud2Fields(3,
                                          "x", 1, sensor_msgs::PointField::FLOAT32,
                                          "y", 1, sensor_msgs::PointField::FLOAT32,
                                          "z", 1, sensor_msgs::PointField::FLOAT32);
            modifier.setPointCloud2FieldsByString(1, "xyz");

            for (int v = 0; v < depth_intrinsic.height; v++)
                for (int u = 0; u < depth_intrinsic.width; u++)
                {
                    float depth_point[3], scaled_depth;
                    uint16_t depth_value;
                    int depth_offset, cloud_offset;

                    // Offset into point cloud data, for point at u, v
                    cloud_offset = (v * msg_pointcloud.row_step) + (u * msg_pointcloud.point_step);

                    // Retrieve depth value, and scale it in terms of meters
                    depth_offset = (u * sizeof(uint16_t)) + (v * sizeof(uint16_t) * depth_intrinsic.width);
                    memcpy(&depth_value, &_image[DEPTH].data[depth_offset], sizeof(uint16_t));
                    scaled_depth = static_cast<float>(depth_value) * depth_scale_meters;
                    if (scaled_depth <= 0.0f || scaled_depth > 40.0)
                    {
                        // Depth value is invalid, so zero it out.
                        depth_point[0] = 0.0f;
                        depth_point[1] = 0.0f;
                        depth_point[2] = 0.0f;
                    }
                    else
                    {
                        // Convert depth image to points in 3D space
                        float depth_pixel[2] = {static_cast<float>(u), static_cast<float>(v)};
                        rs2_deproject_pixel_to_point(depth_point, &depth_intrinsic, depth_pixel, scaled_depth);
                    }

                    // Assign 3d point
                    memcpy(&msg_pointcloud.data[cloud_offset + msg_pointcloud.fields[0].offset], &depth_point[0], sizeof(float)); // X
                    memcpy(&msg_pointcloud.data[cloud_offset + msg_pointcloud.fields[1].offset], &depth_point[1], sizeof(float)); // Y
                    memcpy(&msg_pointcloud.data[cloud_offset + msg_pointcloud.fields[2].offset], &depth_point[2], sizeof(float)); // Z
                } // for

            _pointcloud_publisher.publish(msg_pointcloud);
        }

        Extrinsics rsExtrinsicsToMsg(const rs2_extrinsics& extrinsics)
        {
            Extrinsics extrinsicsMsg;
            for (int i = 0; i < 9; ++i)
            {
                extrinsicsMsg.rotation[i] = extrinsics.rotation[i];
                if (i < 3)
                    extrinsicsMsg.translation[i] = extrinsics.translation[i];
            }

            return extrinsicsMsg;
        }

        Extrinsics getFisheye2ImuExtrinsicsMsg()
        {
            auto& fisheye = _enabled_profiles[FISHEYE].front();
            auto& hid = _enabled_profiles[GYRO].front();
            Extrinsics extrinsicsMsg = rsExtrinsicsToMsg(fisheye.get_extrinsics_to(hid));
            extrinsicsMsg.header.frame_id = "fisheye2imu_extrinsics";
            return extrinsicsMsg;
        }

        Extrinsics getFisheye2DepthExtrinsicsMsg()
        {
            auto& fisheye = _enabled_profiles[FISHEYE].front();
            auto& depth = _enabled_profiles[DEPTH].front();
            Extrinsics extrinsicsMsg = rsExtrinsicsToMsg(fisheye.get_extrinsics_to(depth));
            extrinsicsMsg.header.frame_id = "fisheye2depth_extrinsics";
            return extrinsicsMsg;
        }

        struct float3
        {
            float x, y, z;
        };

        IMUInfo getImuInfo(const stream_index_pair& stream_index)
        {
            IMUInfo info{};
            auto imuIntrinsics = _sensors[stream_index]->get_motion_intrinsics(stream_index.first);
            if (GYRO == stream_index)
            {
                info.header.frame_id = "imu_gyro";
            }
            else if (ACCEL == stream_index)
            {
                info.header.frame_id = "imu_accel";
            }

            auto index = 0;
            for (int i = 0; i < 3; ++i)
            {
                for (int j = 0; j < 4; ++j)
                {
                    info.data[index] = imuIntrinsics.data[i][j];
                    ++index;
                }
                info.noise_variances[i] =  imuIntrinsics.noise_variances[i];
                info.bias_variances[i] = imuIntrinsics.bias_variances[i];
            }
            return info;
        }

        ros::NodeHandle _node_handle, _pnh;
        bool _cameraStarted;
        std::unique_ptr<rs2::context> _ctx;
        rs2::device _dev;

        std::map<stream_index_pair, std::unique_ptr<rs2::sensor>> _sensors;

        std::string _serial_no;
        std::string _dev_location;
        std::string _usb_port_id;
        std::string _camera_type;

        std::map<stream_index_pair, int> _width;
        std::map<stream_index_pair, int> _height;
        std::map<stream_index_pair, int> _fps;
        std::map<stream_index_pair, bool> _enable;
        std::map<stream_index_pair, std::string> _stream_name;
        tf2_ros::StaticTransformBroadcaster _static_tf_broadcaster;

        // DS5 types
        std::map<stream_index_pair, image_transport::Publisher> _image_publishers;
        std::map<stream_index_pair, ros::Publisher> _imu_publishers;
        std::map<stream_index_pair, int> _image_format;
        std::map<stream_index_pair, rs2_format> _format;
        std::map<stream_index_pair, ros::Publisher> _info_publisher;
        std::map<stream_index_pair, cv::Mat> _image;
        std::map<stream_index_pair, std::string> _encoding;
        std::string _base_frame_id;
        std::map<stream_index_pair, std::string> _frame_id;
        std::map<stream_index_pair, std::string> _optical_frame_id;
        std::map<stream_index_pair, int> _seq;
        std::map<stream_index_pair, int> _unit_step_size;
        std::map<stream_index_pair, sensor_msgs::CameraInfo> _camera_info;
        ros::Publisher _fe_to_depth_publisher, _fe_to_imu_publisher;
        bool _intialize_time_base;
        double _camera_time_base;
        std::map<stream_index_pair, std::vector<rs2::stream_profile>> _enabled_profiles;

        ros::Publisher _pointcloud_publisher;
        ros::Time _ros_time_base;
    };//end class

    PLUGINLIB_EXPORT_CLASS(realsense_ros_camera::DS5NodeletCamera, nodelet::Nodelet)

}//end namespace
