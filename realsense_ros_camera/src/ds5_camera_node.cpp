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
#include <librealsense/rs2.hpp>
#include <librealsense/rsutil2.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/version.hpp>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <realsense_ros_camera/constants.h>
#include <realsense_ros_camera/Extrinsics.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <sensor_msgs/Imu.h>
#include <realsense_ros_camera/IMUInfo.h>


struct to_string
{
    std::ostringstream ss;
    template<class T> to_string & operator << (const T & val) { ss << val; return *this; }
    operator std::string() const { return ss.str(); }
};

namespace realsense_ros_camera
{
class DS5NodeletCamera: public nodelet::Nodelet
{
public:
  DS5NodeletCamera() :
    _cameraStarted(false),
    _base_frame_id(""),
    _intialize_time_base(false)
  {
      //rs2::log_to_console(rs2_log_severity::RS2_LOG_SEVERITY_INFO); // Enable LRS logger

      // Types for depth stream
      _format[rs2_stream::RS2_STREAM_DEPTH] = RS2_FORMAT_Z16;   // libRS type
      _image_format[rs2_stream::RS2_STREAM_DEPTH] = CV_16UC1;    // CVBridge type
      _encoding[rs2_stream::RS2_STREAM_DEPTH] = sensor_msgs::image_encodings::TYPE_16UC1; // ROS message type
      _unit_step_size[rs2_stream::RS2_STREAM_DEPTH] = sizeof(uint16_t); // sensor_msgs::ImagePtr row step size
      _stream_name[rs2_stream::RS2_STREAM_DEPTH] = "depth";

      // Infrared stream - Left
      _format[rs2_stream::RS2_STREAM_INFRARED] = RS2_FORMAT_Y8;   // libRS type
      _image_format[rs2_stream::RS2_STREAM_INFRARED] = CV_8UC1;    // CVBridge type
      _encoding[rs2_stream::RS2_STREAM_INFRARED] = sensor_msgs::image_encodings::TYPE_8UC1; // ROS message type
      _unit_step_size[rs2_stream::RS2_STREAM_INFRARED] = sizeof(uint8_t); // sensor_msgs::ImagePtr row step size
      _stream_name[rs2_stream::RS2_STREAM_INFRARED] = "infra1";

      // Infrared stream - Right
      _format[rs2_stream::RS2_STREAM_INFRARED2] = RS2_FORMAT_Y8;   // libRS type
      _image_format[rs2_stream::RS2_STREAM_INFRARED2] = CV_8UC1;    // CVBridge type
      _encoding[rs2_stream::RS2_STREAM_INFRARED2] = sensor_msgs::image_encodings::TYPE_8UC1; // ROS message type
      _unit_step_size[rs2_stream::RS2_STREAM_INFRARED2] = sizeof(uint8_t); // sensor_msgs::ImagePtr row step size
      _stream_name[rs2_stream::RS2_STREAM_INFRARED2] = "infra2";

      // Types for color stream
      _format[rs2_stream::RS2_STREAM_COLOR] = RS2_FORMAT_RGB8;   // libRS type
      _image_format[rs2_stream::RS2_STREAM_COLOR] = CV_8UC3;    // CVBridge type
      _encoding[rs2_stream::RS2_STREAM_COLOR] = sensor_msgs::image_encodings::TYPE_8UC3; // ROS message type
      _unit_step_size[rs2_stream::RS2_STREAM_COLOR] = 3; // sensor_msgs::ImagePtr row step size
      _stream_name[rs2_stream::RS2_STREAM_COLOR] = "color";

      // Types for fisheye stream
      _format[rs2_stream::RS2_STREAM_FISHEYE] = RS2_FORMAT_RAW8;   // libRS type
      _image_format[rs2_stream::RS2_STREAM_FISHEYE] = CV_8UC1;    // CVBridge type
      _encoding[rs2_stream::RS2_STREAM_FISHEYE] = sensor_msgs::image_encodings::TYPE_8UC1; // ROS message type
      _unit_step_size[rs2_stream::RS2_STREAM_FISHEYE] = sizeof(uint8_t); // sensor_msgs::ImagePtr row step size
      _stream_name[rs2_stream::RS2_STREAM_FISHEYE] = "fisheye";

      // Types for Motion-Module streams
      _format[rs2_stream::RS2_STREAM_GYRO] = RS2_FORMAT_MOTION_XYZ32F;   // libRS type
      _image_format[rs2_stream::RS2_STREAM_GYRO] = CV_8UC1;    // CVBridge type
      _encoding[rs2_stream::RS2_STREAM_GYRO] = sensor_msgs::image_encodings::TYPE_8UC1; // ROS message type
      _unit_step_size[rs2_stream::RS2_STREAM_GYRO] = sizeof(uint8_t); // sensor_msgs::ImagePtr row step size
      _stream_name[rs2_stream::RS2_STREAM_GYRO] = "gyro";

      _format[rs2_stream::RS2_STREAM_ACCEL] = RS2_FORMAT_MOTION_XYZ32F;   // libRS type
      _image_format[rs2_stream::RS2_STREAM_ACCEL] = CV_8UC1;    // CVBridge type
      _encoding[rs2_stream::RS2_STREAM_ACCEL] = sensor_msgs::image_encodings::TYPE_8UC1; // ROS message type
      _unit_step_size[rs2_stream::RS2_STREAM_ACCEL] = sizeof(uint8_t); // sensor_msgs::ImagePtr row step size
      _stream_name[rs2_stream::RS2_STREAM_ACCEL] = "accel";
  }

  virtual ~DS5NodeletCamera()
  {
      try{
          if (_cameraStarted)
          {
              bool is_imu_stopped = false;
              bool is_depth_or_ir_stopped = false;
              for (auto& kvp: _sensors)
              {
                  if (false == _enable[kvp.first])
                      continue;

                  if (is_imu_stopped &&
                      (rs2_stream::RS2_STREAM_ACCEL == kvp.first) ||
                      (rs2_stream::RS2_STREAM_GYRO == kvp.first))
                  {
                      continue;
                  }
                  else if ((rs2_stream::RS2_STREAM_ACCEL == kvp.first) ||
                           (rs2_stream::RS2_STREAM_GYRO == kvp.first))
                  {
                      is_imu_stopped = true;
                  }

                  if (is_imu_stopped &&
                      (rs2_stream::RS2_STREAM_DEPTH == kvp.first) ||
                      (rs2_stream::RS2_STREAM_INFRARED == kvp.first) ||
                      (rs2_stream::RS2_STREAM_INFRARED2 == kvp.first))
                  {
                      continue;
                  }
                  else if ((rs2_stream::RS2_STREAM_DEPTH == kvp.first) ||
                           (rs2_stream::RS2_STREAM_INFRARED == kvp.first) ||
                           (rs2_stream::RS2_STREAM_INFRARED2 == kvp.first))
                  {
                      is_depth_or_ir_stopped = true;
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
      catch(...) {}
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
  }//end onInit


  void getParameters()
  {
    ROS_INFO("getParameters...");

    _pnh = getPrivateNodeHandle();

    _pnh.param("serial_no", _serial_no, DEFAULT_SERIAL_NO);

    _pnh.param("depth_width", _width[rs2_stream::RS2_STREAM_DEPTH], DEPTH_WIDTH);
    _pnh.param("depth_height", _height[rs2_stream::RS2_STREAM_DEPTH], DEPTH_HEIGHT);
    _pnh.param("depth_fps", _fps[rs2_stream::RS2_STREAM_DEPTH], DEPTH_FPS);
    _pnh.param("enable_depth", _enable[rs2_stream::RS2_STREAM_DEPTH], ENABLE_DEPTH);

    _pnh.param("infra1_width", _width[rs2_stream::RS2_STREAM_INFRARED], INFRA1_WIDTH);
    _pnh.param("infra1_height", _height[rs2_stream::RS2_STREAM_INFRARED], INFRA1_HEIGHT);
    _pnh.param("infra1_fps", _fps[rs2_stream::RS2_STREAM_INFRARED], INFRA1_FPS);
    _pnh.param("enable_infra1", _enable[rs2_stream::RS2_STREAM_INFRARED], ENABLE_INFRA1);

    _pnh.param("infra2_width", _width[rs2_stream::RS2_STREAM_INFRARED2], INFRA2_WIDTH);
    _pnh.param("infra2_height", _height[rs2_stream::RS2_STREAM_INFRARED2], INFRA2_HEIGHT);
    _pnh.param("infra2_fps", _fps[rs2_stream::RS2_STREAM_INFRARED2], INFRA2_FPS);
    _pnh.param("enable_infra2", _enable[rs2_stream::RS2_STREAM_INFRARED2], ENABLE_INFRA2);

    _pnh.param("color_width", _width[rs2_stream::RS2_STREAM_COLOR], COLOR_WIDTH);
    _pnh.param("color_height", _height[rs2_stream::RS2_STREAM_COLOR], COLOR_HEIGHT);
    _pnh.param("color_fps", _fps[rs2_stream::RS2_STREAM_COLOR], COLOR_FPS);
    _pnh.param("enable_color", _enable[rs2_stream::RS2_STREAM_COLOR], ENABLE_COLOR);

    _pnh.param("fisheye_width", _width[rs2_stream::RS2_STREAM_FISHEYE], FISHEYE_WIDTH);
    _pnh.param("fisheye_height", _height[rs2_stream::RS2_STREAM_FISHEYE], FISHEYE_HEIGHT);
    _pnh.param("fisheye_fps", _fps[rs2_stream::RS2_STREAM_FISHEYE], FISHEYE_FPS);
    _pnh.param("enable_fisheye", _enable[rs2_stream::RS2_STREAM_FISHEYE], ENABLE_FISHEYE);

    _pnh.param("gyro_fps", _fps[rs2_stream::RS2_STREAM_GYRO], GYRO_FPS);
    _pnh.param("accel_fps", _fps[rs2_stream::RS2_STREAM_ACCEL], ACCEL_FPS);
    _pnh.param("enable_imu", _enable[rs2_stream::RS2_STREAM_GYRO], ENABLE_IMU);
    _pnh.param("enable_imu", _enable[rs2_stream::RS2_STREAM_ACCEL], ENABLE_IMU);

    _pnh.param("base_frame_id", _base_frame_id, DEFAULT_BASE_FRAME_ID);
    _pnh.param("depth_frame_id", _frame_id[rs2_stream::RS2_STREAM_DEPTH], DEFAULT_DEPTH_FRAME_ID);
    _pnh.param("infra1_frame_id", _frame_id[rs2_stream::RS2_STREAM_INFRARED], DEFAULT_INFRA1_FRAME_ID);
    _pnh.param("infra2_frame_id", _frame_id[rs2_stream::RS2_STREAM_INFRARED2], DEFAULT_INFRA2_FRAME_ID);
    _pnh.param("color_frame_id", _frame_id[rs2_stream::RS2_STREAM_COLOR], DEFAULT_COLOR_FRAME_ID);
    _pnh.param("fisheye_frame_id", _frame_id[rs2_stream::RS2_STREAM_FISHEYE], DEFAULT_FISHEYE_FRAME_ID);
    _pnh.param("imu_gyro_frame_id", _frame_id[rs2_stream::RS2_STREAM_GYRO], DEFAULT_IMU_FRAME_ID);
    _pnh.param("imu_accel_frame_id", _frame_id[rs2_stream::RS2_STREAM_ACCEL], DEFAULT_IMU_FRAME_ID);

    _pnh.param("depth_optical_frame_id", _optical_frame_id[rs2_stream::RS2_STREAM_DEPTH], DEFAULT_DEPTH_OPTICAL_FRAME_ID);
    _pnh.param("infra1_optical_frame_id", _optical_frame_id[rs2_stream::RS2_STREAM_INFRARED], DEFAULT_INFRA1_OPTICAL_FRAME_ID);
    _pnh.param("infra2_optical_frame_id", _optical_frame_id[rs2_stream::RS2_STREAM_INFRARED2], DEFAULT_INFRA2_OPTICAL_FRAME_ID);
    _pnh.param("color_optical_frame_id", _optical_frame_id[rs2_stream::RS2_STREAM_COLOR], DEFAULT_COLOR_OPTICAL_FRAME_ID);
    _pnh.param("fisheye_optical_frame_id", _optical_frame_id[rs2_stream::RS2_STREAM_FISHEYE], DEFAULT_FISHEYE_OPTICAL_FRAME_ID);
    _pnh.param("gyro_optical_frame_id", _optical_frame_id[rs2_stream::RS2_STREAM_GYRO], DEFAULT_GYRO_OPTICAL_FRAME_ID);
    _pnh.param("accel_optical_frame_id", _optical_frame_id[rs2_stream::RS2_STREAM_ACCEL], DEFAULT_ACCEL_OPTICAL_FRAME_ID);
  }//end getParameters


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
                _sensors[rs2_stream::RS2_STREAM_DEPTH] = std::unique_ptr<rs2::sensor>(new rs2::sensor(elem));
                _sensors[rs2_stream::RS2_STREAM_INFRARED] = std::unique_ptr<rs2::sensor>(new rs2::sensor(elem));
                _sensors[rs2_stream::RS2_STREAM_INFRARED2] = std::unique_ptr<rs2::sensor>(new rs2::sensor(elem));
            }
            else if ("RGB Camera" == module_name)
            {
                _sensors[rs2_stream::RS2_STREAM_COLOR] = std::unique_ptr<rs2::sensor>(new rs2::sensor(elem));
            }
            else if ("Wide FOV Camera" == module_name)
            {
                _sensors[rs2_stream::RS2_STREAM_FISHEYE] = std::unique_ptr<rs2::sensor>(new rs2::sensor(elem));
            }
            else if ("Motion Module" == module_name)
            {
                auto hid_sensor = new rs2::sensor(elem);
                _sensors[rs2_stream::RS2_STREAM_GYRO] = std::unique_ptr<rs2::sensor>(hid_sensor);
                _sensors[rs2_stream::RS2_STREAM_ACCEL] = std::unique_ptr<rs2::sensor>(hid_sensor);
            }
            else
            {
                ROS_ERROR_STREAM("Module Name \"" << module_name << "\" isn't supported by LibRealSense!");
                ros::shutdown();
            }
            ROS_INFO_STREAM(std::string(elem.get_info(RS2_CAMERA_INFO_NAME)) << " was found.");
        }

        // Update "enable" map
        std::vector<std::vector<rs2_stream>> streams(IMAGE_STREAMS);
        streams.insert(streams.end(), HID_STREAMS.begin(), HID_STREAMS.end());
        for (auto elem : streams)
        {
            for (auto& stream : elem)
            {
                if (true == _enable[stream] && _sensors.find(stream) == _sensors.end()) // check if device supports the enabled stream
                {
                    ROS_INFO_STREAM(rs2_stream_to_string(stream) << " stream isn't supported by current device! -- Skipping...");
                    _enable[stream] = false;
                }
            }
        }
    }
    catch(const std::exception& ex)
    {
        ROS_ERROR_STREAM("An exception has been thrown: " << ex.what());
        throw;
    }
  }//end setupDevice


  void setupPublishers()
  {
    ROS_INFO("setupPublishers...");
    image_transport::ImageTransport image_transport(getNodeHandle());

    if (true == _enable[rs2_stream::RS2_STREAM_DEPTH])
    {
      _image_publishers[rs2_stream::RS2_STREAM_DEPTH] = image_transport.advertise("camera/depth/image_raw", 1);
      _info_publisher[rs2_stream::RS2_STREAM_DEPTH] = _node_handle.advertise<sensor_msgs::CameraInfo>("camera/depth/camera_info", 1);
      _pointcloud_publisher = _node_handle.advertise<sensor_msgs::PointCloud2>("/camera/points", 1);
    }

    if (true == _enable[rs2_stream::RS2_STREAM_INFRARED])
    {
      _image_publishers[rs2_stream::RS2_STREAM_INFRARED] = image_transport.advertise("camera/infra1/image_raw", 1);
      _info_publisher[rs2_stream::RS2_STREAM_INFRARED] = _node_handle.advertise<sensor_msgs::CameraInfo>("camera/infra1/camera_info", 1);
      _pointcloud_publisher = _node_handle.advertise<sensor_msgs::PointCloud2>("/camera/points", 1);
    }

    if (true == _enable[rs2_stream::RS2_STREAM_INFRARED2])
    {
      _image_publishers[rs2_stream::RS2_STREAM_INFRARED2] = image_transport.advertise("camera/infra2/image_raw", 1);
      _info_publisher[rs2_stream::RS2_STREAM_INFRARED2] = _node_handle.advertise<sensor_msgs::CameraInfo>("camera/infra2/camera_info", 1);
      _pointcloud_publisher = _node_handle.advertise<sensor_msgs::PointCloud2>("/camera/points", 1);
    }

    if (true == _enable[rs2_stream::RS2_STREAM_COLOR])
    {
      _image_publishers[rs2_stream::RS2_STREAM_COLOR] = image_transport.advertise("camera/color/image_raw", 1);
      _info_publisher[rs2_stream::RS2_STREAM_COLOR] = _node_handle.advertise<sensor_msgs::CameraInfo>("camera/color/camera_info", 1);
    }

    if (true == _enable[rs2_stream::RS2_STREAM_FISHEYE] &&
        true == _enable[rs2_stream::RS2_STREAM_DEPTH])
    {
        _image_publishers[rs2_stream::RS2_STREAM_FISHEYE] = image_transport.advertise("camera/fisheye/image_raw", 1);
        _info_publisher[rs2_stream::RS2_STREAM_FISHEYE] = _node_handle.advertise<sensor_msgs::CameraInfo>("camera/fisheye/camera_info", 1);
        _fe_to_depth_publisher = _node_handle.advertise<Extrinsics>("camera/extrinsics/fisheye2depth", 1, true);
    }

    if (true == _enable[rs2_stream::RS2_STREAM_GYRO])
    {
        _imu_publishers[rs2_stream::RS2_STREAM_GYRO] = _node_handle.advertise<sensor_msgs::Imu>("camera/gyro/sample", 100);
        _info_publisher[rs2_stream::RS2_STREAM_GYRO] = _node_handle.advertise<IMUInfo>("camera/gyro/imu_info", 1, true);
    }

    if (true == _enable[rs2_stream::RS2_STREAM_ACCEL])
    {
        _imu_publishers[rs2_stream::RS2_STREAM_ACCEL] = _node_handle.advertise<sensor_msgs::Imu>("camera/accel/sample", 100);
        _info_publisher[rs2_stream::RS2_STREAM_ACCEL] = _node_handle.advertise<IMUInfo>("camera/accel/imu_info", 1, true);
    }

    if (true == _enable[rs2_stream::RS2_STREAM_FISHEYE] &&
        (true == _enable[rs2_stream::RS2_STREAM_GYRO] ||
        true == _enable[rs2_stream::RS2_STREAM_ACCEL]))
    {
        _fe_to_imu_publisher = _node_handle.advertise<Extrinsics>("camera/extrinsics/fisheye2imu", 1, true);
    }
  }//end setupPublishers


  void setupStreams()
  {
      ROS_INFO("setupStreams...");
      try{
          std::vector<std::vector<rs2::stream_profile>> enabled_profiles;
          for (const auto streams : IMAGE_STREAMS)
          {
              std::vector<rs2::stream_profile> enabled_profile;
              for (auto& elem : streams)
              {
                  if (true == _enable[elem])
                  {
                      ROS_INFO_STREAM(_stream_name[elem] << " stream is enabled - width: " << _width[elem] << ", height: " << _height[elem] << ", fps: " << _fps[elem]);
                      auto& sens = _sensors[elem];
                      enabled_profile.push_back({elem, _width[elem], _height[elem], _fps[elem], _format[elem]});
                      // Publish info about the stream
                      getStreamCalibData(elem, sens.get());

                      // Setup stream callback for stream
                      _image[elem] = cv::Mat(_width[elem], _height[elem], _image_format[elem], cv::Scalar(0, 0, 0));
                  }
              }
              if (!enabled_profile.empty())
              {
                  enabled_profiles.push_back(enabled_profile);
              }
          }

          for (auto& profiles : enabled_profiles)
          {
            auto& sens = _sensors[profiles.front().stream];
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

                auto stream = frame.get_stream_type();
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

                if((rs2_stream::RS2_STREAM_DEPTH == stream) && (0 != _pointcloud_publisher.getNumSubscribers()))
                  publishPCTopic(t);

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
              }
            });
          }//end for


          enabled_profiles.clear();
          // Streaming HID
          for (const auto streams : HID_STREAMS)
          {
              std::vector<rs2::stream_profile> motion_module_profiles;
              for (auto& elem : streams)
              {
                  if (true == _enable[elem])
                      motion_module_profiles.push_back({ elem, 1, 1, _fps[elem], _format[elem] });
              }
              if (!motion_module_profiles.empty())
              {
                  enabled_profiles.push_back(motion_module_profiles);
              }
          }

          for (auto& profiles : enabled_profiles)
          {
              auto& sens = _sensors[RS2_STREAM_GYRO];
              sens->open(profiles);

              sens->start([this](rs2::frame frame){
                  auto stream = frame.get_stream_type();
                  if (false == _intialize_time_base)
                      return;

                  if (0 != _info_publisher[stream].getNumSubscribers() ||
                      0 != _imu_publishers[stream].getNumSubscribers())
                  {
                      double elapsed_camera_ms = (/*ms*/ frame.get_timestamp() - /*ms*/ _camera_time_base) / /*ms to seconds*/ 1000;
                      ros::Time t(_ros_time_base.toSec() + elapsed_camera_ms);

                      auto imu_msg = sensor_msgs::Imu();
                      imu_msg.header.frame_id = _optical_frame_id[stream];
                      imu_msg.orientation.x = 0.0;
                      imu_msg.orientation.y = 0.0;
                      imu_msg.orientation.z = 0.0;
                      imu_msg.orientation.w = 0.0;
                      imu_msg.orientation_covariance = { -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

                      auto axes = *(reinterpret_cast<const float3*>(frame.get_data()));
                      if (rs2_stream::RS2_STREAM_GYRO == stream)
                      {
                        imu_msg.angular_velocity.x = axes.x;
                        imu_msg.angular_velocity.y = axes.y;
                        imu_msg.angular_velocity.z = axes.z;
                      }
                      else if (rs2_stream::RS2_STREAM_ACCEL == stream)
                      {
                        imu_msg.linear_acceleration.x = axes.x;
                        imu_msg.linear_acceleration.y = axes.y;
                        imu_msg.linear_acceleration.z = axes.z;
                      }
                      _seq[stream] += 1;
                      imu_msg.header.seq = _seq[stream];
                      imu_msg.header.stamp = t;
                      _imu_publishers[stream].publish(imu_msg);
                  }
              });

              if (true == _enable[rs2_stream::RS2_STREAM_GYRO])
              {
                  ROS_INFO_STREAM(_stream_name[RS2_STREAM_GYRO] << " stream is enabled - " << "fps: " << _fps[RS2_STREAM_GYRO]);
                  IMUInfo gyroInfo{};
                  getImuInfo(RS2_STREAM_GYRO, gyroInfo);
                  _info_publisher[RS2_STREAM_GYRO].publish(gyroInfo);
              }

              if (true == _enable[rs2_stream::RS2_STREAM_ACCEL])
              {
                  ROS_INFO_STREAM(_stream_name[RS2_STREAM_ACCEL] << " stream is enabled - " << "fps: " << _fps[RS2_STREAM_ACCEL]);
                  IMUInfo accelInfo{};
                  getImuInfo(RS2_STREAM_ACCEL, accelInfo);
                  _info_publisher[RS2_STREAM_ACCEL].publish(accelInfo);
              }
          }


          if (true == _enable[rs2_stream::RS2_STREAM_DEPTH] &&
              true == _enable[rs2_stream::RS2_STREAM_FISHEYE])
          {
              auto ex = getFisheye2DepthExtrinsicsMsg();
              _fe_to_depth_publisher.publish(ex);
          }

          if (true == _enable[rs2_stream::RS2_STREAM_FISHEYE] &&
              (_enable[rs2_stream::RS2_STREAM_GYRO] || _enable[rs2_stream::RS2_STREAM_ACCEL]))
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

  }//end setupStreams


  void getStreamCalibData(rs2_stream stream, rs2::sensor* sens)
  {
    rs2_intrinsics intrinsic = sens->get_intrinsics({stream, _width[stream], _height[stream], _fps[stream], _format[stream]});

    _camera_info[stream].width = intrinsic.width;
    _camera_info[stream].height = intrinsic.height;
    _camera_info[stream].header.frame_id = _optical_frame_id[stream];

    _camera_info[stream].K.at(0) = intrinsic.fx;
    _camera_info[stream].K.at(2) = intrinsic.ppx;
    _camera_info[stream].K.at(4) = intrinsic.fy;
    _camera_info[stream].K.at(5) = intrinsic.ppy;
    _camera_info[stream].K.at(8) = 1;

    _camera_info[stream].P.at(0) = _camera_info[stream].K.at(0);
    _camera_info[stream].P.at(1) = 0;
    _camera_info[stream].P.at(2) = _camera_info[stream].K.at(2);
    _camera_info[stream].P.at(3) = 0;
    _camera_info[stream].P.at(4) = 0;
    _camera_info[stream].P.at(5) = _camera_info[stream].K.at(4);
    _camera_info[stream].P.at(6) = _camera_info[stream].K.at(5);
    _camera_info[stream].P.at(7) = 0;
    _camera_info[stream].P.at(8) = 0;
    _camera_info[stream].P.at(9) = 0;
    _camera_info[stream].P.at(10) = 1;
    _camera_info[stream].P.at(11) = 0;

    _camera_info[stream].distortion_model = "plumb_bob";

    // set R (rotation matrix) values to identity matrix
    if (stream != rs2_stream::RS2_STREAM_DEPTH)
    {
      _camera_info[stream].R.at(0) = 1.0;
      _camera_info[stream].R.at(1) = 0.0;
      _camera_info[stream].R.at(2) = 0.0;
      _camera_info[stream].R.at(3) = 0.0;
      _camera_info[stream].R.at(4) = 1.0;
      _camera_info[stream].R.at(5) = 0.0;
      _camera_info[stream].R.at(6) = 0.0;
      _camera_info[stream].R.at(7) = 0.0;
      _camera_info[stream].R.at(8) = 1.0;
    }

    for (int i = 0; i < 5; i++)
    {
      _camera_info[stream].D.push_back(intrinsic.coeffs[i]);
    }
  }//end getStreamCalibData


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

    if (true == _enable[rs2_stream::RS2_STREAM_COLOR])
    {
        // Transform base frame to depth frame
        b2c_msg.header.stamp = transform_ts_;
        b2c_msg.header.frame_id = _base_frame_id;
        b2c_msg.child_frame_id = _frame_id[rs2_stream::RS2_STREAM_COLOR];
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
        c2co_msg.header.frame_id = _frame_id[rs2_stream::RS2_STREAM_COLOR];
        c2co_msg.child_frame_id = _optical_frame_id[rs2_stream::RS2_STREAM_COLOR];
        c2co_msg.transform.translation.x = 0;
        c2co_msg.transform.translation.y = 0;
        c2co_msg.transform.translation.z = 0;
        c2co_msg.transform.rotation.x = q_c2co.getX();
        c2co_msg.transform.rotation.y = q_c2co.getY();
        c2co_msg.transform.rotation.z = q_c2co.getZ();
        c2co_msg.transform.rotation.w = q_c2co.getW();
        _static_tf_broadcaster.sendTransform(c2co_msg);
    }

    if (true == _enable[rs2_stream::RS2_STREAM_DEPTH])
    {
        // Transform base frame to depth frame
        b2d_msg.header.stamp = transform_ts_;
        b2d_msg.header.frame_id = _base_frame_id;
        b2d_msg.child_frame_id = _frame_id[rs2_stream::RS2_STREAM_DEPTH];
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
        d2do_msg.header.frame_id = _frame_id[rs2_stream::RS2_STREAM_DEPTH];
        d2do_msg.child_frame_id = _optical_frame_id[rs2_stream::RS2_STREAM_DEPTH];
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

  void publishPCTopic(const ros::Time& t)
  {
    auto depth_sensor = _sensors[rs2_stream::RS2_STREAM_DEPTH]->as<rs2::depth_sensor>();
    auto depth_intrinsic = depth_sensor.get_intrinsics({rs2_stream::RS2_STREAM_DEPTH, _width[rs2_stream::RS2_STREAM_DEPTH], _height[rs2_stream::RS2_STREAM_DEPTH], _fps[rs2_stream::RS2_STREAM_DEPTH], _format[rs2_stream::RS2_STREAM_DEPTH]});
    auto depth_scale_meters = depth_sensor.get_depth_scale();

    sensor_msgs::PointCloud2 msg_pointcloud;
    msg_pointcloud.header.stamp = t;
    msg_pointcloud.header.frame_id = _optical_frame_id[rs2_stream::RS2_STREAM_DEPTH];
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
        memcpy(&depth_value, &_image[rs2_stream::RS2_STREAM_DEPTH].data[depth_offset], sizeof(uint16_t));
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

  Extrinsics rsExtrinsicsToMsg(rs2_extrinsics extrinsics)
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
      auto& fisheye_sensor = _sensors[rs2_stream::RS2_STREAM_FISHEYE];
      auto& hid_sensor = _sensors[rs2_stream::RS2_STREAM_GYRO];
      Extrinsics extrinsicsMsg = rsExtrinsicsToMsg(fisheye_sensor->get_extrinsics_to(rs2_stream::RS2_STREAM_FISHEYE, *hid_sensor, rs2_stream::RS2_STREAM_GYRO));
      extrinsicsMsg.header.frame_id = "fisheye2imu_extrinsics";
      return extrinsicsMsg;
  }

  Extrinsics getFisheye2DepthExtrinsicsMsg()
  {
      auto& fisheye_sensor = _sensors[rs2_stream::RS2_STREAM_FISHEYE];
      auto& depth_sensor = _sensors[rs2_stream::RS2_STREAM_DEPTH];
      Extrinsics extrinsicsMsg = rsExtrinsicsToMsg(fisheye_sensor->get_extrinsics_to(rs2_stream::RS2_STREAM_FISHEYE, *depth_sensor, rs2_stream::RS2_STREAM_DEPTH));
      extrinsicsMsg.header.frame_id = "fisheye2depth_extrinsics";
      return extrinsicsMsg;
  }

private:
  struct float3
  {
      float x, y, z;
  };

  void getImuInfo(rs2_stream stream, IMUInfo &info)
  {
      auto imuIntrinsics = _sensors[RS2_STREAM_GYRO]->get_motion_intrinsics(stream);
      if (rs2_stream::RS2_STREAM_GYRO == stream)
      {
          info.header.frame_id = "imu_gyro";
      }
      else if (rs2_stream::RS2_STREAM_ACCEL == stream)
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
  }

  const std::vector<std::vector<rs2_stream>> IMAGE_STREAMS =
  {{rs2_stream::RS2_STREAM_DEPTH, rs2_stream::RS2_STREAM_INFRARED, rs2_stream::RS2_STREAM_INFRARED2},
   {rs2_stream::RS2_STREAM_COLOR},
   {rs2_stream::RS2_STREAM_FISHEYE}};

  const std::vector<std::vector<rs2_stream>> HID_STREAMS = {{rs2_stream::RS2_STREAM_GYRO,
                                                             rs2_stream::RS2_STREAM_ACCEL}};

  ros::NodeHandle _node_handle, _pnh;
  bool _cameraStarted;
  std::unique_ptr<rs2::context> _ctx;
  rs2::device _dev;
  std::map<rs2_stream, std::unique_ptr<rs2::sensor>> _sensors;

  std::string _serial_no;
  std::string _dev_location;
  std::string _usb_port_id;
  std::string _camera_type;

  std::map<rs2_stream, int> _width;
  std::map<rs2_stream, int> _height;
  std::map<rs2_stream, int> _fps;
  std::map<rs2_stream, bool> _enable;
  std::map<rs2_stream, std::string> _stream_name;
  tf2_ros::StaticTransformBroadcaster _static_tf_broadcaster;

  // DS5 types
  std::map<rs2_stream, image_transport::Publisher> _image_publishers;
  std::map<rs2_stream, ros::Publisher> _imu_publishers;
  std::map<rs2_stream, int> _image_format;
  std::map<rs2_stream, rs2_format> _format;
  std::map<rs2_stream, ros::Publisher> _info_publisher;
  std::map<rs2_stream, cv::Mat> _image;
  std::map<rs2_stream, std::string> _encoding;
  bool _intialize_time_base;
  double _camera_time_base;
  std::string _base_frame_id;
  std::map<rs2_stream, std::string> _frame_id;
  std::map<rs2_stream, std::string> _optical_frame_id;
  std::map<rs2_stream, int> _seq;
  std::map<rs2_stream, int> _unit_step_size;
  std::map<rs2_stream, sensor_msgs::CameraInfo> _camera_info;
  ros::Publisher _fe_to_depth_publisher, _fe_to_imu_publisher;

  ros::Publisher _pointcloud_publisher;
  ros::Time _ros_time_base;
};//end class

PLUGINLIB_DECLARE_CLASS(realsense_ros_camera, NodeletDS5Camera, realsense_ros_camera::DS5NodeletCamera, nodelet::Nodelet);

}//end namespace
