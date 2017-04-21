// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved

#include <omp.h>
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
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <realsense_ros_camera/Extrinsics.h>
#include <realsense_ros_camera/IMUInfo.h>
#include <realsense_ros_camera/constants.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>


namespace realsense_ros_camera
{
class NodeletCamera: public nodelet::Nodelet
{
public:
  NodeletCamera() :
    cameraStarted_(false)
  {
    // Types for depth stream
    format_[rs::stream::depth] = rs::format::z16;   // libRS type
    image_format_[rs::stream::depth] = CV_16UC1;    // CVBridge type
    encoding_[rs::stream::depth] = sensor_msgs::image_encodings::TYPE_16UC1; // ROS message type
    unit_step_size_[rs::stream::depth] = sizeof(uint16_t); // sensor_msgs::ImagePtr row step size
    stream_name_[rs::stream::depth] = "depth";

    // Types for color stream
    format_[rs::stream::color] = rs::format::rgb8;  // libRS type
    image_format_[rs::stream::color] = CV_8UC3;     // CVBridge type
    encoding_[rs::stream::color] = sensor_msgs::image_encodings::RGB8; // ROS message type
    unit_step_size_[rs::stream::color] = sizeof(unsigned char) * 3; // sensor_msgs::ImagePtr row step size
    stream_name_[rs::stream::color] = "color";

    // Types for fisheye stream
    format_[rs::stream::fisheye] = rs::format::raw8;  // libRS type
    image_format_[rs::stream::fisheye] = CV_8UC1;     // CVBridge type
    encoding_[rs::stream::fisheye] = sensor_msgs::image_encodings::TYPE_8UC1; // ROS message type
    unit_step_size_[rs::stream::fisheye] = sizeof(unsigned char); // sensor_msgs::ImagePtr row step size
    stream_name_[rs::stream::fisheye] = "fisheye";
  }

  virtual ~NodeletCamera()
  {
    if (cameraStarted_)
    {
      if (isZR300_)
        device_->stop(rs::source::all_sources);
      else
        device_->stop();
    }

    usleep(250);
    ctx_.reset();
  }

private:
  virtual void onInit()
  {
    getParameters();

    if (false == setupDevice())
      return;

    setupPublishers();
    setupStreams();
    publishStaticTransforms();
  }//end onInit


  void getParameters()
  {
    pnh_ = getPrivateNodeHandle();

    pnh_.param("serial_no", serial_no_, DEFAULT_SERIAL_NO);

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

    pnh_.param("base_frame_id", base_frame_id_, DEFAULT_BASE_FRAME_ID);
    pnh_.param("depth_frame_id", frame_id_[rs::stream::depth], DEFAULT_DEPTH_FRAME_ID);
    pnh_.param("color_frame_id", frame_id_[rs::stream::color], DEFAULT_COLOR_FRAME_ID);
    pnh_.param("fisheye_frame_id", frame_id_[rs::stream::fisheye], DEFAULT_FISHEYE_FRAME_ID);
    pnh_.param("imu_frame_id", imu_frame_id_, DEFAULT_IMU_FRAME_ID);
    pnh_.param("depth_optical_frame_id", optical_frame_id_[rs::stream::depth], DEFAULT_DEPTH_OPTICAL_FRAME_ID);
    pnh_.param("color_optical_frame_id", optical_frame_id_[rs::stream::color], DEFAULT_COLOR_OPTICAL_FRAME_ID);
    pnh_.param("fisheye_optical_frame_id", optical_frame_id_[rs::stream::fisheye], DEFAULT_FISHEYE_OPTICAL_FRAME_ID);
    pnh_.param("imu_optical_frame_id", optical_imu_frame_id_, DEFAULT_IMU_OPTICAL_FRAME_ID);

    optical_imu_id_[0] = "imu_accel_frame_id";
    optical_imu_id_[1] = "imu_gyro_frame_id";
  }//end getParameters


  bool setupDevice()
  {
    ctx_.reset(new rs::context());
    int num_of_cams = ctx_ -> get_device_count();
    if (num_of_cams == 0)
    {
      ROS_ERROR("error : no RealSense R200, LR200, or ZR300 devices found.");
      ctx_.reset();
      return false;
    }

    rs::device *detected_dev;
    for (int i = 0; i < num_of_cams; i++)
    {
      detected_dev = ctx_->get_device(i);
      detected_dev->get_serial();
      if (serial_no_.empty() || (serial_no_ == std::string(detected_dev->get_serial())))
      {
        device_ = detected_dev;
        break;
      }
    }

    if (device_ == nullptr)
    {
      ROS_ERROR_STREAM("error: No RealSense device with serial_no = " << serial_no_ << " found.");
      ctx_.reset();
      return false;
    }

    auto device_name = device_ -> get_name();
    ROS_INFO_STREAM(device_name << ", serial_no: " << std::string(device_ ->get_serial()));
    if (std::string(device_name).find("ZR300") == std::string::npos)
    {
      isZR300_ = false;
      enable_[rs::stream::fisheye] = false;

      if ((std::string(device_name).find("R200") == std::string::npos) && 
          (std::string(device_name).find("LR200") == std::string::npos))
      {
        ROS_ERROR_STREAM("error: This ROS node supports R200, LR200, and ZR300 only.");
        ROS_ERROR_STREAM("       See https://github.com/intel-ros/realsense for F200 and SR300 support.");
        ctx_.reset();
        return false;
      }
    }
    else
    {
      // Only enable ZR300 functionality if fisheye stream is enabled.  
      // Accel/Gyro automatically enabled when fisheye requested
      if (true == enable_[rs::stream::fisheye])
        isZR300_ = true;
    }

    return true;
  }//end setupDevice


  void setupPublishers()
  {
    image_transport::ImageTransport image_transport(getNodeHandle());

    // Stream publishers and latched topics
    if (true == enable_[rs::stream::color])
    {
      image_publishers_[rs::stream::color] = image_transport.advertise("camera/color/image_raw", 1);
      info_publisher_[rs::stream::color] = 
        node_handle.advertise< sensor_msgs::CameraInfo >("camera/color/camera_info", 1);
    }

    if (true == enable_[rs::stream::depth])
    {
      image_publishers_[rs::stream::depth] = image_transport.advertise("camera/depth/image_raw", 1);
      info_publisher_[rs::stream::depth] = 
        node_handle.advertise< sensor_msgs::CameraInfo >("camera/depth/camera_info", 1);
    
      pointcloud_publisher_ = node_handle.advertise<sensor_msgs::PointCloud2>("/camera/points", 1);
    }

    if (isZR300_)
    {
      // Stream publishers
      image_publishers_[rs::stream::fisheye] = image_transport.advertise("camera/fisheye/image_raw", 1);
      info_publisher_[rs::stream::fisheye] = 
        node_handle.advertise< sensor_msgs::CameraInfo >("camera/fisheye/camera_info", 1);

      // HW timestamp version of image publishers, for realsense_ros_slam package
      image_publishers_hw_timestamp_[rs::stream::color] = node_handle.advertise<sensor_msgs::Image>("/camera/color/image_raw_hw_timestamp", 1);
      image_publishers_hw_timestamp_[rs::stream::depth] = node_handle.advertise<sensor_msgs::Image>("/camera/depth/image_raw_hw_timestamp", 1);
      image_publishers_hw_timestamp_[rs::stream::fisheye] = node_handle.advertise<sensor_msgs::Image>("/camera/fisheye/image_raw_hw_timestamp", 1);

      imu_publishers_[RS_EVENT_IMU_GYRO] = node_handle.advertise< sensor_msgs::Imu >("camera/gyro/sample", 100); 
      imu_publishers_[RS_EVENT_IMU_ACCEL] = node_handle.advertise< sensor_msgs::Imu >("camera/accel/sample", 100);

      // Latched topics
      fe2imu_publisher_ = node_handle.advertise< Extrinsics >("camera/extrinsics/fisheye2imu", 1, true);
      fe2depth_publisher_ = node_handle.advertise< Extrinsics >("camera/extrinsics/fisheye2depth", 1, true);
      accelInfo_publisher_ = node_handle.advertise< IMUInfo >("camera/accel/imu_info", 1, true);
      gyroInfo_publisher_ = node_handle.advertise< IMUInfo >("camera/gyro/imu_info", 1, true);
    }
  }//end setupPublishers


  void setupStreams()
  {
    device_->set_option(rs::option::r200_lr_auto_exposure_enabled, 1);

    const rs::stream All[] = { rs::stream::depth, rs::stream::color, rs::stream::fisheye };
    for (const auto stream : All)
    {
      if (false == enable_[stream])
        continue;

      // Define lambda callback for receiving stream data
      stream_callback_per_stream[stream] = [this,stream](rs::frame frame)
      {
        image_[stream].data = (unsigned char *) frame.get_data();
        ros::Time t = ros::Time::now();

        if((stream == rs::stream::depth) && (0 != pointcloud_publisher_.getNumSubscribers()))
          publishPCTopic(t);

        seq_[stream] += 1;                   
        if((0 != image_publishers_[stream].getNumSubscribers()) || 
           (0 != image_publishers_hw_timestamp_[stream].getNumSubscribers()))
        {
          sensor_msgs::ImagePtr img;          
          img = cv_bridge::CvImage(std_msgs::Header(), encoding_[stream], image_[stream]).toImageMsg();
          img->width = image_[stream].cols;
          img->height = image_[stream].rows;
          img->is_bigendian = false;
          img->step = image_[stream].cols * unit_step_size_[stream];
          img->header.frame_id = optical_frame_id_[stream];
          img->header.stamp = t;
          img->header.seq = seq_[stream];

          // ROS Timestamp
          if(0 != image_publishers_[stream].getNumSubscribers())
            image_publishers_[stream].publish(img);

          // Camera HW Timestamp for realsense_ros_slam
          if(0 != image_publishers_hw_timestamp_[stream].getNumSubscribers())
          {
            if (rs::timestamp_domain::microcontroller != frame.get_frame_timestamp_domain())
            {
              ROS_ERROR_STREAM("error: Junk time stamp in stream:" << (int)(stream) <<
                               "\twith frame counter:" << frame.get_frame_number());
            }
            else
            {
              img->header.stamp = ros::Time(frame.get_timestamp());
              image_publishers_hw_timestamp_[stream].publish(img);
            }
          }
        }
        
        if(0 != image_publishers_[stream].getNumSubscribers())
        {
          camera_info_[stream].header.stamp = t;
          camera_info_[stream].header.seq = seq_[stream];
          info_publisher_[stream].publish(camera_info_[stream]);
        }
      };

      // Enable the stream
      device_->enable_stream(stream, width_[stream], height_[stream], format_[stream], fps_[stream]);

      // Publish info about the stream
      getStreamCalibData(stream);

      // Setup stream callback for stream
      image_[stream] = cv::Mat(camera_info_[stream].height, camera_info_[stream].width, 
        image_format_[stream], cv::Scalar(0, 0, 0));
      device_->set_frame_callback(stream, stream_callback_per_stream[stream]);

      ROS_INFO_STREAM("  enabled " << stream_name_[stream] << " stream, width: " 
        << camera_info_[stream].width << " height: " << camera_info_[stream].height << " fps: " << fps_[stream]);
    }//end for


    if (isZR300_)
    {
      // Needed to align image timestamps to common clock-domain with the motion events
      device_->set_option(rs::option::fisheye_strobe, 1); 
      // This option causes the fisheye image to be aquired in-sync with the depth image.
      device_->set_option(rs::option::fisheye_external_trigger, 1); 
      device_->set_option(rs::option::fisheye_color_auto_exposure, 1);
      seq_motion[RS_EVENT_IMU_GYRO] = 0;
      seq_motion[RS_EVENT_IMU_ACCEL] = 0;
      
      //define callback to the motion events and set it.
      std::function<void(rs::motion_data)> motion_callback;
      motion_callback = [this](rs::motion_data entry)
      {
        if ((entry.timestamp_data.source_id != RS_EVENT_IMU_GYRO) && 
            (entry.timestamp_data.source_id != RS_EVENT_IMU_ACCEL))
          return;

        rs_event_source motionType = entry.timestamp_data.source_id;
        
        // If there is nobody subscribed to the stream, do no further
        // processing
        if( 0 == imu_publishers_[motionType].getNumSubscribers())
          return;

        sensor_msgs::Imu imu_msg = sensor_msgs::Imu();
        imu_msg.header.stamp = ros::Time::now();
        imu_msg.header.frame_id = optical_imu_id_[motionType];
        imu_msg.orientation.x = 0.0;
        imu_msg.orientation.y = 0.0;
        imu_msg.orientation.z = 0.0;
        imu_msg.orientation.w = 0.0;
        imu_msg.orientation_covariance = { -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        if (motionType == RS_EVENT_IMU_GYRO)
        {
          imu_msg.angular_velocity.x = entry.axes[0];
          imu_msg.angular_velocity.y = entry.axes[1];
          imu_msg.angular_velocity.z = entry.axes[2];
        }
        if (motionType == RS_EVENT_IMU_ACCEL)
        {
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

      device_->enable_motion_tracking(motion_callback, timestamp_callback);
      ROS_INFO_STREAM("  enabled accel and gyro stream");

      // publish ZR300-specific intrinsics/extrinsics
      IMUInfo accelInfo, gyroInfo;
      getImuInfo(device_, accelInfo, gyroInfo);

      fe2imu_publisher_.publish(getFisheye2ImuExtrinsicsMsg(device_));
      fe2depth_publisher_.publish(getFisheye2DepthExtrinsicsMsg(device_));
      accelInfo_publisher_.publish(accelInfo);
      gyroInfo_publisher_.publish(gyroInfo);
    }

    if (isZR300_)
      device_->start(rs::source::all_sources);
    else
      device_->start();
    cameraStarted_ = true;
  }//end setupStreams


  void getStreamCalibData(rs::stream stream)
  {
    rs::intrinsics intrinsic = device_->get_stream_intrinsics(stream);

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
      rs::extrinsics extrinsic = device_->get_extrinsics(rs::stream::depth, rs::stream::color);
      camera_info_[stream].P.at(3) = extrinsic.translation[0];     // Tx
      camera_info_[stream].P.at(7) = extrinsic.translation[1];     // Ty
      camera_info_[stream].P.at(11) = extrinsic.translation[2];    // Tz

      for (int i = 0; i < 9; i++)
        camera_info_[stream].R.at(i) = extrinsic.rotation[i];
    }

    switch ((int32_t)intrinsic.model())
    {
    case 0:
      camera_info_[stream].distortion_model = "plumb_bob";
      break;
    case 1:
      // This is the same as "modified_brown_conrady", but used by ROS
      camera_info_[stream].distortion_model = "plumb_bob"; 
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


  void publishStaticTransforms()
  {
    // Publish transforms for the cameras
    tf::Quaternion q_c2co;
    tf::Quaternion q_d2do;
    tf::Quaternion q_i2io;
    geometry_msgs::TransformStamped b2d_msg;
    geometry_msgs::TransformStamped d2do_msg;
    geometry_msgs::TransformStamped b2c_msg;
    geometry_msgs::TransformStamped c2co_msg;
    geometry_msgs::TransformStamped b2i_msg;
    geometry_msgs::TransformStamped i2io_msg;

    // Get the current timestamp for all static transforms
    ros::Time transform_ts_ = ros::Time::now();

    // The color frame is used as the base frame.
    // Hence no additional transformation is done from base frame to color frame.
    b2c_msg.header.stamp = transform_ts_;
    b2c_msg.header.frame_id = base_frame_id_;
    b2c_msg.child_frame_id = frame_id_[rs::stream::color];
    b2c_msg.transform.translation.x = 0;
    b2c_msg.transform.translation.y = 0;
    b2c_msg.transform.translation.z = 0;
    b2c_msg.transform.rotation.x = 0;
    b2c_msg.transform.rotation.y = 0;
    b2c_msg.transform.rotation.z = 0;
    b2c_msg.transform.rotation.w = 1;
    static_tf_broadcaster_.sendTransform(b2c_msg);

    // Transform color frame to color optical frame
    q_c2co.setRPY(-M_PI / 2, 0.0, -M_PI / 2);
    c2co_msg.header.stamp = transform_ts_;
    c2co_msg.header.frame_id = frame_id_[rs::stream::color];
    c2co_msg.child_frame_id = optical_frame_id_[rs::stream::color];
    c2co_msg.transform.translation.x = 0;
    c2co_msg.transform.translation.y = 0;
    c2co_msg.transform.translation.z = 0;
    c2co_msg.transform.rotation.x = q_c2co.getX();
    c2co_msg.transform.rotation.y = q_c2co.getY();
    c2co_msg.transform.rotation.z = q_c2co.getZ();
    c2co_msg.transform.rotation.w = q_c2co.getW();
    static_tf_broadcaster_.sendTransform(c2co_msg);

    // Transform base frame to depth frame
    rs::extrinsics color2depth_extrinsic = device_->get_extrinsics(rs::stream::color, rs::stream::depth);
    b2d_msg.header.stamp = transform_ts_;
    b2d_msg.header.frame_id = base_frame_id_;
    b2d_msg.child_frame_id = frame_id_[rs::stream::depth];
    b2d_msg.transform.translation.x =  color2depth_extrinsic.translation[2];
    b2d_msg.transform.translation.y = -color2depth_extrinsic.translation[0];
    b2d_msg.transform.translation.z = -color2depth_extrinsic.translation[1];
    b2d_msg.transform.rotation.x = 0;
    b2d_msg.transform.rotation.y = 0;
    b2d_msg.transform.rotation.z = 0;
    b2d_msg.transform.rotation.w = 1;
    static_tf_broadcaster_.sendTransform(b2d_msg);

    // Transform depth frame to depth optical frame
    q_d2do.setRPY(-M_PI / 2, 0.0, -M_PI / 2);
    d2do_msg.header.stamp = transform_ts_;
    d2do_msg.header.frame_id = frame_id_[rs::stream::depth];
    d2do_msg.child_frame_id = optical_frame_id_[rs::stream::depth];
    d2do_msg.transform.translation.x = 0;
    d2do_msg.transform.translation.y = 0;
    d2do_msg.transform.translation.z = 0;
    d2do_msg.transform.rotation.x = q_d2do.getX();
    d2do_msg.transform.rotation.y = q_d2do.getY();
    d2do_msg.transform.rotation.z = q_d2do.getZ();
    d2do_msg.transform.rotation.w = q_d2do.getW();
    static_tf_broadcaster_.sendTransform(d2do_msg);

    if (isZR300_)
    {
      tf::Quaternion q_f2fo, q_imu2imuo;
      geometry_msgs::TransformStamped b2f_msg;
      geometry_msgs::TransformStamped f2fo_msg;
      geometry_msgs::TransformStamped b2imu_msg;
      geometry_msgs::TransformStamped imu2imuo_msg;

      // Transform base frame to fisheye frame
      b2f_msg.header.stamp = transform_ts_;
      b2f_msg.header.frame_id = base_frame_id_;
      b2f_msg.child_frame_id = frame_id_[rs::stream::fisheye];
      rs::extrinsics color2fisheye_extrinsic = device_->get_extrinsics(rs::stream::color, rs::stream::fisheye);
      b2f_msg.transform.translation.x =  color2fisheye_extrinsic.translation[2];
      b2f_msg.transform.translation.y = -color2fisheye_extrinsic.translation[0];
      b2f_msg.transform.translation.z = -color2fisheye_extrinsic.translation[1];
      b2f_msg.transform.rotation.x = 0;
      b2f_msg.transform.rotation.y = 0;
      b2f_msg.transform.rotation.z = 0;
      b2f_msg.transform.rotation.w = 1;
      static_tf_broadcaster_.sendTransform(b2f_msg);

      // Transform fisheye frame to fisheye optical frame
      q_f2fo.setRPY(-M_PI / 2, 0.0, -M_PI / 2);
      f2fo_msg.header.stamp = transform_ts_;
      f2fo_msg.header.frame_id = frame_id_[rs::stream::fisheye];
      f2fo_msg.child_frame_id = optical_frame_id_[rs::stream::fisheye];
      f2fo_msg.transform.translation.x = 0;
      f2fo_msg.transform.translation.y = 0;
      f2fo_msg.transform.translation.z = 0;
      f2fo_msg.transform.rotation.x = q_f2fo.getX();
      f2fo_msg.transform.rotation.y = q_f2fo.getY();
      f2fo_msg.transform.rotation.z = q_f2fo.getZ();
      f2fo_msg.transform.rotation.w = q_f2fo.getW();
      static_tf_broadcaster_.sendTransform(f2fo_msg);
    }
  }

  void publishPCTopic(ros::Time t)
  {
    rs::intrinsics depth_intrinsic = device_->get_stream_intrinsics(rs::stream::depth);
    float depth_scale_meters = device_->get_depth_scale();
    
    sensor_msgs::PointCloud2 msg_pointcloud;
    msg_pointcloud.header.stamp = t;
    msg_pointcloud.header.frame_id = optical_frame_id_[rs::stream::depth];
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
        memcpy(&depth_value, &image_[rs::stream::depth].data[depth_offset], sizeof(uint16_t));
        scaled_depth = static_cast<float>(depth_value) * depth_scale_meters;
        if (scaled_depth <= 0.0f || scaled_depth > MAX_Z)
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
          rs_deproject_pixel_to_point(depth_point, &depth_intrinsic, depth_pixel, scaled_depth);
        }
        
        // Assign 3d point
        memcpy(&msg_pointcloud.data[cloud_offset + msg_pointcloud.fields[0].offset], &depth_point[0], sizeof(float)); // X
        memcpy(&msg_pointcloud.data[cloud_offset + msg_pointcloud.fields[1].offset], &depth_point[1], sizeof(float)); // Y
        memcpy(&msg_pointcloud.data[cloud_offset + msg_pointcloud.fields[2].offset], &depth_point[2], sizeof(float)); // Z
      } // for
  
    pointcloud_publisher_.publish(msg_pointcloud);
  }


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


private:
  ros::NodeHandle node_handle, pnh_;
  bool cameraStarted_;
  std::unique_ptr< rs::context > ctx_;
  rs::device *device_;

  std::string serial_no_;
  std::string usb_port_id_;
  std::string camera_type_;

  std::map<rs::stream, int> width_;
  std::map<rs::stream, int> height_;
  std::map<rs::stream, int> fps_;
  std::map<rs::stream, bool> enable_;
  std::map<rs::stream, std::string> stream_name_;
  tf2_ros::StaticTransformBroadcaster static_tf_broadcaster_;

  // R200 and ZR300 types
  std::map<rs::stream, image_transport::Publisher> image_publishers_;
  std::map<rs::stream, ros::Publisher> image_publishers_hw_timestamp_;
  std::map<rs::stream, int> image_format_;
  std::map<rs::stream, rs::format> format_;
  std::map<rs::stream, ros::Publisher> info_publisher_;
  std::map<rs::stream, cv::Mat> image_;
  std::map<rs::stream, std::string> encoding_;
  std::string base_frame_id_;
  std::map<rs::stream, std::string> frame_id_;
  std::map<rs::stream, std::string> optical_frame_id_;
  std::string imu_frame_id_, optical_imu_frame_id_;
  std::map<rs::stream, int> seq_;
  std::map<rs::stream, int> unit_step_size_;
  std::map<rs::stream, std::function<void(rs::frame)>> stream_callback_per_stream;
  std::map<rs::stream, sensor_msgs::CameraInfo> camera_info_;
  ros::Publisher pointcloud_publisher_;

  // ZR300 specific types
  bool isZR300_ = false;
  ros::Publisher accelInfo_publisher_, gyroInfo_publisher_, fe2imu_publisher_, fe2depth_publisher_;
  ros::Publisher imu_publishers_[2];
  int seq_motion[2];
  std::string optical_imu_id_[2];
};//end class

PLUGINLIB_DECLARE_CLASS(realsense_ros_camera, NodeletCamera, realsense_ros_camera::NodeletCamera, nodelet::Nodelet);

}//end namespace
