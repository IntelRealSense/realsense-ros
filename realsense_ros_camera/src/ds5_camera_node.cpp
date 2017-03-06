// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2016 Intel Corporation. All Rights Reserved

#include <iostream>
#include <functional>
#include <iomanip>
#include <map>
#include <atomic>
#include <nodelet/nodelet.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <ros/package.h>
// #include <librealsense/rs.hpp>

#include <librealsense/rs2.hpp>
#include <librealsense/rsutil2.hpp>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
//#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/version.hpp>
#include <sensor_msgs/CameraInfo.h>
//#include <realsense_ros_camera/Extrinsics.h>
//#include <realsense_ros_camera/IMUInfo.h>
#include <realsense_ros_camera/constants.h>
//#include <sensor_msgs/Imu.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>



// R400 types
std::map<rs2_stream, image_transport::Publisher> image_publishers_;
std::map<rs2_stream, int> image_format_;
std::map<rs2_stream, rs2_format> format_;
std::map<rs2_stream, ros::Publisher> info_publisher_;
std::map<rs2_stream, cv::Mat> image_;
std::map<rs2_stream, std::string> encoding_;
std::string base_frame_id_;
std::map<rs2_stream, std::string> frame_id_;
std::map<rs2_stream, std::string> optical_frame_id_;
std::map<rs2_stream, int> seq_;
std::map<rs2_stream, int> unit_step_size_;
// std::map<rs::stream, std::function<void(rs::frame)>> stream_callback_per_stream;
std::map<rs2_stream, sensor_msgs::CameraInfo> camera_info_;


namespace realsense_ros_camera
{
class DS5NodeletCamera: public nodelet::Nodelet
{
public:
  DS5NodeletCamera() :
    cameraStarted(false)
  {
    // libRealsense format types for stream
    format_[rs2_stream::RS2_STREAM_DEPTH] = RS2_FORMAT_Z16;

    // CVBridge native image types
    image_format_[rs2_stream::RS2_STREAM_DEPTH] = CV_16UC1;

    // ROS sensor_msgs::ImagePtr encoding types
    encoding_[rs2_stream::RS2_STREAM_DEPTH] = sensor_msgs::image_encodings::TYPE_16UC1;

    // ROS sensor_msgs::ImagePtr row step sizes
    unit_step_size_[rs2_stream::RS2_STREAM_DEPTH] = sizeof(uint16_t);

    stream_name_[rs2_stream::RS2_STREAM_DEPTH] = "depth";
  }

  virtual ~DS5NodeletCamera()
  {
    if (cameraStarted)
    {
      device.stop();
    }
    ctx.reset();

    const rs2_stream All[] = {rs2_stream::RS2_STREAM_DEPTH};
    for (const auto stream : All)
    {
      if (false == enable_[stream])
        continue;
      image_publishers_[stream].shutdown();
    }
    image_publishers_.clear();
  }

private:
  virtual void onInit()
  {
    getParameters();

    if (false == setupDevice())
      return;

    setupPublishers();
    setupStreams();
    // publishStaticTransforms();
  }//end onInit


  void getParameters()
  {
    pnh_ = getPrivateNodeHandle();

    pnh_.param("serial_no", serial_no, DEFAULT_SERIAL_NO);

    pnh_.param("depth_width", width_[rs2_stream::RS2_STREAM_DEPTH], 640);
    pnh_.param("depth_height", height_[rs2_stream::RS2_STREAM_DEPTH], 480);
    pnh_.param("depth_fps", fps_[rs2_stream::RS2_STREAM_DEPTH], 30);
    pnh_.param("enable_depth", enable_[rs2_stream::RS2_STREAM_DEPTH], true);

    pnh_.param("base_frame_id", base_frame_id_, DEFAULT_BASE_FRAME_ID);
    pnh_.param("depth_frame_id", frame_id_[rs2_stream::RS2_STREAM_DEPTH], DEFAULT_DEPTH_FRAME_ID);
    pnh_.param("depth_optical_frame_id", optical_frame_id_[rs2_stream::RS2_STREAM_DEPTH], DEFAULT_DEPTH_OPTICAL_FRAME_ID);
  }//end getParameters


  bool setupDevice()
  {
    ctx.reset(new rs2::context());

    auto list = ctx->query_devices();
    if (list.size() == 0)
    {
      ROS_ERROR("error : no RealSense devices found (R400, R10)");
      ctx.reset();
      return false;
    }

    for( auto dev : list )
    {
      std::string dev_serial_no = dev.get_camera_info(RS2_CAMERA_INFO_DEVICE_SERIAL_NUMBER);
      if (serial_no.empty() || (serial_no == dev_serial_no))
      {
        device = dev;
        break;
      }
    }

    ROS_INFO_STREAM( std::string(device.get_camera_info(RS2_CAMERA_INFO_DEVICE_NAME)) << ", serial_no: " << std::string(device.get_camera_info(RS2_CAMERA_INFO_DEVICE_SERIAL_NUMBER)));
    return true;
  }//end setupDevice


  void setupPublishers()
  {
    image_transport::ImageTransport image_transport(getNodeHandle());

    if (true == enable_[rs2_stream::RS2_STREAM_DEPTH])
    {
      image_publishers_[rs2_stream::RS2_STREAM_DEPTH] = image_transport.advertise("camera/depth/image_raw", 1);
      info_publisher_[rs2_stream::RS2_STREAM_DEPTH] = node_handle.advertise< sensor_msgs::CameraInfo >("camera/depth/camera_info", 1);
    }
  }//end setupPublishers


  void setupStreams()
  {
    // device->set_option(rs::option::r200_lr_auto_exposure_enabled, 1);

    const rs2_stream All[] = {rs2_stream::RS2_STREAM_DEPTH};
    for (const auto stream : All )
    {
      if (false == enable_[stream])
        continue;    


      // Enable the stream
      device.open({ stream, width_[stream], height_[stream], fps_[stream], format_[stream] });

      device.start([stream](rs2::frame frame)
      {
        image_[stream].data = (unsigned char *) frame.get_data();

        sensor_msgs::ImagePtr img;
        img = cv_bridge::CvImage(std_msgs::Header(), encoding_[stream], image_[stream]).toImageMsg();
        img->width = frame.get_width();
        img->height = frame.get_height();
        img->is_bigendian = false;
        img->step = frame.get_width() * frame.get_bytes_per_pixel();
        seq_[stream] += 1;

        img->header.frame_id = optical_frame_id_[stream];
        img->header.stamp = ros::Time(frame.get_timestamp());
        img->header.seq = seq_[stream];
        image_publishers_[stream].publish(img);

        camera_info_[stream].header.stamp = img->header.stamp; 
        camera_info_[stream].header.seq = img->header.seq;
        info_publisher_[stream].publish(camera_info_[stream]);
      });

      // Publish info about the stream
      getStreamCalibData(stream);

      // Setup stream callback for stream
      image_[stream] = cv::Mat(width_[stream], height_[stream], image_format_[stream], cv::Scalar(0, 0, 0));

      ROS_INFO_STREAM("  enabled " << stream_name_[stream] << " stream, width: " << camera_info_[stream].width << " height: " << camera_info_[stream].height << " fps: " << fps_[stream]);
    }//end for

    cameraStarted = true;
  }//end setupStreams


  void getStreamCalibData(rs2_stream stream)
  {
    const rs2_intrinsics intrinsic = device.get_intrinsics({stream, width_[stream], height_[stream], fps_[stream], format_[stream]});

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

  //   if (stream == rs::stream::depth)
  //   {
  //     // set depth to color translation values in Projection matrix (P)
  //     rs::extrinsics extrinsic = device->get_extrinsics(rs::stream::depth, rs::stream::color);
  //     camera_info_[stream].P.at(3) = extrinsic.translation[0];     // Tx
  //     camera_info_[stream].P.at(7) = extrinsic.translation[1];     // Ty
  //     camera_info_[stream].P.at(11) = extrinsic.translation[2];    // Tz

  //     for (int i = 0; i < 9; i++)
  //       camera_info_[stream].R.at(i) = extrinsic.rotation[i];
  //   }

  //   switch ((int32_t)intrinsic.model())
  //   {
  //   case 0:
  //     camera_info_[stream].distortion_model = "none";
  //     break;
  //   case 1:
  //     camera_info_[stream].distortion_model = "plumb_bob"; // This is the same as "modified_brown_conrady", but used by ROS
  //     break;
  //   case 2:
  //     camera_info_[stream].distortion_model = "inverse_brown_conrady";
  //     break;
  //   case 3:
  //     camera_info_[stream].distortion_model = "distortion_ftheta";
  //     break;
  //   default:
  //     camera_info_[stream].distortion_model = "others";
  //     break;
  //   }

  //   // set R (rotation matrix) values to identity matrix
  //   if (stream != rs::stream::depth)
  //   {
  //     camera_info_[stream].R.at(0) = 1.0;
  //     camera_info_[stream].R.at(1) = 0.0;
  //     camera_info_[stream].R.at(2) = 0.0;
  //     camera_info_[stream].R.at(3) = 0.0;
  //     camera_info_[stream].R.at(4) = 1.0;
  //     camera_info_[stream].R.at(5) = 0.0;
  //     camera_info_[stream].R.at(6) = 0.0;
  //     camera_info_[stream].R.at(7) = 0.0;
  //     camera_info_[stream].R.at(8) = 1.0;
  //   }

    for (int i = 0; i < 5; i++)
    {
      camera_info_[stream].D.push_back(intrinsic.coeffs[i]);
    }
  }//end getStreamCalibData


  // void publishStaticTransforms()
  // {
  //   // Publish transforms for the cameras
  //   tf::Quaternion q_c2co;
  //   tf::Quaternion q_d2do;
  //   tf::Quaternion q_i2io;
  //   geometry_msgs::TransformStamped b2d_msg;
  //   geometry_msgs::TransformStamped d2do_msg;
  //   geometry_msgs::TransformStamped b2c_msg;
  //   geometry_msgs::TransformStamped c2co_msg;
  //   geometry_msgs::TransformStamped b2i_msg;
  //   geometry_msgs::TransformStamped i2io_msg;

  //   // Get the current timestamp for all static transforms
  //   ros::Time transform_ts_ = ros::Time::now();

  //   // The color frame is used as the base frame.
  //   // Hence no additional transformation is done from base frame to color frame.
  //   b2c_msg.header.stamp = transform_ts_;
  //   b2c_msg.header.frame_id = base_frame_id_;
  //   b2c_msg.child_frame_id = frame_id_[rs::stream::color];
  //   b2c_msg.transform.translation.x = 0;
  //   b2c_msg.transform.translation.y = 0;
  //   b2c_msg.transform.translation.z = 0;
  //   b2c_msg.transform.rotation.x = 0;
  //   b2c_msg.transform.rotation.y = 0;
  //   b2c_msg.transform.rotation.z = 0;
  //   b2c_msg.transform.rotation.w = 1;
  //   static_tf_broadcaster_.sendTransform(b2c_msg);

  //   // Transform color frame to color optical frame
  //   q_c2co.setRPY(-M_PI / 2, 0.0, -M_PI / 2);
  //   c2co_msg.header.stamp = transform_ts_;
  //   c2co_msg.header.frame_id = frame_id_[rs::stream::color];
  //   c2co_msg.child_frame_id = optical_frame_id_[rs::stream::color];
  //   c2co_msg.transform.translation.x = 0;
  //   c2co_msg.transform.translation.y = 0;
  //   c2co_msg.transform.translation.z = 0;
  //   c2co_msg.transform.rotation.x = q_c2co.getX();
  //   c2co_msg.transform.rotation.y = q_c2co.getY();
  //   c2co_msg.transform.rotation.z = q_c2co.getZ();
  //   c2co_msg.transform.rotation.w = q_c2co.getW();
  //   static_tf_broadcaster_.sendTransform(c2co_msg);

  //   // Transform base frame to depth frame
  //   rs::extrinsics color2depth_extrinsic = device->get_extrinsics(rs::stream::color, rs::stream::depth);
  //   b2d_msg.header.stamp = transform_ts_;
  //   b2d_msg.header.frame_id = base_frame_id_;
  //   b2d_msg.child_frame_id = frame_id_[rs::stream::depth];
  //   b2d_msg.transform.translation.x =  color2depth_extrinsic.translation[2];
  //   b2d_msg.transform.translation.y = -color2depth_extrinsic.translation[0];
  //   b2d_msg.transform.translation.z = -color2depth_extrinsic.translation[1];
  //   b2d_msg.transform.rotation.x = 0;
  //   b2d_msg.transform.rotation.y = 0;
  //   b2d_msg.transform.rotation.z = 0;
  //   b2d_msg.transform.rotation.w = 1;
  //   static_tf_broadcaster_.sendTransform(b2d_msg);

  //   // Transform depth frame to depth optical frame
  //   q_d2do.setRPY(-M_PI / 2, 0.0, -M_PI / 2);
  //   d2do_msg.header.stamp = transform_ts_;
  //   d2do_msg.header.frame_id = frame_id_[rs::stream::depth];
  //   d2do_msg.child_frame_id = optical_frame_id_[rs::stream::depth];
  //   d2do_msg.transform.translation.x = 0;
  //   d2do_msg.transform.translation.y = 0;
  //   d2do_msg.transform.translation.z = 0;
  //   d2do_msg.transform.rotation.x = q_d2do.getX();
  //   d2do_msg.transform.rotation.y = q_d2do.getY();
  //   d2do_msg.transform.rotation.z = q_d2do.getZ();
  //   d2do_msg.transform.rotation.w = q_d2do.getW();
  //   static_tf_broadcaster_.sendTransform(d2do_msg);

  //   if (isZR300)
  //   {
  //     tf::Quaternion q_f2fo, q_imu2imuo;
  //     geometry_msgs::TransformStamped b2f_msg;
  //     geometry_msgs::TransformStamped f2fo_msg;
  //     geometry_msgs::TransformStamped b2imu_msg;
  //     geometry_msgs::TransformStamped imu2imuo_msg;

  //     // Transform base frame to fisheye frame
  //     b2f_msg.header.stamp = transform_ts_;
  //     b2f_msg.header.frame_id = base_frame_id_;
  //     b2f_msg.child_frame_id = frame_id_[rs::stream::fisheye];
  //     rs::extrinsics color2fisheye_extrinsic = device->get_extrinsics(rs::stream::color, rs::stream::fisheye);
  //     b2f_msg.transform.translation.x =  color2fisheye_extrinsic.translation[2];
  //     b2f_msg.transform.translation.y = -color2fisheye_extrinsic.translation[0];
  //     b2f_msg.transform.translation.z = -color2fisheye_extrinsic.translation[1];
  //     b2f_msg.transform.rotation.x = 0;
  //     b2f_msg.transform.rotation.y = 0;
  //     b2f_msg.transform.rotation.z = 0;
  //     b2f_msg.transform.rotation.w = 1;
  //     static_tf_broadcaster_.sendTransform(b2f_msg);

  //     // Transform fisheye frame to fisheye optical frame
  //     q_f2fo.setRPY(-M_PI / 2, 0.0, -M_PI / 2);
  //     f2fo_msg.header.stamp = transform_ts_;
  //     f2fo_msg.header.frame_id = frame_id_[rs::stream::fisheye];
  //     f2fo_msg.child_frame_id = optical_frame_id_[rs::stream::fisheye];
  //     f2fo_msg.transform.translation.x = 0;
  //     f2fo_msg.transform.translation.y = 0;
  //     f2fo_msg.transform.translation.z = 0;
  //     f2fo_msg.transform.rotation.x = q_f2fo.getX();
  //     f2fo_msg.transform.rotation.y = q_f2fo.getY();
  //     f2fo_msg.transform.rotation.z = q_f2fo.getZ();
  //     f2fo_msg.transform.rotation.w = q_f2fo.getW();
  //     static_tf_broadcaster_.sendTransform(f2fo_msg);

  //     // // Transform base frame to imu frame
  //     // b2imu_msg.header.stamp = transform_ts_;
  //     // b2imu_msg.header.frame_id = base_frame_id_;
  //     // b2imu_msg.child_frame_id = imu_frame_id_;
  //     // rs::extrinsics color2imu_extrinsic = device->get_motion_extrinsics_from(rs::stream::color);
  //     // b2imu_msg.transform.translation.x =  color2imu_extrinsic.translation[2];
  //     // b2imu_msg.transform.translation.y = -color2imu_extrinsic.translation[0];
  //     // b2imu_msg.transform.translation.z = -color2imu_extrinsic.translation[1];
  //     // b2imu_msg.transform.rotation.x = 0;
  //     // b2imu_msg.transform.rotation.y = 0;
  //     // b2imu_msg.transform.rotation.z = 0;
  //     // b2imu_msg.transform.rotation.w = 1;
  //     // static_tf_broadcaster_.sendTransform(b2imu_msg);

  //     // // Transform imu frame to imu optical frame
  //     // q_imu2imuo.setRPY(-M_PI/2, 0.0, -M_PI/2);
  //     // imu2imuo_msg.header.stamp = transform_ts_;
  //     // imu2imuo_msg.header.frame_id = imu_frame_id_;
  //     // imu2imuo_msg.child_frame_id = optical_imu_frame_id_;
  //     // imu2imuo_msg.transform.translation.x = 0;
  //     // imu2imuo_msg.transform.translation.y = 0;
  //     // imu2imuo_msg.transform.translation.z = 0;
  //     // imu2imuo_msg.transform.rotation.x = q_imu2imuo.getX();
  //     // imu2imuo_msg.transform.rotation.y = q_imu2imuo.getY();
  //     // imu2imuo_msg.transform.rotation.z = q_imu2imuo.getZ();
  //     // imu2imuo_msg.transform.rotation.w = q_imu2imuo.getW();
  //     // static_tf_broadcaster_.sendTransform(imu2imuo_msg);
  //   }
  // }

private:
  ros::NodeHandle node_handle, pnh_;
  bool cameraStarted;
  std::unique_ptr<rs2::context > ctx;
  rs2::device device;

  std::string serial_no;

  std::map<rs2_stream, int> width_;
  std::map<rs2_stream, int> height_;
  std::map<rs2_stream, int> fps_;
  std::map<rs2_stream, bool> enable_;
  std::map<rs2_stream, std::string> stream_name_;
  // tf2_ros::StaticTransformBroadcaster static_tf_broadcaster_;
};//end class

}//end namespace

#include <pluginlib/class_list_macros.h>
PLUGINLIB_DECLARE_CLASS(realsense_ros_camera, NodeletDS5Camera, realsense_ros_camera::DS5NodeletCamera, nodelet::Nodelet);