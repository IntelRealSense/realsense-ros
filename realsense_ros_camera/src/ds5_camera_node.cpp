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
#include <tf/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>


namespace realsense_ros_camera
{
class DS5NodeletCamera: public nodelet::Nodelet
{
public:
  DS5NodeletCamera() :
    cameraStarted_(false)
  {
      // Types for depth stream
    format_[rs2_stream::RS2_STREAM_DEPTH] = RS2_FORMAT_Z16;   // libRS type
    image_format_[rs2_stream::RS2_STREAM_DEPTH] = CV_16UC1;    // CVBridge type
    encoding_[rs2_stream::RS2_STREAM_DEPTH] = sensor_msgs::image_encodings::TYPE_16UC1; // ROS message type
    unit_step_size_[rs2_stream::RS2_STREAM_DEPTH] = sizeof(uint16_t); // sensor_msgs::ImagePtr row step size
    stream_name_[rs2_stream::RS2_STREAM_DEPTH] = "depth";
  }

  virtual ~DS5NodeletCamera()
  {
    if (cameraStarted_)
    {
      device.stop();
    }
    usleep(250);
    ctx.reset();
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
      if (serial_no_.empty() || (serial_no_ == dev_serial_no))
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
    
      pointcloud_publisher_ = node_handle.advertise<sensor_msgs::PointCloud2>("/camera/points", 1);
    }
  }//end setupPublishers


  void setupStreams()
  {
    const rs2_stream All[] = {rs2_stream::RS2_STREAM_DEPTH};
    for (const auto stream : All )
    {
      if (false == enable_[stream])
        continue;    

      // Enable the stream
      device.open({ stream, width_[stream], height_[stream], fps_[stream], format_[stream] });

    // Publish info about the stream
      getStreamCalibData(stream);

      // Setup stream callback for stream
      image_[stream] = cv::Mat(width_[stream], height_[stream], image_format_[stream], cv::Scalar(0, 0, 0));

      // Start device with the following callback
      device.start([this,stream](rs2::frame frame)
      {
        image_[stream].data = (unsigned char *) frame.get_data();
        ros::Time t = ros::Time::now();

        if((stream == rs2_stream::RS2_STREAM_DEPTH) && (0 != pointcloud_publisher_.getNumSubscribers()))
          publishPCTopic(t);

        seq_[stream] += 1;
        if(0 != image_publishers_[stream].getNumSubscribers())
        {
          sensor_msgs::ImagePtr img;          
          img = cv_bridge::CvImage(std_msgs::Header(), encoding_[stream], image_[stream]).toImageMsg();
          img->width = frame.get_width();
          img->height = frame.get_height();
          img->is_bigendian = false;
          img->step = frame.get_width() * frame.get_bytes_per_pixel();
          img->header.frame_id = optical_frame_id_[stream];
          img->header.stamp = t;
          img->header.seq = seq_[stream];

          image_publishers_[stream].publish(img);
        }

        if(0 != image_publishers_[stream].getNumSubscribers())
        {
          camera_info_[stream].header.stamp = t; 
          camera_info_[stream].header.seq = seq_[stream];
          info_publisher_[stream].publish(camera_info_[stream]);
        }
      });

      ROS_INFO_STREAM("  enabled " << stream_name_[stream] << " stream, width: " << camera_info_[stream].width << " height: " << camera_info_[stream].height << " fps: " << fps_[stream]);
    }//end for

    cameraStarted_ = true;
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

    camera_info_[stream].distortion_model = "plumb_bob";

    // set R (rotation matrix) values to identity matrix
    if (stream != rs2_stream::RS2_STREAM_DEPTH)
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
    tf::Quaternion q_d2do;
    geometry_msgs::TransformStamped b2d_msg;
    geometry_msgs::TransformStamped d2do_msg;

    // Get the current timestamp for all static transforms
    ros::Time transform_ts_ = ros::Time::now();

    // Transform base frame to depth frame
    b2d_msg.header.stamp = transform_ts_;
    b2d_msg.header.frame_id = base_frame_id_;
    b2d_msg.child_frame_id = frame_id_[rs2_stream::RS2_STREAM_DEPTH];
    b2d_msg.transform.translation.x =  0; 
    b2d_msg.transform.translation.y = 0; 
    b2d_msg.transform.translation.z = 0;
    b2d_msg.transform.rotation.x = 0;
    b2d_msg.transform.rotation.y = 0;
    b2d_msg.transform.rotation.z = 0;
    b2d_msg.transform.rotation.w = 1;
    static_tf_broadcaster_.sendTransform(b2d_msg);

    // Transform depth frame to depth optical frame
    q_d2do.setRPY(-M_PI / 2, 0.0, -M_PI / 2);
    d2do_msg.header.stamp = transform_ts_;
    d2do_msg.header.frame_id = frame_id_[rs2_stream::RS2_STREAM_DEPTH];
    d2do_msg.child_frame_id = optical_frame_id_[rs2_stream::RS2_STREAM_DEPTH];
    d2do_msg.transform.translation.x = 0;
    d2do_msg.transform.translation.y = 0;
    d2do_msg.transform.translation.z = 0;
    d2do_msg.transform.rotation.x = q_d2do.getX();
    d2do_msg.transform.rotation.y = q_d2do.getY();
    d2do_msg.transform.rotation.z = q_d2do.getZ();
    d2do_msg.transform.rotation.w = q_d2do.getW();
    static_tf_broadcaster_.sendTransform(d2do_msg);
  }

  void publishPCTopic(ros::Time t)
  {
    const rs2_intrinsics depth_intrinsic = device.get_intrinsics({rs2_stream::RS2_STREAM_DEPTH, width_[rs2_stream::RS2_STREAM_DEPTH], height_[rs2_stream::RS2_STREAM_DEPTH], fps_[rs2_stream::RS2_STREAM_DEPTH], format_[rs2_stream::RS2_STREAM_DEPTH]});
    float depth_scale_meters = device.get_depth_scale();
      
    sensor_msgs::PointCloud2 msg_pointcloud;
    msg_pointcloud.header.stamp = t;
    msg_pointcloud.header.frame_id = optical_frame_id_[rs2_stream::RS2_STREAM_DEPTH];
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
        memcpy(&depth_value, &image_[rs2_stream::RS2_STREAM_DEPTH].data[depth_offset], sizeof(uint16_t));
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
  
    pointcloud_publisher_.publish(msg_pointcloud);
  }

private:
  ros::NodeHandle node_handle, pnh_;
  bool cameraStarted_;
  std::unique_ptr<rs2::context > ctx;
  rs2::device device;

  std::string serial_no_;
  std::string usb_port_id_;
  std::string camera_type_;

  std::map<rs2_stream, int> width_;
  std::map<rs2_stream, int> height_;
  std::map<rs2_stream, int> fps_;
  std::map<rs2_stream, bool> enable_;
  std::map<rs2_stream, std::string> stream_name_;
  tf2_ros::StaticTransformBroadcaster static_tf_broadcaster_;

  // DS5 types
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
  std::map<rs2_stream, sensor_msgs::CameraInfo> camera_info_;
 
  ros::Publisher pointcloud_publisher_;
};//end class

PLUGINLIB_DECLARE_CLASS(realsense_ros_camera, NodeletDS5Camera, realsense_ros_camera::DS5NodeletCamera, nodelet::Nodelet);

}//end namespace
