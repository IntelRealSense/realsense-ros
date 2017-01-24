#include "SubscribeTopics.h"

SubscribeTopics::SubscribeTopics(){}
SubscribeTopics::~SubscribeTopics(){}
void SubscribeTopics::onInit(ros::NodeHandle & nh, rs::slam::slam * slam)
{
  l_nh = nh;
  l_slam = slam;
}
void SubscribeTopics::subscribeStreamMessages()
{
  std::string depthImageStream =  "camera/depth/depth_stream_and_info";
  std::string fisheyeImageStream =  "camera/fisheye/fisheye_stream_and_info";
  ROS_INFO_STREAM("Listening on " << depthImageStream);
  ROS_INFO_STREAM("Listening on " << fisheyeImageStream);
  l_depth_sub = l_nh.subscribe(depthImageStream, 100, & SubscribeTopics::depthMessageCallback, this);
  l_fisheye_sub = l_nh.subscribe(fisheyeImageStream, 100, & SubscribeTopics::fisheyeMessageCallback, this);
}
void SubscribeTopics::subscribeMotion()
{
  std::string motionInfo_gyro = "camera/imu/gyro";
  std::string motionInfo_accel = "camera/imu/accel";
  ROS_INFO_STREAM("Listening on " << motionInfo_gyro);
  ROS_INFO_STREAM("Listening on " << motionInfo_accel);
  l_motion_gyro_sub = l_nh.subscribe(motionInfo_gyro, 10000, & SubscribeTopics::motion_gyroCallback, this);
  l_motion_accel_sub = l_nh.subscribe(motionInfo_accel, 10000, & SubscribeTopics::motion_accelCallback, this);
}
void SubscribeTopics::depthMessageCallback(const realsense_camera::StreamInfoConstPtr & depthImageMsg)
{
  mut_depth.lock();
  SubscribeTopics::GetStreamSample(depthImageMsg, rs::core::stream_type::depth); 
  mut_depth.unlock();
}
void SubscribeTopics::fisheyeMessageCallback(const realsense_camera::StreamInfoConstPtr & fisheyeImageMsg)
{
  mut_fisheye.lock();
  SubscribeTopics::GetStreamSample(fisheyeImageMsg, rs::core::stream_type::fisheye); 
  mut_fisheye.unlock();
}
uint64_t seq_depth = 0, seq_fisheye = 0;
cv::Mat image_depth[100], image_fisheye[100];
uint64_t cpt_depth = 0, cpt_fisheye = 0;
rs::core::image_info info_fisheye[100], info_depth[100];
void SubscribeTopics::GetStreamSample(const realsense_camera::StreamInfoConstPtr & imageMsg, rs::core::stream_type stream)
{	    
  int width=imageMsg->image.width;
  int height=imageMsg->image.height;
  rs::core::correlated_sample_set sample_set = {};
  if (stream == rs::core::stream_type::fisheye){
    info_fisheye[cpt_fisheye] = {
                                 width,
                                 height,
                                 rs::utils::convert_pixel_format(rs::format::raw8),
                                 1 * width
                                 };
    image_fisheye[cpt_fisheye] = cv::Mat(height, width, CV_8UC1, (unsigned char*)imageMsg->image.data.data()).clone();           
    sample_set[stream] = rs::core::image_interface::create_instance_from_raw_data(
                                                       & info_fisheye[cpt_fisheye],
                                                       image_fisheye[cpt_fisheye].data,
                                                       stream,
                                                       rs::core::image_interface::flag::any,
                                                       (double)imageMsg->stamps,
                                                       (uint64_t)imageMsg->frame_number,
                                                       rs::core::timestamp_domain::microcontroller
                                                       );
    if (cpt_fisheye < 99)
    {
      cpt_fisheye++;
    }
    else
    {
      cpt_fisheye = 0;
    }
  }
  else
  {
    info_depth[cpt_depth] = {                                                            
                             width,
                             height,
                             rs::utils::convert_pixel_format(rs::format::z16),
                             2 * width
                            };
    image_depth[cpt_depth] = cv::Mat(height, width, CV_16UC1, (unsigned char*)imageMsg->image.data.data()).clone();
    sample_set[stream] = rs::core::image_interface::create_instance_from_raw_data(
                                                       & info_depth[cpt_depth],
                                                       image_depth[cpt_depth].data,
                                                       stream,
                                                       rs::core::image_interface::flag::any,
                                                       (double)imageMsg->stamps,
                                                       (uint64_t)imageMsg->frame_number,
                                                       rs::core::timestamp_domain::microcontroller
                                                       );
    if (cpt_depth < 99)
    {
      cpt_depth++;
    }
    else
    {
      cpt_depth = 0;
    }
  }
  if (l_slam->process_sample_set(sample_set) < rs::core::status_no_error)
  {
    ROS_ERROR("error: failed to process sample");
  }
  sample_set[stream]->release();
}//end of GetStreamSample
void SubscribeTopics::motion_gyroCallback(const realsense_camera::MotionInfoConstPtr & motionInfoMsg)
{
  mut_gyro_imu.lock();
  SubscribeTopics::GetMotionSample(motionInfoMsg, rs::core::motion_type::gyro);
  mut_gyro_imu.unlock();
}
void SubscribeTopics::motion_accelCallback(const realsense_camera::MotionInfoConstPtr & motionInfoMsg)
{
  mut_accel_imu.lock();
  SubscribeTopics::GetMotionSample(motionInfoMsg, rs::core::motion_type::accel);
  mut_accel_imu.unlock();
}
void SubscribeTopics::GetMotionSample(const realsense_camera::MotionInfoConstPtr & motionInfoMsg, rs::core::motion_type motionType)
{
  rs::core::correlated_sample_set sample_set = {};
  sample_set[motionType].timestamp = motionInfoMsg->stamps;
  sample_set[motionType].type = motionType;
  sample_set[motionType].frame_number= motionInfoMsg->frame_number;
  if (motionType == rs::core::motion_type::accel)
  {   
    sample_set[motionType].data[0] = (float)motionInfoMsg->imu.linear_acceleration.x;
    sample_set[motionType].data[1] = (float)motionInfoMsg->imu.linear_acceleration.y;
    sample_set[motionType].data[2] = (float)motionInfoMsg->imu.linear_acceleration.z;
  }
  else if (motionType == rs::core::motion_type::gyro)
  {
    sample_set[motionType].data[0] = (float)motionInfoMsg->imu.angular_velocity.x;
    sample_set[motionType].data[1] = (float)motionInfoMsg->imu.angular_velocity.y;
    sample_set[motionType].data[2] = (float)motionInfoMsg->imu.angular_velocity.z;
  }

  if (l_slam->process_sample_set(sample_set) < rs::core::status_no_error)
  {
    ROS_ERROR("error: failed to process sample");
  }
}

