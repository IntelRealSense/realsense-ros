// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved

#include "realsense_ros_slam/slam_nodelet.h"
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <realsense_ros_slam/TrackingAccuracy.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf/transform_broadcaster.h>

PLUGINLIB_EXPORT_CLASS(realsense_ros_slam::SNodeletSlam, nodelet::Nodelet)

uint64_t seq_depth = 0, seq_fisheye = 0;
cv::Mat image_depth[100], image_fisheye[100];
uint64_t cpt_depth = 0, cpt_fisheye = 0;
rs::core::image_info info_fisheye[100], info_depth[100];

std::string pkgpath;
std::string trajectoryFilename;
std::string relocalizationFilename;
std::string occupancyFilename;
std::string topic_camera_pose, topic_reloc_pose, topic_pose2d, topic_map, topic_tracking_accuracy, topic_odom;
double map_resolution, hoi_min, hoi_max, doi_min, doi_max;
bool is_pub_odom = false;

ros::Publisher pub_pose2d, pub_pose, pub_map, pub_accuracy, pub_reloc, pub_odom;
geometry_msgs::Pose2D pose2d;

IplImage * ipNavMap = NULL;
std::vector< stRobotPG > g_robotPGStack;
std::vector< CvPoint > g_relocalizationPointStack;

namespace realsense_ros_slam
{

SubscribeTopics::SubscribeTopics()
{
}


SubscribeTopics::~SubscribeTopics()
{
}


void SubscribeTopics::onInit(ros::NodeHandle & nh, rs::slam::slam * slam)
{
  l_nh = nh;
  l_slam = slam;
}

void SubscribeTopics::subscribeStreamMessages()
{
  std::string depthImageStream =  "camera/depth/image_raw";
  std::string fisheyeImageStream =  "camera/fisheye/image_raw";
  ROS_INFO_STREAM("Listening on " << depthImageStream);
  ROS_INFO_STREAM("Listening on " << fisheyeImageStream);
  l_depth_sub = l_nh.subscribe(depthImageStream, 100, & SubscribeTopics::depthMessageCallback, this);
  l_fisheye_sub = l_nh.subscribe(fisheyeImageStream, 100, & SubscribeTopics::fisheyeMessageCallback, this);
}

void SubscribeTopics::subscribeMotion()
{
  std::string motionInfo_gyro = "camera/gyro/sample";
  std::string motionInfo_accel = "camera/accel/sample";
  ROS_INFO_STREAM("Listening on " << motionInfo_gyro);
  ROS_INFO_STREAM("Listening on " << motionInfo_accel);
  l_motion_gyro_sub = l_nh.subscribe(motionInfo_gyro, 10000, & SubscribeTopics::motionGyroCallback, this);
  l_motion_accel_sub = l_nh.subscribe(motionInfo_accel, 10000, & SubscribeTopics::motionAccelCallback, this);
}

void SubscribeTopics::depthMessageCallback(const sensor_msgs::ImageConstPtr &depthImageMsg)
{
  mut_depth.lock();
  SubscribeTopics::getStreamSample(depthImageMsg, rs::core::stream_type::depth);
  mut_depth.unlock();
}


void SubscribeTopics::fisheyeMessageCallback(const sensor_msgs::ImageConstPtr &fisheyeImageMsg)
{
  mut_fisheye.lock();
  SubscribeTopics::getStreamSample(fisheyeImageMsg, rs::core::stream_type::fisheye);
  mut_fisheye.unlock();
}

void SubscribeTopics::getStreamSample(const sensor_msgs::ImageConstPtr &imageMsg, rs::core::stream_type stream)
{
  int width = imageMsg->width;
  int height = imageMsg->height;
  rs::core::correlated_sample_set sample_set = {};
  if (stream == rs::core::stream_type::fisheye)
  {
    info_fisheye[cpt_fisheye] =
    {
      width,
      height,
      rs::utils::convert_pixel_format(rs::format::raw8),
      1 * width
    };
    image_fisheye[cpt_fisheye] = cv::Mat(height, width, CV_8UC1, (unsigned char*)imageMsg->data.data()).clone();
    sample_set[stream] = rs::core::image_interface::create_instance_from_raw_data(
                           & info_fisheye[cpt_fisheye],
                           image_fisheye[cpt_fisheye].data,
                           stream,
                           rs::core::image_interface::flag::any,
                           imageMsg->header.stamp.toSec(),
                           (uint64_t)imageMsg->header.seq,
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
    info_depth[cpt_depth] =
    {
      width,
      height,
      rs::utils::convert_pixel_format(rs::format::z16),
      2 * width
    };
    image_depth[cpt_depth] = cv::Mat(height, width, CV_16UC1, (unsigned char*)imageMsg->data.data()).clone();
    sample_set[stream] = rs::core::image_interface::create_instance_from_raw_data(
                           & info_depth[cpt_depth],
                           image_depth[cpt_depth].data,
                           stream,
                           rs::core::image_interface::flag::any,
                           imageMsg->header.stamp.toSec(),
                           (uint64_t)imageMsg->header.seq,
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
}//end of getStreamSample


void SubscribeTopics::motionGyroCallback(const sensor_msgs::ImuConstPtr &imuMsg)
{
  mut_gyro_imu.lock();
  SubscribeTopics::getMotionSample(imuMsg, rs::core::motion_type::gyro);
  mut_gyro_imu.unlock();
}


void SubscribeTopics::motionAccelCallback(const sensor_msgs::ImuConstPtr &imuMsg)
{
  mut_accel_imu.lock();
  SubscribeTopics::getMotionSample(imuMsg, rs::core::motion_type::accel);
  mut_accel_imu.unlock();
}


void SubscribeTopics::getMotionSample(const sensor_msgs::ImuConstPtr &imuMsg, rs::core::motion_type motionType)
{
  rs::core::correlated_sample_set sample_set = {};
  sample_set[motionType].timestamp = imuMsg->header.stamp.toSec();
  sample_set[motionType].type = motionType;
  sample_set[motionType].frame_number = imuMsg->header.seq;
  if (motionType == rs::core::motion_type::accel)
  {
    sample_set[motionType].data[0] = (float)imuMsg->linear_acceleration.x;
    sample_set[motionType].data[1] = (float)imuMsg->linear_acceleration.y;
    sample_set[motionType].data[2] = (float)imuMsg->linear_acceleration.z;
  }
  else if (motionType == rs::core::motion_type::gyro)
  {
    sample_set[motionType].data[0] = (float)imuMsg->angular_velocity.x;
    sample_set[motionType].data[1] = (float)imuMsg->angular_velocity.y;
    sample_set[motionType].data[2] = (float)imuMsg->angular_velocity.z;
  }

  if (l_slam->process_sample_set(sample_set) < rs::core::status_no_error)
  {
    ROS_ERROR("error: failed to process sample");
  }
}


class slam_tracking_event_handler : public rs::slam::tracking_event_handler
{
public:
  slam_tracking_event_handler()
  {
    ROS_INFO("tracking_event_handler");
  }


  void on_restart()
  {
    ROS_INFO("Restart--------------------------------");
    fflush(stdout);
  }


  ~slam_tracking_event_handler()
  {
    ROS_INFO("deconstructure");
  }
};


void convertToPG(rs::slam::PoseMatrix4f & pose, Eigen::Vector3f & gravity, stRobotPG & robotPG)
{
  Eigen::Vector3f g = gravity;
  Eigen::Vector3f g_startG(0.0f, 1.0f, 0.0f);
  Eigen::Vector3f x3D(1.0f, 0.0f, 0.0f);
  Eigen::Vector3f y3D(0.0f, 1.0f, 0.0f);
  Eigen::Vector3f z3D(0.0f, 0.0f, 1.0f);
  Eigen::Vector3f x3D_(pose.at(0, 0), pose.at(0, 1), pose.at(0, 2));
  Eigen::Vector3f y3D_(pose.at(1, 0), pose.at(1, 1), pose.at(1, 2));
  Eigen::Vector3f z3D_(pose.at(2, 0), pose.at(2, 1), pose.at(2, 2));
  Eigen::Vector3f z3DTo_(z3D.dot(x3D_), z3D.dot(y3D_), z3D.dot(z3D_));
  Eigen::Vector3f ProjX = g.cross(z3D);
  Eigen::Vector3f ProjY = ProjX.cross(g);
  Eigen::Vector2f v(z3DTo_.dot(ProjX), z3DTo_.dot(ProjY));
  Eigen::Vector2f v_(z3D.dot(ProjX), z3D.dot(ProjY));
  float theta1 = atan2(v.x(), v.y());
  float theta2 = atan2(v_.x(), v_.y());
  float theta = theta2 - theta1;
  theta += CV_PI / 2; // Needed to point the right direction
  if (theta < 0)
  {
    theta += 2 * CV_PI;
  }
  else if (theta >= 2 * CV_PI)
  {
    theta -= 2 * CV_PI;
  }
  Eigen::Vector3f origin_(pose.at(0, 3), pose.at(1, 3), pose.at(2, 3));
  Eigen::Vector3f ProjX0 = g_startG.cross(z3D);
  Eigen::Vector3f ProjY0 = ProjX0.cross(g_startG);
  float x = origin_.dot(ProjX0);
  float y = origin_.dot(ProjY0);
  robotPG.initdefault();
  robotPG.theta = theta;
  robotPG.x = x;
  robotPG.y = y;
}


void extrinsicsMsgToRSExtrinsics(const realsense_ros_camera::ExtrinsicsConstPtr &extrinsicsMsg, rs::core::extrinsics &rsExtrinsics)
{
  for (int i = 0; i < 9; ++i)
  {
    rsExtrinsics.rotation[i] = extrinsicsMsg->rotation[i];
    if (i < 3) rsExtrinsics.translation[i] = extrinsicsMsg->translation[i];
  }
}

void imuInfoMsgToRSImuIntrinsics(const realsense_ros_camera::IMUInfoConstPtr &imuInfoMsg, rs::core::motion_device_intrinsics &rsImuIntrinsics)
{
  int index = 0;
  for (int i = 0; i < 3; ++i)
  {
    for (int j = 0; j < 4; ++j)
    {
      rsImuIntrinsics.data[i][j] = imuInfoMsg->data[index];
      ++index;
    }
    rsImuIntrinsics.noise_variances[i] = imuInfoMsg->noise_variances[i];
    rsImuIntrinsics.bias_variances[i] = imuInfoMsg->bias_variances[i];
  }
}

tf2::Quaternion quaternionFromPoseMatrix(rs::slam::PoseMatrix4f cameraPose)
{
  tf2::Matrix3x3 rotMat = tf2::Matrix3x3(
                            cameraPose.at(0, 0),
                            cameraPose.at(0, 1),
                            cameraPose.at(0, 2),
                            cameraPose.at(1, 0),
                            cameraPose.at(1, 1),
                            cameraPose.at(1, 2),
                            cameraPose.at(2, 0),
                            cameraPose.at(2, 1),
                            cameraPose.at(2, 2)
                          );

  tf2::Quaternion quat;
  rotMat.getRotation(quat);
  return quat;
}

geometry_msgs::Pose poseMatrixToMsg(rs::slam::PoseMatrix4f camera_pose)
{
  tf2::Quaternion quat = quaternionFromPoseMatrix(camera_pose);
  geometry_msgs::Quaternion quat_msg;
  tf2::convert<tf2::Quaternion, geometry_msgs::Quaternion>(quat, quat_msg);

  geometry_msgs::Point point_msg;
  point_msg.x = camera_pose.at(0, 3);
  point_msg.y = camera_pose.at(1, 3);
  point_msg.z = camera_pose.at(2, 3);

  geometry_msgs::Pose pose_msg;
  pose_msg.orientation = quat_msg;
  pose_msg.position = point_msg;
  return pose_msg;
}

geometry_msgs::PoseStamped getPoseStampedMsg(rs::slam::PoseMatrix4f cameraPose, uint64_t frameNum, double timestamp_ms)
{
  std_msgs::Header header;
  header.stamp = ros::Time(timestamp_ms / 1000);
  header.frame_id = "camera_link";
  if (frameNum > INT32_MAX)
  {
    header.seq = frameNum - INT32_MAX;
  }
  else
  {
    header.seq = frameNum;
  }

  geometry_msgs::PoseStamped pose_stamped_msg;
  pose_stamped_msg.header = header;
  pose_stamped_msg.pose = poseMatrixToMsg(cameraPose);
  return pose_stamped_msg;
}

class slam_event_handler : public rs::core::video_module_interface::processing_event_handler
{
public:
  std::shared_ptr< rs::slam::occupancy_map > occ_map;
  slam_event_handler()
  {
    std::cout << "created.........." << std::endl;
  }

  void module_output_ready(rs::core::video_module_interface * sender, rs::core::correlated_sample_set * sample)
  {
    rs::slam::slam *slamPtr = dynamic_cast< rs::slam::slam * >(sender);
    uint64_t feFrameNum = sample->images[static_cast<uint8_t>(rs::core::stream_type::fisheye)]->query_frame_number();
    double feTimeStamp = sample->images[static_cast<uint8_t>(rs::core::stream_type::fisheye)]->query_time_stamp();

    // Publish camera pose
    rs::slam::PoseMatrix4f cameraPose;
    slamPtr->get_camera_pose(cameraPose);
    geometry_msgs::PoseStamped pose_msg = getPoseStampedMsg(cameraPose, feFrameNum, feTimeStamp);
    pub_pose.publish(pose_msg);

    // Publish relocalized camera pose, if any
    rs::slam::PoseMatrix4f relocPose;
    if (slamPtr->get_relocalization_pose(relocPose))
    {
      geometry_msgs::PoseStamped reloc_pose_msg = getPoseStampedMsg(relocPose, feFrameNum, feTimeStamp);
      pub_reloc.publish(reloc_pose_msg);
    }

    // Publish tracking accuracy
    rs::slam::tracking_accuracy accuracy = slamPtr->get_tracking_accuracy();
    TrackingAccuracy accuracyMsg;
    accuracyMsg.header.stamp = ros::Time(feTimeStamp);
    accuracyMsg.header.seq = feFrameNum;
    accuracyMsg.tracking_accuracy = (uint32_t)accuracy;
    pub_accuracy.publish(accuracyMsg);

    // Publish 2D pose
    Eigen::Vector3f gravity = Eigen::Vector3f(0, 1, 0);
    stRobotPG robotPG;
    convertToPG(cameraPose, gravity, robotPG);
    pose2d.x = robotPG.x;
    pose2d.y = robotPG.y;
    pose2d.theta = robotPG.theta;
    pub_pose2d.publish(pose2d);
    g_robotPGStack.push_back(robotPG);

    // Publish odometry
    if (is_pub_odom)
    {
      nav_msgs::Odometry odom;
      odom.header.stamp = ros::Time(feTimeStamp);
      odom.header.seq = feFrameNum;
      odom.header.frame_id = "odom";
      odom.pose.pose.position.x = pose2d.x;
      odom.pose.pose.position.y = pose2d.y;
      tf2::Quaternion quat(tf2::Vector3(0, 0, 1), pose2d.theta); // Rotate around the z axis by angle theta
      tf2::convert<tf2::Quaternion, geometry_msgs::Quaternion>(quat, odom.pose.pose.orientation);
      pub_odom.publish(odom);
    }

    // Publish occupancy map
    int wmap = 512;
    int hmap = 512;
    if (!occ_map)
    {
      occ_map = slamPtr->create_occupancy_map(wmap * hmap);
      std::cout << " creating occupancy map: resolution: " << slamPtr->get_occupancy_map_resolution() << std::endl;
    }
    if (ipNavMap == NULL)
    {
      ipNavMap = cvCreateImage(cvSize(wmap, hmap), 8, 1);
      cvSet(ipNavMap, 10, NULL);
    }
    int status = slamPtr->get_occupancy_map_update(occ_map);
    int count = occ_map->get_tile_count();
    nav_msgs::OccupancyGrid map_msg;
    if (status >= 0 && count > 0)
    {
      const int32_t* map = occ_map->get_tile_coordinates();
      for (int i = 0; i < count; i++)
      {
        int _x = map[3 * i] + wmap / 2;
        int _y = map[3 * i + 1] + hmap / 2;
        if (_x >= 0 && _x < wmap)
        {
          if (_y >= 0 && _y < hmap)
          {
            int V = map[3 * i + 2];
            ipNavMap->imageData[wmap * _y + _x] = V;
          }
        }
      }
    }
    std::vector<signed char> vMap(ipNavMap->imageData, ipNavMap->imageData + wmap * hmap);
    map_msg.data = vMap;
    map_msg.info.resolution = map_resolution;
    map_msg.info.width      = wmap;
    map_msg.info.height     = hmap;
    map_msg.info.origin.position.x = -(wmap / 2) * map_resolution;
    map_msg.info.origin.position.y = -(hmap / 2) * map_resolution;
    pub_map.publish(map_msg);
  }

  ~slam_event_handler()
  {
    ROS_INFO("deconstructure event handler");
  }
};


SNodeletSlam::SNodeletSlam()
{
}


SNodeletSlam::~SNodeletSlam()
{
}


void SNodeletSlam::onInit()
{
  ros::NodeHandle pnh = getPrivateNodeHandle();
  pnh.param< double >("map_resolution", map_resolution, 0.05);
  pnh.param< double >("hoi_min", hoi_min, -0.5);
  pnh.param< double >("hoi_max", hoi_max, 0.1);
  pnh.param< double >("doi_min", doi_min, 0.3);
  pnh.param< double >("doi_max", doi_max, 3.0);
  pnh.param< std::string >("trajectoryFilename", trajectoryFilename, "trajectory.ppm");
  pnh.param< std::string >("relocalizationFilename", relocalizationFilename, "relocalization.bin");
  pnh.param< std::string >("occupancyFilename", occupancyFilename, "occupancy.bin");
  pnh.param< std::string >("topic_camera_pose", topic_camera_pose, "camera_pose");
  pnh.param< std::string >("topic_reloc_pose", topic_reloc_pose, "reloc_pose");
  pnh.param< std::string >("topic_pose2d", topic_pose2d, "pose2d");
  pnh.param< std::string >("topic_map", topic_map, "map");
  pnh.param< std::string >("topic_tracking_accuracy", topic_tracking_accuracy, "tracking_accuracy");
  pnh.param< std::string >("topic_odom", topic_odom, "odom");
  pnh.param< bool >("publish_odometry", is_pub_odom, false);

  nh = getMTNodeHandle();
  pub_pose = nh.advertise< geometry_msgs::PoseStamped >(topic_camera_pose, 1, true);
  pub_reloc = nh.advertise< geometry_msgs::PoseStamped >(topic_reloc_pose, 1, true);
  pub_pose2d = nh.advertise< geometry_msgs::Pose2D >(topic_pose2d, 2, true);
  pub_map = nh.advertise< nav_msgs::OccupancyGrid >(topic_map, 1, true);
  pub_accuracy = nh.advertise< realsense_ros_slam::TrackingAccuracy >(topic_tracking_accuracy, 1, true);
  if (is_pub_odom) pub_odom = nh.advertise< nav_msgs::Odometry >(topic_odom, 1, true);
  pkgpath = ros::package::getPath("realsense_ros_slam") + "/";

  sub_depthInfo = nh.subscribe("camera/depth/camera_info", 1, &SNodeletSlam::depthInfoCallback, this);
  sub_fisheyeInfo = nh.subscribe("camera/fisheye/camera_info", 1, &SNodeletSlam::fisheyeInfoCallback, this);
  sub_accelInfo = nh.subscribe("camera/accel/imu_info", 1, &SNodeletSlam::accelInfoCallback, this);
  sub_gyroInfo = nh.subscribe("camera/gyro/imu_info", 1, &SNodeletSlam::gyroInfoCallback, this);
  sub_fe2imu = nh.subscribe("camera/extrinsics/fisheye2imu", 1, &SNodeletSlam::fe2ImuCallback, this);
  sub_fe2depth = nh.subscribe("camera/extrinsics/fisheye2depth", 1, &SNodeletSlam::fe2depthCallback, this);

  actual_config = {};
  ROS_INFO("end of onInit");
}//end onInit

void SNodeletSlam::depthInfoCallback(const sensor_msgs::CameraInfoConstPtr& depthCameraInfo)
{
  sub_depthInfo.shutdown();
  mut_init.lock();
  depthCameraInfo_ = depthCameraInfo;
  startIfReady();
  mut_init.unlock();
}

void SNodeletSlam::fisheyeInfoCallback(const sensor_msgs::CameraInfoConstPtr& fisheyeCameraInfo)
{
  sub_fisheyeInfo.shutdown();
  mut_init.lock();
  fisheyeCameraInfo_ = fisheyeCameraInfo;
  startIfReady();
  mut_init.unlock();
}

void SNodeletSlam::accelInfoCallback(const realsense_ros_camera::IMUInfoConstPtr& accelInfo)
{
  sub_accelInfo.shutdown();
  mut_init.lock();
  accelInfo_ = accelInfo;
  startIfReady();
  mut_init.unlock();
}

void SNodeletSlam::gyroInfoCallback(const realsense_ros_camera::IMUInfoConstPtr& gyroInfo)
{
  sub_gyroInfo.shutdown();
  mut_init.lock();
  gyroInfo_ = gyroInfo;
  startIfReady();
  mut_init.unlock();
}

void SNodeletSlam::fe2ImuCallback(const realsense_ros_camera::ExtrinsicsConstPtr& fe2imu)
{
  sub_fe2imu.shutdown();
  mut_init.lock();
  fe2imu_ = fe2imu;
  startIfReady();
  mut_init.unlock();
}

void SNodeletSlam::fe2depthCallback(const realsense_ros_camera::ExtrinsicsConstPtr& fe2depth)
{
  sub_fe2depth.shutdown();
  mut_init.lock();
  fe2depth_ = fe2depth;
  startIfReady();
  mut_init.unlock();
}

void SNodeletSlam::startIfReady()
{
  // If we have all the messages we were waiting for, start SLAM.
  if (depthCameraInfo_ && fisheyeCameraInfo_ && accelInfo_ && gyroInfo_ && fe2imu_ && fe2depth_)
  {
    startSlam();
  }
}

void SNodeletSlam::startSlam()
{
  ROS_INFO("Staring SLAM...");

  std::unique_ptr<rs::slam::slam> slam(new rs::slam::slam());
  slam->set_occupancy_map_resolution(map_resolution);
  slam->set_occupancy_map_height_of_interest(hoi_min, hoi_max);
  slam->set_occupancy_map_depth_of_interest(doi_min, doi_max);
  slam->force_relocalization_pose(false);

  slam_event_handler scenePerceptionEventHandler;
  slam->register_event_handler(&scenePerceptionEventHandler);

  slam_tracking_event_handler trackingEventHandler;
  slam->register_tracking_event_handler(&trackingEventHandler);

  supported_config = {};
  if (slam->query_supported_module_config(0, supported_config) < rs::core::status_no_error)
  {
    std::cerr << "error : failed to query the first supported module configuration" << std::endl;
    return ;
  }

  // Set camera intrinsics
  rs::core::intrinsics  depth_intrinsics, fisheye_intrinsics;
  SNodeletSlam::setCalibrationData(depthCameraInfo_, depth_intrinsics);
  SNodeletSlam::setCalibrationData(fisheyeCameraInfo_, fisheye_intrinsics);

  std::map< rs::core::stream_type, rs::core::intrinsics > intrinsics;
  intrinsics[rs::core::stream_type::depth] = depth_intrinsics;
  intrinsics[rs::core::stream_type::fisheye] = fisheye_intrinsics;
  SNodeletSlam::setStreamConfigIntrin(rs::core::stream_type::depth, intrinsics);
  SNodeletSlam::setStreamConfigIntrin(rs::core::stream_type::fisheye, intrinsics);

  // Set IMU intrinsics
  rs::core::motion_device_intrinsics acc, gyro;
  imuInfoMsgToRSImuIntrinsics(accelInfo_, acc);
  imuInfoMsgToRSImuIntrinsics(gyroInfo_, gyro);
  actual_config[rs::core::motion_type::accel].is_enabled = true;
  actual_config[rs::core::motion_type::accel].intrinsics = acc;
  actual_config[rs::core::motion_type::gyro].is_enabled = true;
  actual_config[rs::core::motion_type::gyro].intrinsics = gyro;

  // Set extrinsics
  rs::core::extrinsics fe2imu, fe2depth;
  extrinsicsMsgToRSExtrinsics(fe2imu_, fe2imu);
  extrinsicsMsgToRSExtrinsics(fe2depth_, fe2depth);
  actual_config[rs::core::stream_type::fisheye].extrinsics_motion = fe2imu;
  actual_config[rs::core::stream_type::fisheye].extrinsics = fe2depth;

  // Set actual config
  if (slam->set_module_config(actual_config) < rs::core::status_no_error)
  {
    NODELET_ERROR("error : failed to set the enabled module configuration");
    return ;
  }

  //slam->load_relocalization_map(relocalizationFilename);
  sub.onInit(nh, slam.get());
  sub.subscribeStreamMessages();
  sub.subscribeMotion();

  ros::Rate r(30);
  while (ros::ok())
  {
    r.sleep();
  }

  std::string result = slam->save_occupancy_map_as_ppm(pkgpath + trajectoryFilename, true) == 0 ? "Saved trajectory to file." : "FAILED to save trajectory to file.";
  std::cout << result << std::endl;
  result = slam->save_relocalization_map(pkgpath + relocalizationFilename) == 0 ? "Saved relocalization map to file." : "FAILED to save relocalization map to file.";
  std::cout << result << std::endl;
  result = slam->save_occupancy_map(pkgpath + occupancyFilename) == 0 ? "Saved occupancy map to file." : "FAILED to save occupancy map to file.";
  std::cout << result << std::endl;

  slam->flush_resources();
}//end of callback


void SNodeletSlam::setCalibrationData(const sensor_msgs::CameraInfoConstPtr & cameraInfoMsg, rs::core::intrinsics & cameraInfo)
{
  NODELET_INFO("setCalibrationData ");
  cameraInfo.width = cameraInfoMsg->width;
  cameraInfo.height = cameraInfoMsg->height;
  cameraInfo.fx = cameraInfoMsg->K[0];
  cameraInfo.fy = cameraInfoMsg->K[4];
  cameraInfo.ppx = cameraInfoMsg->K[2];
  cameraInfo.ppy = cameraInfoMsg->K[5];
  for (int i = 0; i < 5; i++)
    cameraInfo.coeffs[i] = (float)cameraInfoMsg->D[i];
  std::string distortion_model = cameraInfoMsg->distortion_model;
  if (distortion_model.compare("modified_brown_conrady") == 0)
  {
    cameraInfo.model = rs::core::distortion_type::modified_brown_conrady;
  }
  else if (distortion_model.compare("none") == 0)
  {
    cameraInfo.model = rs::core::distortion_type::none;
  }
  else if (distortion_model.compare("distortion_ftheta") == 0)
  {
    cameraInfo.model = rs::core::distortion_type::distortion_ftheta;
  }
  else
  {
    cameraInfo.model = rs::core::distortion_type::none;
  }
}


void SNodeletSlam::setStreamConfigIntrin(rs::core::stream_type stream, std::map< rs::core::stream_type, rs::core::intrinsics > intrinsics)
{
  auto & supported_stream_config = supported_config[stream];
  if (!supported_stream_config.is_enabled || supported_stream_config.size.width != intrinsics[stream].width || supported_stream_config.size.height != intrinsics[stream].height)
  {
    ROS_ERROR("size of stream is not supported by slam");
    ROS_ERROR_STREAM("  supported: stream " << (uint32_t) stream << ", width: " << supported_stream_config.size.width  << " height: " << supported_stream_config.size.height);
    ROS_ERROR_STREAM("  received: stream " << (uint32_t) stream << ", width: " << intrinsics[stream].width  << " height: " << intrinsics[stream].height);

    return;
  }
  rs::core::video_module_interface::actual_image_stream_config &actual_stream_config = actual_config[stream];
  actual_config[stream].size.width = intrinsics[stream].width;
  actual_config[stream].size.height = intrinsics[stream].height;
  actual_stream_config.frame_rate = supported_stream_config.frame_rate;
  actual_stream_config.intrinsics = intrinsics[stream];
  actual_stream_config.is_enabled = true;
}//end of setStremConfigIntrin


void SNodeletSlam::setMotionData(realsense_ros_camera::IMUInfo & imu_res, rs::core::motion_device_intrinsics & motion_intrin)
{
  int index = 0;
  for (int i = 0; i < 3; ++i)
  {
    for (int j = 0; j < 4; ++j)
    {
      motion_intrin.data[i][j] = imu_res.data[index];
      ++index;
    }
    motion_intrin.noise_variances[i] = imu_res.noise_variances[i];
    motion_intrin.bias_variances[i] = imu_res.bias_variances[i];
  }
}//end of setMotionData


void SNodeletSlam::setMotionConfigIntrin(rs::core::motion_type motion, std::map< rs::core::motion_type, rs::core::motion_device_intrinsics > motion_intrinsics)
{
  actual_config[motion].is_enabled = true;
  actual_config[motion].intrinsics = motion_intrinsics[motion];
}//end of setMotionConfigIntrin


void SNodeletSlam::setExtrinData(realsense_ros_camera::Extrinsics & fe_res, rs::core::extrinsics & extrinsics)
{
  for (int i = 0; i < 9; ++i)
  {
    extrinsics.rotation[i] = fe_res.rotation[i];
    if (i < 3)
    {
      extrinsics.translation[i] = fe_res.translation[i];
    }
  }
}//end of setExtrinData
}//end namespace
