#include <iostream>
#include <functional>
#include <iomanip>
#include <map>
#include <atomic>
#include <thread>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/OccupancyGrid.h>
#include <realsense_camera/StreamInfo.h>
#include <sensor_msgs/Imu.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <rs_slam_test/PoseMatrix.h>
#include <ros/time.h>
#include <std_msgs/String.h>

#include <rs_sdk.h>
#include <librealsense/slam/slam.h>
#include <sensor_msgs/CameraInfo.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <Eigen/Eigen>
#include "ParamTypes.h"
#include "2DMapCommon.h"
#include <opencv2/core/version.hpp>
#include <realsense_camera/GetIMUInfo.h>
#include <realsense_camera/GetFExtrinsics.h>
#include "SubscribeTopics.h"

namespace rs_slam_test
{
  class SNodeletSlam:public nodelet::Nodelet
  {
  public:
    SNodeletSlam();
    ~SNodeletSlam();
    ros::NodeHandle nh;
    std::shared_ptr< message_filters::Subscriber< sensor_msgs::CameraInfo > > sub_depthInfo,sub_fisheyeInfo;
    std::shared_ptr< message_filters::TimeSynchronizer< sensor_msgs::CameraInfo, sensor_msgs::CameraInfo > > info_TimeSynchronizer;
    ros::ServiceClient client_imu, client_fisheye;
    rs::core::video_module_interface::supported_module_config supported_config;
    rs::core::video_module_interface::actual_module_config actual_config;
    SubscribeTopics sub;

    void onInit()override;
  private:
    void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr & fisheyeCameraInfo, const sensor_msgs::CameraInfoConstPtr & depthCameraInfo);
    void setCalibrationData(const sensor_msgs::CameraInfoConstPtr & cameraInfoMsg, rs::core::intrinsics & cameraInfo);
    void setStreamConfigIntrin(rs::core::stream_type stream, std::map< rs::core::stream_type, rs::core::intrinsics > intrinsics);
    void setMotionData(realsense_camera::IMUInfo& imu_res, rs::core::motion_device_intrinsics& motion_intrin);
    void setMotionConfigIntrin(rs::core::motion_type motion, std::map< rs::core::motion_type, rs::core::motion_device_intrinsics > motion_intrinsics);
    void setExtrinData(realsense_camera::Extrinsics& fe_res, rs::core::extrinsics& extrinsics);
  };//end class
} 
