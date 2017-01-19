#include <vector>
#include <cstring>
#include <sys/types.h>
#include <sys/stat.h>
#include <string>
#include <fstream>
#include <iostream>
#include <unistd.h>
#include <signal.h>
#include <stdio.h>
#include <malloc.h>
#include <stdlib.h>

#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include "ros/ros.h"
#include "ros/package.h"
#include "ros/time.h"
#include <std_msgs/String.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include "opencv2/imgproc/types_c.h"
#include "opencv2/opencv.hpp"
#include <librealsense/rs.hpp>
#include "rs_sdk.h"
#include "person_tracking_video_module_factory.h"
#include "TrackingRenderer.h"
#include "PersonTracker.h"
#include "PersonTrackingHelper.h"
#include "PersonTrackingPublisher.h"
#include "PersonTrackingServer.h"

namespace realsense_pt_demo
{
  class TNodeletPt:public nodelet::Nodelet
  {
  public:
    TNodeletPt(){}
    ros::NodeHandle nh;
    void onInit()override;

  private:
    bool isallow;
    std::string device_version;
    std::unique_ptr< rs::person_tracking::person_tracking_video_module_interface > ptModule;
    std::shared_ptr< message_filters::Subscriber< sensor_msgs::CameraInfo > > sub_color_camera_info;
    std::shared_ptr< message_filters::Subscriber< sensor_msgs::CameraInfo > > sub_depth_camera_info;
    std::shared_ptr< message_filters::TimeSynchronizer< sensor_msgs::CameraInfo, sensor_msgs::CameraInfo > > time_sync_info;
    std::shared_ptr< message_filters::Subscriber< sensor_msgs::Image > > sub_depth;
    std::shared_ptr< message_filters::Subscriber< sensor_msgs::Image > > sub_color;
    std::shared_ptr< message_filters::TimeSynchronizer< sensor_msgs::Image, sensor_msgs::Image > > time_sync;
    image_transport::Publisher pub_result;
    std::mutex processing_mutex;
    PersonTrackingPublisher publishers;
    PersonTrackingServer servers;
    ros::Time starttime;
    uint64_t seq;
    //set ros params
    void setParams(PersonTrackingConfig & config);
    void setCalibrationData(const sensor_msgs::CameraInfoConstPtr & cameraInfoMsg, rs::core::intrinsics & cameraInfo);
    void GetNextFrame(const sensor_msgs::ImageConstPtr & depthImageMsg, const sensor_msgs::ImageConstPtr & colorImageMsg, rs::core::correlated_sample_set & sample_set);
    int8_t get_pixel_size(rs::format format);
    void personTrackingCallback(const sensor_msgs::ImageConstPtr & depthImageMsg, const sensor_msgs::ImageConstPtr & colorImageMsg);
    void subscribeFrameMessages();
    void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr & colorCameraInfo, const sensor_msgs::CameraInfoConstPtr & depthCameraInfo);
    rs::core::video_module_interface::actual_module_config getActualModule(ros::NodeHandle & nh, rs::core::intrinsics & depth_intrinsics, rs::core::intrinsics & color_intrinsics, rs::core::extrinsics & extrinsics);
    PersonTrackingConfig config;
  };

}

