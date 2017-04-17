// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved

#include "realsense_ros_person_nodelet.h"

PLUGINLIB_EXPORT_CLASS(realsense_ros_person::TNodeletPt, nodelet::Nodelet)

namespace realsense_ros_person
{
const std::string TNodeletPt::DEFAULT_PUBLISHER = "defaultPublisher";

TNodeletPt::TNodeletPt() :
  m_ros2realsense(new realsense_ros_person::Ros2RealSenseSdkConverter()),
  m_isTestMode(false)
{
}

void TNodeletPt::onInit()
{
  setParams(config);
  NODELET_INFO("start nodelet param setted");
  ROS_INFO_STREAM("enable" << config.recognitionEnabled);

  std::string path = ros::package::getPath("realsense_ros_person");

  /***************create and configure person tracker video module**********************/
  ptModule.reset(rs::person_tracking::person_tracking_video_module_factory::create_person_tracking_video_module());

  //configure person tracking - enable requested features
  ConfigurePersonTracking(config, ptModule.get()->QueryConfiguration());
  //configure person tracking - load recognition database if needed
  bool loadDb = false;
  getPrivateNodeHandle().param<bool>("loadDb", loadDb, false);
  if (loadDb)
  {
    std::string dbPath;
    getPrivateNodeHandle().param<std::string>("dbPath", dbPath, path + "/db"); //in case of loadDb
    LoadDatabase(dbPath, ptModule->QueryConfiguration());
  }

  // Subcribe to camera info
  std::string colorCameraInfoStream = "camera/color/camera_info";
  std::string depthCameraInfoStream = "camera/depth/camera_info";

  mColorCameraInfoSubscriber = std::shared_ptr<message_filters::Subscriber<sensor_msgs::CameraInfo>>(new message_filters::Subscriber<sensor_msgs::CameraInfo>(nh, colorCameraInfoStream, 1));
  mDepthCameraInfoSubscriber = std::shared_ptr<message_filters::Subscriber<sensor_msgs::CameraInfo>>(new message_filters::Subscriber<sensor_msgs::CameraInfo>(nh, depthCameraInfoStream, 1));

  mTimeSynchronizer_info = std::shared_ptr<message_filters::TimeSynchronizer<sensor_msgs::CameraInfo, sensor_msgs::CameraInfo>>(new message_filters::TimeSynchronizer<sensor_msgs::CameraInfo, sensor_msgs::CameraInfo>(*mColorCameraInfoSubscriber, *mDepthCameraInfoSubscriber, 40));
  mTimeSynchronizer_info->registerCallback(boost::bind(&TNodeletPt::cameraInfoCallback, this, _1, _2));

  getPrivateNodeHandle().param<bool>("isTestMode", m_isTestMode, false);
  if (m_isTestMode)
  {
    NODELET_INFO("test mode publish person tracking test topic(includes color image)");
  }
  //configure and init publisher
  std::string publisherType;
  getPrivateNodeHandle().param<std::string>("publisherType", publisherType, "123");
  if (publisherType.compare(DEFAULT_PUBLISHER) == 0)
  {
    mPublisher.reset(new PersonTrackingDefaultPublisher());
    NODELET_INFO("using default publisher");
  }
  else
  {
    NODELET_ERROR("unsupported publisher: %s", publisherType.c_str());
    throw std::runtime_error("unsupported publisher");
  }
  mPublisher->onInit(nh);
  //init server
  mServer.onInit(nh, ptModule.get(), config);

  NODELET_INFO("end OnInit");

}   //end OnInit

void TNodeletPt::setParams(PersonTrackingConfig& config)
{
  ros::NodeHandle & nodeHandle = getPrivateNodeHandle();
  std::string path = ros::package::getPath("realsense_ros_person");
  NODELET_INFO("setParams");
  nodeHandle.param<bool>("recognitionEnabled", config.recognitionEnabled, false);
  nodeHandle.param<bool>("pointingGestureEnabled", config.pointingGestureEnabled, false);
  nodeHandle.param<bool>("waveGestureEnabled", config.waveGestureEnabled, false);

  nodeHandle.param<bool>("trackingEnabled", config.trackingEnabled, true);
  nodeHandle.param<bool>("skeletonEnabled", config.skeletonEnabled, false);

  nodeHandle.param<bool>("headPoseEnabled", config.headPoseEnabled, false);
  nodeHandle.param<bool>("headBoundingBoxEnabled", config.headBoundingBoxEnabled, false);
  nodeHandle.param<bool>("landmarksEnabled", config.landmarksEnabled, false);
}
///////////////////////////////////////////////////////////////////////////////
void TNodeletPt::cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& colorCameraInfo, const sensor_msgs::CameraInfoConstPtr& depthCameraInfo)
{
  NODELET_INFO("cameraInfoCallback");
  rs::core::video_module_interface::actual_module_config config = m_ros2realsense->CreateSdkModuleConfig(colorCameraInfo, depthCameraInfo);
  if (ptModule == nullptr || config.projection == nullptr)
  {
    NODELET_ERROR_STREAM("person tracking module or projection is null");
  }
  if (ptModule->set_module_config(config) != rs::core::status_no_error)
  {
    NODELET_ERROR_STREAM("error : failed to set the enabled module configuration");
    return;
  }
  mColorCameraInfoSubscriber->unsubscribe();
  mDepthCameraInfoSubscriber->unsubscribe();
  seq = 0;
  subscribeFrameMessages();
}

void TNodeletPt::subscribeFrameMessages()
{
  std::string depthImageStream =  "camera/depth/image_raw";
  std::string colorImageStream =  "camera/color/image_raw";
  NODELET_INFO_STREAM("Listening on " << depthImageStream);
  NODELET_INFO_STREAM("Listening on " << colorImageStream);
  // Subscribe to color, point cloud and depth using synchronization filter
  mDepthSubscriber = std::shared_ptr<message_filters::Subscriber<sensor_msgs::Image>>
                     (new message_filters::Subscriber<sensor_msgs::Image>(nh, depthImageStream, 1));
  mColorSubscriber = std::shared_ptr<message_filters::Subscriber<sensor_msgs::Image>>
                     (new message_filters::Subscriber<sensor_msgs::Image>(nh, colorImageStream, 1));

  mTimeSynchronizer = std::shared_ptr<message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image>>
                      (new message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image>(*mDepthSubscriber, *mColorSubscriber, 40));
  mTimeSynchronizer->registerCallback(boost::bind(&TNodeletPt::personTrackingCallback, this, _1, _2));

}

void TNodeletPt::personTrackingCallback(const sensor_msgs::ImageConstPtr& depthImageMsg, const sensor_msgs::ImageConstPtr& colorImageMsg)
{
  rs::core::correlated_sample_set sampleSet = m_ros2realsense->CreateSdkSampleSet(colorImageMsg, depthImageMsg);
  seq++;
  {
    //prevent race condition on asynchronous  requests(change configuration/recognize) and processing
    std::lock_guard<std::mutex> guard(mProcessingMutex);
    if (ptModule->process_sample_set(sampleSet) != rs::core::status_no_error)
    {
      NODELET_ERROR("error : failed to process sample");
      return;
    }
    mPublisher->publishOutput(*ptModule->QueryConfiguration(), *ptModule->QueryOutput());
    if (m_isTestMode)
    {
      //publish test output(includes color image) only at test mode
      mPublisher->publishTestOutput(*ptModule->QueryConfiguration(), *ptModule->QueryOutput(), colorImageMsg);
    }
  }
  m_ros2realsense->ReleaseSampleSet(sampleSet);
}
}
