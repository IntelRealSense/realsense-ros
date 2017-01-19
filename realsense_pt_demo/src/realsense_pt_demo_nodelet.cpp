/******************************************************************************
   INTEL CORPORATION PROPRIETARY INFORMATION
   This software is supplied under the terms of a license agreement or nondisclosure
   agreement with Intel Corporation and may not be copied or disclosed except in
   accordance with the terms of that agreement
   Copyright(c) 2011-2015 Intel Corporation. All Rights Reserved.

Auther: Elisa_LU 
Email: lu.x.lu@intel.com
*******************************************************************************/
#include "realsense_pt_demo_nodelet.h"
#ifndef PERSON_TRACKING_DATA_FILES
#define PERSON_TRACKING_DATA_FILES "/usr/share/librealsense/pt/data/"
#endif

PLUGINLIB_EXPORT_CLASS(realsense_pt_demo::TNodeletPt, nodelet::Nodelet)

namespace realsense_pt_demo
{
  void TNodeletPt::onInit()
  {
    nh = getNodeHandle();
    image_transport::ImageTransport it(nh);
    pub_result = it.advertise("image_result", 1);
    setParams(config);
    NODELET_INFO("start nodelet param setted");
    struct stat stat_struct;
    if (stat(PERSON_TRACKING_DATA_FILES, &stat_struct) != 0)
    {
      NODELET_ERROR_STREAM("Failed to find person tracking data files at " <<  PERSON_TRACKING_DATA_FILES);
      exit(EXIT_FAILURE);
    }

    std::string person_tracking_data_files = PERSON_TRACKING_DATA_FILES;
    int size = person_tracking_data_files.length();
    wchar_t wc_person_tracking_data_files[size + 1];
    mbstowcs(wc_person_tracking_data_files, person_tracking_data_files.c_str(), size + 1);
    ptModule.reset(rs::person_tracking::person_tracking_video_module_factory::create_person_tracking_video_module(std::wstring(wc_person_tracking_data_files).c_str()));
    if (isallow)
    {
      ptModule->QueryConfiguration()->QueryTracking()->SetDetectionMode(Intel::RealSense::PersonTracking::PersonTrackingConfiguration::TrackingConfiguration::DetectionMode::MANUAL_CLOSE_RANGE);
    }
    //configure person tracking - enable requested features
    ConfigurePersonTracking(config, ptModule.get()->QueryConfiguration());
    // Subcribe to camera info
    std::string colorCameraInfoStream = "camera/color/camera_info";
    std::string depthCameraInfoStream = "camera/depth/camera_info";

    sub_color_camera_info = std::shared_ptr< message_filters::Subscriber< sensor_msgs::CameraInfo > >(new message_filters::Subscriber< sensor_msgs::CameraInfo >(nh, colorCameraInfoStream, 1));
    sub_depth_camera_info = std::shared_ptr< message_filters::Subscriber< sensor_msgs::CameraInfo > >(new message_filters::Subscriber< sensor_msgs::CameraInfo >(nh, depthCameraInfoStream, 1));
    //
    time_sync_info = std::shared_ptr< message_filters::TimeSynchronizer< sensor_msgs::CameraInfo, sensor_msgs::CameraInfo > >(new message_filters::TimeSynchronizer< sensor_msgs::CameraInfo, sensor_msgs::CameraInfo >(* sub_color_camera_info, * sub_depth_camera_info, 40));
    time_sync_info->registerCallback(boost::bind(& TNodeletPt::cameraInfoCallback, this, _1, _2));
    //for publish info
    publishers.onInit(nh, ptModule.get());
    //init server
    servers.onInit(nh, ptModule.get(), config);
    servers.autoImgRecog = isallow; 
    NODELET_INFO("end OnInit");
  }   //end OnInit   
  void TNodeletPt::setParams(PersonTrackingConfig & config)
  {
    ros::NodeHandle & nodeHandle = getPrivateNodeHandle();
    int streamingmode, colorresolution, depthresolution;
    std::string path = ros::package::getPath("realsense_pt_demo");
    NODELET_INFO("setParams");
    nodeHandle.param< bool >("recognitionEnabled", config.recognitionEnabled, false);
    nodeHandle.param< bool >("gesturesEnabled", config.gesturesEnabled, false);
    nodeHandle.param< bool >("trackingEnabled", config.trackingEnabled, false);
    nodeHandle.param< int >("trackingMode", config.trackingMode, 0);//0: start the MW in detection mode, 1: track first person in frame
    nodeHandle.param< bool >("loadDb", config.loadDb, false);
    nodeHandle.param< std::string >("dbPath", config.dbPath, path+"/db"); //in case of loadDb
    nodeHandle.param< bool >("skeletonEnabled", config.sceletonEnabled, false);
    nodeHandle.param< bool >("segmentationEnabled", config.segmentationEnabled, false);
    nodeHandle.param< bool >("headPositionEnabled", config.headPositionEnabled, false);
    nodeHandle.param< bool >("headBoundingBoxEnabled", config.headBoundingBoxEnabled, false);
    nodeHandle.param< bool >("landmarksEnabled", config.landmarksEnabled, false);
    nodeHandle.param< bool >("allowImgRegisration", isallow, false);
    nodeHandle.param< std::string >("device_version", device_version, "last");
  }
  void TNodeletPt::cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr & colorCameraInfo, const sensor_msgs::CameraInfoConstPtr & depthCameraInfo)
  {
    NODELET_INFO("callback");
    rs::core::intrinsics  depth_intrinsics, color_intrinsics;
    rs::core::extrinsics  extrinsics;
    //create projection object
    setCalibrationData(depthCameraInfo, depth_intrinsics);
    setCalibrationData(colorCameraInfo, color_intrinsics);
    for (int i = 0; i < 9; i++)
      extrinsics.rotation[i] = depthCameraInfo->R[i];
    if (device_version.compare("last") == 0)
    {
      extrinsics.translation[0] = depthCameraInfo->P[3];
      extrinsics.translation[1] = depthCameraInfo->P[7];
      extrinsics.translation[2] = depthCameraInfo->P[11];
    }
    else
    {
      extrinsics.translation[0] = depthCameraInfo->P[3] * 1000;
      extrinsics.translation[1] = depthCameraInfo->P[7] * 1000;
      extrinsics.translation[2] = depthCameraInfo->P[11] * 1000;
    }
    auto actualModuleConfig=getActualModule(nh, depth_intrinsics, color_intrinsics, extrinsics);
    actualModuleConfig.projection = rs::core::projection_interface::create_instance(&color_intrinsics, &depth_intrinsics, &extrinsics);
    // set the enabled module configuration
    if (ptModule->set_module_config(actualModuleConfig) != rs::core::status_no_error)
    {
      NODELET_ERROR_STREAM("error : failed to set the enabled module configuration"<<ptModule->set_module_config(actualModuleConfig));
      return ;
    }
    sub_color_camera_info->unsubscribe();
    sub_depth_camera_info->unsubscribe();
    starttime=ros::Time::now();
    seq=0;
    subscribeFrameMessages();
  }
  void TNodeletPt::setCalibrationData(const sensor_msgs::CameraInfoConstPtr & cameraInfoMsg, rs::core::intrinsics & cameraInfo)
  {
    cameraInfo.width = cameraInfoMsg->width;
    cameraInfo.height = cameraInfoMsg->height;
    cameraInfo.fx = cameraInfoMsg->K[0];
    cameraInfo.fy = cameraInfoMsg->K[4];
    cameraInfo.ppx = cameraInfoMsg->K[2];
    cameraInfo.ppy = cameraInfoMsg->K[5];
    for (int i = 0; i < 5; i++)
      cameraInfo.coeffs[i] = cameraInfoMsg->D[i];
    std::string distortion_model=cameraInfoMsg->distortion_model;
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
  rs::core::video_module_interface::actual_module_config TNodeletPt::getActualModule(ros::NodeHandle & nh, rs::core::intrinsics & depth_intrinsics, rs::core::intrinsics & color_intrinsics, rs::core::extrinsics & extrinsics)
  {
    rs::core::video_module_interface::actual_module_config actualModuleConfig = {};
    std::vector< rs::core::stream_type > possible_streams = {
                                                            rs::core::stream_type::depth,
                                                            rs::core::stream_type::color //person tracking uses only color & depth
                                                            };
    std::map< rs::core::stream_type, rs::core::intrinsics > intrinsics;
    intrinsics[rs::core::stream_type::depth] = depth_intrinsics;
    intrinsics[rs::core::stream_type::color] = color_intrinsics;
    std::map< rs::core::stream_type, std::string > frame_rates;
    frame_rates[rs::core::stream_type::depth] = "/depth_fps";
    frame_rates[rs::core::stream_type::color] = "/color_fps";
    for (auto &stream : possible_streams)
    {
      std::cout<<"width "<<intrinsics[stream].width<<std::endl;
      rs::core::video_module_interface::actual_image_stream_config & actualStreamConfig = actualModuleConfig[stream];
      actualStreamConfig.size.width = intrinsics[stream].width;
      actualStreamConfig.size.height = intrinsics[stream].height;
      if (!nh.getParam(frame_rates[stream], actualStreamConfig.frame_rate))
      {   
        NODELET_WARN_STREAM("Didn't get the param "<<frame_rates[stream]);
        actualStreamConfig.frame_rate=30;
      }
      std::cout<<actualStreamConfig.frame_rate<<std::endl;
      actualStreamConfig.intrinsics = intrinsics[stream];
      if(stream == rs::core::stream_type::depth){
        actualStreamConfig.extrinsics = extrinsics;
      }
      actualStreamConfig.is_enabled = true;
    }
    return actualModuleConfig;
  }
  void TNodeletPt::subscribeFrameMessages()
  {
    std::string depthImageStream =  "camera/depth/image_raw";
    std::string colorImageStream =  "camera/color/image_raw";
    NODELET_INFO_STREAM("Listening on " << depthImageStream);
    NODELET_INFO_STREAM("Listening on " << colorImageStream);
    sub_depth = std::shared_ptr< message_filters::Subscriber< sensor_msgs::Image > >(new message_filters::Subscriber< sensor_msgs::Image >(nh, depthImageStream, 1));
    sub_color = std::shared_ptr< message_filters::Subscriber< sensor_msgs::Image > >(new message_filters::Subscriber< sensor_msgs::Image >(nh, colorImageStream, 1));
    time_sync = std::shared_ptr< message_filters::TimeSynchronizer< sensor_msgs::Image, sensor_msgs::Image > >(new message_filters::TimeSynchronizer< sensor_msgs::Image, sensor_msgs::Image >(* sub_depth, * sub_color, 40));
    time_sync->registerCallback(boost::bind(& TNodeletPt::personTrackingCallback, this, _1, _2));
  }
  void TNodeletPt::personTrackingCallback(const sensor_msgs::ImageConstPtr & depthImageMsg, const sensor_msgs::ImageConstPtr & colorImageMsg)
  {
    NODELET_DEBUG("image msgs");
    if (!processing_mutex.try_lock()) return;
    NODELET_DEBUG("processFrame started");
    TrackingRenderer trackingRenderer;
    PersonTracker personTracker(trackingRenderer, ptModule.get()->QueryConfiguration());
    //get frame	
    rs::core::correlated_sample_set sampleSet;
    GetNextFrame(depthImageMsg,colorImageMsg,sampleSet);
    seq++;
    {
      std::lock_guard<std::mutex> lock(personTracker.GetPersonTrackingMutex());
     //process frame
      if (ptModule->process_sample_set(sampleSet) != rs::core::status_no_error)
      {
        NODELET_ERROR("error : failed to process sample");
      }
    }
    cv::Mat renderImage;
    try
    {
      renderImage = cv_bridge::toCvShare(colorImageMsg, sensor_msgs::image_encodings::RGB8)->image;
    }
    catch (cv_bridge::Exception & e)
    {
      NODELET_ERROR("return false colorImage");
      return;
    }
    cv::cvtColor(renderImage,renderImage, CV_RGB2BGR);
    SetTracking(config, ptModule->QueryOutput());
    personTracker.DrawFrame(renderImage, ptModule->QueryOutput());
    //pub image
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::BGR8, renderImage).toImageMsg();
    msg->header.frame_id = "result image";
    msg->header.stamp = ros::Time::now();      // Publish timestamp to synchronize frames.
    pub_result.publish(msg);
    publishers.publishOutput(*(ptModule->QueryOutput()));
    processing_mutex.unlock();
  }
  void TNodeletPt::GetNextFrame(const sensor_msgs::ImageConstPtr & depthImageMsg, const sensor_msgs::ImageConstPtr & colorImageMsg, rs::core::correlated_sample_set & sample_set)
  {
    std::map< rs::core::stream_type, sensor_msgs::ImageConstPtr > images;
    images[rs::core::stream_type::depth] = depthImageMsg;
    images[rs::core::stream_type::color] = colorImageMsg;
    std::map< rs::core::stream_type, rs::format > formats;
    formats[rs::core::stream_type::depth]=rs::format::z16;
    formats[rs::core::stream_type::color]=rs::format::rgb8;
    for (auto & stream : {rs::core::stream_type::color, rs::core::stream_type::depth})
    {
      int width = images[stream]->width;
      int height = images[stream]->height;
      rs::core::image_info info = {
                                   width,
                                   height,
                                   rs::utils::convert_pixel_format(formats[stream]),
                                   get_pixel_size(formats[stream]) * width
                                  };
      ros::Time now = ros::Time::now();
      ros::Duration d = now-starttime;
      double stamp = (double)d.toNSec() / 1000000000;
      sample_set[stream] = rs::core::image_interface::create_instance_from_raw_data(
                                                            &info,
                                                            images[stream]->data.data(),
                                                            stream,
                                                            rs::core::image_interface::flag::any,
                                                            0,//stamp,
                                                            0,// seq,
                                                            rs::core::timestamp_domain::microcontroller);
    }
  }//end GetFrame
  int8_t TNodeletPt::get_pixel_size(rs::format format)
  {
    switch (format)
    {
      case rs::format::any:return 0;
      case rs::format::z16:return 2;
      case rs::format::disparity16:return 2;
      case rs::format::xyz32f:return 4;
      case rs::format::yuyv:return 2;
      case rs::format::rgb8:return 3;
      case rs::format::bgr8:return 3;
      case rs::format::rgba8:return 4;
      case rs::format::bgra8:return 4;
      case rs::format::y8:return 1;
      case rs::format::y16:return 2;
      case rs::format::raw8:return 1;
      case rs::format::raw10:return 0;//not supported
      case rs::format::raw16:return 2;
    }
  }//end get_pixel_size
}
