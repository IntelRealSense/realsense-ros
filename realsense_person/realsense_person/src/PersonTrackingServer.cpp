#include "PersonTrackingServer.h"
#include <vector>
#include <fstream>

PersonTrackingServer::PersonTrackingServer(){}

void PersonTrackingServer::onInit(ros::NodeHandle& nodeHandle, rs::person_tracking::person_tracking_video_module_interface * personTracking, PersonTrackingConfig & config)
{
  mPersonTracking = personTracking;
  mConfig = config;
  mTrackingConfigService = nodeHandle.advertiseService("person_tracking/tracking_config", & PersonTrackingServer::trackingConfigRequestCallback, this);
  mRecognitionRequestService = nodeHandle.advertiseService("person_tracking/recognition_request", & PersonTrackingServer::recognitionRequestCallback, this);
  mTrackingRequestService = nodeHandle.advertiseService("person_tracking/tracking_request", & PersonTrackingServer::trackingRequestCallback, this);
  mSaveRecognitionDbService = nodeHandle.advertiseService("person_tracking/save_recognition", & PersonTrackingServer::saveRecognitionDbCallback, this);
  mLoadRecognitionDbService = nodeHandle.advertiseService("person_tracking/load_recognition", & PersonTrackingServer::loadRecognitionDbCallback, this);
  mRecognitionImgRequestService = nodeHandle.advertiseService("person_tracking/recognition_image_request",& PersonTrackingServer::recognitionImgRequestCallback, this);
}

bool PersonTrackingServer::trackingConfigRequestCallback(realsense_srvs::TrackingConfig::Request & request, realsense_srvs::TrackingConfig::Response & response)
{
  mConfig.segmentationEnabled = request.enableSegmentation;
  mConfig.recognitionEnabled = request.enableRecognition;
  mConfig.sceletonEnabled = request.enableSkeleton;
  mConfig.gesturesEnabled = request.enableGestures;
  ConfigurePersonTracking(mConfig, mPersonTracking->QueryConfiguration());
  response.status = true;
  return true;
}
bool PersonTrackingServer::recognitionRequestCallback(realsense_srvs::RecognitionRequest::Request & request, realsense_srvs::RecognitionRequest::Response & response)
{
  if (!mPersonTracking->QueryConfiguration()->QueryRecognition()->IsEnabled())
  {
    ROS_INFO("Recognition is not enabled");
    response.status = false;
    return true;
  }

  ROS_INFO_STREAM("Received recognition request for person: " << request.personId);
  PXCPersonTrackingData* trackingData = mPersonTracking->QueryOutput();
  PXCPersonTrackingData::Person* personData = trackingData->QueryPersonDataById(request.personId);
  if (!personData)
  {
    ROS_ERROR_STREAM("Couldn't find recognition request target");
    response.status = false;
    return true;
  }
  ROS_INFO_STREAM("Found recognition request target");
  PXCPersonTrackingData::PersonRecognition* recognition = personData->QueryRecognition();
  int32_t outputRecognitionId, outputTrackingId, outDescriptorId;
  ROS_INFO("User registration");
  auto status = recognition->RegisterUser(& outputRecognitionId, & outputTrackingId, & outDescriptorId);
  response.status = false;
  switch (status)
  {
    case PXCPersonTrackingData::PersonRecognition::RegistrationStatus::RegistrationSuccessful:
         ROS_INFO_STREAM("Registered person: " << outputRecognitionId<<" TrackingId: "<< outputTrackingId<<" DescriptorId: "<< outDescriptorId);
         response.status = true;
         break;
    case PXCPersonTrackingData::PersonRecognition::RegistrationStatus::RegistrationFailedAlreadyRegistered:
         ROS_INFO_STREAM("Person already registered, remove person: " << outputRecognitionId<<" TrackingId: "<< outputTrackingId<<" DescriptorId: "<< outDescriptorId);
         mPersonTracking->QueryConfiguration()->QueryRecognition()->QueryDatabase()->RemoveEntry(outputRecognitionId);
         response.status = true;
         break;
    case PXCPersonTrackingData::PersonRecognition::RegistrationStatus::RegistrationFailed:
         ROS_INFO_STREAM("Registration failed: person: "<< outputRecognitionId<<" TrackingId: "<< outputTrackingId<<" DescriptorId: "<< outDescriptorId);
         break;
    case PXCPersonTrackingData::PersonRecognition::RegistrationStatus::RegistrationFailedFaceNotDetected:
         ROS_INFO_STREAM("Registration failed: face not detected, person: "<< outputRecognitionId<<" TrackingId: "<< outputTrackingId<<" DescriptorId: "<< outDescriptorId);
         break;	     
    case PXCPersonTrackingData::PersonRecognition::RegistrationStatus::RegistrationFailedFaceNotClear:
         ROS_INFO_STREAM("Registration failed: face not clear, person: "<< outputRecognitionId<<" TrackingId: "<< outputTrackingId<<" DescriptorId: "<< outDescriptorId);
         break;
    case PXCPersonTrackingData::PersonRecognition::RegistrationStatus::RegistrationFailedPersonTooFar:
         ROS_INFO_STREAM("Registration failed: person too far, person: "<< outputRecognitionId<<" TrackingId: "<< outputTrackingId<<" DescriptorId: "<< outDescriptorId);
         break;
    case PXCPersonTrackingData::PersonRecognition::RegistrationStatus::RegistrationFailedPersonTooClose:
         ROS_INFO_STREAM("Registration failed: person to close, person: "<< outputRecognitionId<<" TrackingId: "<< outputTrackingId<<" DescriptorId: "<< outDescriptorId);
         break;
  }
  return true;
}

bool PersonTrackingServer::trackingRequestCallback(realsense_srvs::TrackingRequest::Request & request, realsense_srvs::TrackingRequest::Response & response)
{
  ROS_INFO_STREAM("Received tracking request for person: " << request.personId);
  PXCPersonTrackingData* trackingData = mPersonTracking->QueryOutput();
  PXCPersonTrackingData::Person* personData = trackingData->QueryPersonDataById(request.personId);
  if (!personData)
  {
    ROS_ERROR_STREAM("Couldn't find tracking request target");
    response.status = false;
    return true;
  }
  ROS_INFO_STREAM("Found tracking request target");
  PXCPersonTrackingData::TrackingState trackingState = trackingData->GetTrackingState();
  if (trackingState == PXCPersonTrackingData::TRACKING_STATE_DETECTING)
  {
    ROS_INFO_STREAM("start tracking");
    trackingData->StartTracking(request.personId);
  }
  else
  {
    ROS_INFO_STREAM("stop tracking");
    trackingData->StopTracking(request.personId);
  }
  response.status = true;
  return true;
}

bool PersonTrackingServer::saveRecognitionDbCallback(realsense_srvs::SaveRecognitionDB::Request & request, realsense_srvs::SaveRecognitionDB::Response & response)
{
  std::string dbPath = request.saveToPath;
  ROS_INFO_STREAM("Save database to: " << dbPath);
  try
  {
    SaveDatabase(dbPath, mPersonTracking->QueryConfiguration());
    response.status = true;
  }
  catch(const std::runtime_error & error)
  {
    ROS_ERROR_STREAM("Failed to save database to: " << dbPath);
    response.status=false;
  }
  return true;
}

bool PersonTrackingServer::loadRecognitionDbCallback(realsense_srvs::LoadRecognitionDB::Request & request, realsense_srvs::LoadRecognitionDB::Response & response)
{
  std::string dbPath = request.loadFromPath;
  ROS_INFO_STREAM("Loading database from: " << dbPath);
  try
  {
    LoadDatabase(dbPath, mPersonTracking->QueryConfiguration());
    response.status = true;
  }
  catch(const std::runtime_error & error)
  {
    ROS_ERROR_STREAM("Failed to load database from: "<< dbPath);
    response.status=false;
  }
  return true;
}

void PersonTrackingServer::CreateDepthImage(rs::core::correlated_sample_set & sampleSet, const void * depthData, int width, int height)
{
  rs::core::stream_type stream = rs::core::stream_type::depth;
  rs::core::pixel_format pixel_format = rs::core::pixel_format::z16;
  rs::core::image_info info = {
                               width,
                               height,
                               pixel_format,
                               2 * width
                              };
  sampleSet[stream] = rs::core::image_interface::create_instance_from_raw_data(
                                                 &info,
                                                 rs::core::image_interface::image_data_with_data_releaser(depthData, nullptr),
                                                 stream,
                                                 rs::core::image_interface::flag::any,
                                                 0,
                                                 0,
                                                 rs::core::timestamp_domain::microcontroller);
}

void PersonTrackingServer::CreateColorImage(rs::core::correlated_sample_set & sampleSet, const void * colorData, int width, int height)
{
  rs::core::stream_type stream = rs::core::stream_type::color;
  rs::core::pixel_format pixel_format = rs::core::pixel_format::rgb8;
  int channels = (pixel_format == rs::core::pixel_format::rgb8 || pixel_format == rs::core::pixel_format::bgr8) ? 3 : 4;
  rs::core::image_info info = {
                               width,
                               height,
                               pixel_format,
                               channels * width
                              };
  sampleSet[stream] = rs::core::image_interface::create_instance_from_raw_data(
                                                 &info,
                                                 rs::core::image_interface::image_data_with_data_releaser(colorData, nullptr),
                                                 stream,
                                                 rs::core::image_interface::flag::any,
                                                 0,
                                                 0,
                                                 rs::core::timestamp_domain::microcontroller);
}


bool PersonTrackingServer::recognitionImgRequestCallback(realsense_srvs::RecognitionImgRequest::Request & request, realsense_srvs::RecognitionImgRequest::Response & response)
{
  if (!autoImgRecog)
  {
    ROS_WARN("Regisration by image is not allowed");
    response.status = false;
    return true;
  }	
  if (!mPersonTracking->QueryConfiguration()->QueryRecognition()->IsEnabled())
  {
    ROS_WARN("Recognition is not enabled");
    response.status = false;
    return true;
  }
  ROS_INFO_STREAM("Received recognition request by image");
  // read image and create images
  std::string imagePath= request.personImgPath;
  ROS_INFO_STREAM("Reading image: " << imagePath);
  cv::Mat colorImage = cv::imread(imagePath, CV_LOAD_IMAGE_COLOR);   // Read the file
  if (!colorImage.data)
  {
    ROS_WARN_STREAM("Could not open or find the image");
    response.status = false;
    return true;
  }
  ROS_INFO("Image read successfully");
  cv::Mat depthImage(320, 240, CV_16UC1);
  rs::core::correlated_sample_set sampleSet = {};
  CreateColorImage(sampleSet, colorImage.data, colorImage.cols, colorImage.rows);
  CreateDepthImage(sampleSet, depthImage.data, depthImage.cols, depthImage.rows);
  // process frame
  ROS_INFO("Process images...");
  if (mPersonTracking->process_sample_set(sampleSet) != rs::core::status_no_error)
  {
    ROS_ERROR("error : failed to process sample");
    response.status = false;
    return true;
  }
  // verify detected person
  if (mPersonTracking->QueryOutput()->QueryNumberOfPeople() == 0)
  {
    ROS_ERROR("Failed to detect person in frame");
    response.status = false;
    return true;
  }
  
  // register person
  ROS_INFO("Register pesron...");
  int32_t outputRecognitionId, outputTrackingId, outDescriptorId;
  auto result = mPersonTracking->QueryOutput()->QueryPersonData(PXCPersonTrackingData::AccessOrderType::ACCESS_ORDER_BY_INDEX, 0)->QueryRecognition()->RegisterUser(& outputRecognitionId, & outputTrackingId, & outDescriptorId);
 
  response.status = true;
  switch (result)
  {
    case PXCPersonTrackingData::PersonRecognition::RegistrationStatus::RegistrationSuccessful:
         ROS_INFO_STREAM("Registered person: " << outputRecognitionId<<" TrackingId: "<< outputTrackingId<<" DescriptorId: "<< outDescriptorId);
         response.status = true;
         break;
    case PXCPersonTrackingData::PersonRecognition::RegistrationStatus::RegistrationFailedAlreadyRegistered:
         ROS_INFO_STREAM("Person already registered, remove person: " << outputRecognitionId<<" TrackingId: "<< outputTrackingId<<" DescriptorId: "<< outDescriptorId);
         mPersonTracking->QueryConfiguration()->QueryRecognition()->QueryDatabase()->RemoveEntry(outputRecognitionId);
         response.status = true;
         break;
    case PXCPersonTrackingData::PersonRecognition::RegistrationStatus::RegistrationFailed:
         ROS_INFO_STREAM("Registration failed: person: "<< outputRecognitionId<<" TrackingId: "<< outputTrackingId<<" DescriptorId: "<< outDescriptorId);
         break;
    case PXCPersonTrackingData::PersonRecognition::RegistrationStatus::RegistrationFailedFaceNotDetected:
         ROS_INFO_STREAM("Registration failed: face not detected, person: "<< outputRecognitionId<<" TrackingId: "<< outputTrackingId<<" DescriptorId: "<< outDescriptorId);
         break;
    case PXCPersonTrackingData::PersonRecognition::RegistrationStatus::RegistrationFailedFaceNotClear:
         ROS_INFO_STREAM("Registration failed: face not clear, person: "<< outputRecognitionId<<" TrackingId: "<< outputTrackingId<<" DescriptorId: "<< outDescriptorId);
         break;
    case PXCPersonTrackingData::PersonRecognition::RegistrationStatus::RegistrationFailedPersonTooFar:
         ROS_INFO_STREAM("Registration failed: person too far, person: "<< outputRecognitionId<<" TrackingId: "<< outputTrackingId<<" DescriptorId: "<< outDescriptorId);
         break;
    case PXCPersonTrackingData::PersonRecognition::RegistrationStatus::RegistrationFailedPersonTooClose:
         ROS_INFO_STREAM("Registration failed: person to close, person: "<< outputRecognitionId<<" TrackingId: "<< outputTrackingId<<" DescriptorId: "<< outDescriptorId);
         break;
  }

  return true;
}
