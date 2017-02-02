/******************************************************************************
 Copyright (c) 2017, Intel Corporation
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:

 1. Redistributions of source code must retain the above copyright notice, this
 list of conditions and the following disclaimer.

 2. Redistributions in binary form must reproduce the above copyright notice,
 this list of conditions and the following disclaimer in the documentation
 and/or other materials provided with the distribution.

 3. Neither the name of the copyright holder nor the names of its contributors
 may be used to endorse or promote products derived from this software without
 specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

#include <realsense_person/person_nodelet.h>
#include <chrono>

PLUGINLIB_EXPORT_CLASS(realsense_person::PersonNodelet, nodelet::Nodelet)

namespace realsense_person
{
  /*
   * Nodelet constructor.
   */
  PersonNodelet::PersonNodelet()
  {
    pt_video_module_.reset(PersonModuleInterface::person_tracking_video_module_factory::
        create_person_tracking_video_module(PERSON_MODULE_DATA_PATH));
    projection_interface_ = nullptr;
    caminfo_disconnected_ = false;
    tracking_id_ = -1;
    current_time_ = 0.0;
    last_detection_time_ = 0.0;
    last_tracking_time_ = 0.0;
    publish_detection_ = false;
    publish_detection_image_ = false;
    publish_tracking_ = false;
    publish_tracking_image_ = false;
  }

  /*
   * Nodelet destructor.
   */
  PersonNodelet::~PersonNodelet()
  {
    if (projection_interface_)
    {
      projection_interface_->release();
    }
  }

  /*
   * Initialize Nodelet.
   */
  void PersonNodelet::onInit()
  {
    getStaticParameters();
    startDynamicReconfServer();
    definePubSub();
    subscribeToCamInfoTopics();
    advertiseServices();
  }

  /*
   * Get static Parameters.
   */
  void PersonNodelet::getStaticParameters()
  {
    nodelet_name_ = getName();
    nh_ = getNodeHandle();
    pnh_ = getPrivateNodeHandle();
    pnh_.param("subscribe_rate", subscribe_rate_, SUBSCRIBE_RATE);
  }

  /*
   * Start the Dynamic Reconfigure Server.
   */
  void PersonNodelet::startDynamicReconfServer()
  {
    dynamic_reconf_server_.reset(new dynamic_reconfigure::Server<realsense_person::person_paramsConfig>(pnh_));
    dynamic_reconf_server_->setCallback(boost::bind(&PersonNodelet::dynamicReconfCallback, this, _1, _2));
  }

  /*
   * Get dynamic Parameters.
   */
  void PersonNodelet::dynamicReconfCallback(realsense_person::person_paramsConfig &config, uint32_t level)
  {
    ROS_INFO_STREAM(nodelet_name_ << " - Setting detection rate to " << config.detection_rate);
    detection_rate_ = config.detection_rate;

    ROS_INFO_STREAM(nodelet_name_ << " - Setting tracking rate to " << config.tracking_rate);
    tracking_rate_ = config.tracking_rate;

    if ((config.enable_recognition) &&
        (!pt_video_module_->QueryConfiguration()->QueryRecognition()->IsEnabled()))
    {
      ROS_INFO_STREAM(nodelet_name_ << " - Enabling recognition");
      pt_video_module_->QueryConfiguration()->QueryRecognition()->Enable();
    }
    else if ((!config.enable_recognition) &&
        (pt_video_module_->QueryConfiguration()->QueryRecognition()->IsEnabled()))
    {
      ROS_INFO_STREAM(nodelet_name_ << " - Disabling recognition");
      pt_video_module_->QueryConfiguration()->QueryRecognition()->Disable();
    }

/*  TODO: Temporarily removed in MW Beta3 version
    if ((config.enable_orientation) &&
        (!pt_video_module_->QueryConfiguration()->QueryTracking()->IsPersonOrientationEnabled()))
    {
      ROS_INFO_STREAM(nodelet_name_ << " - Enabling orientation");
      pt_video_module_->QueryConfiguration()->QueryTracking()->EnablePersonOrientation();
    }
    else if ((!config.enable_orientation) &&
        (pt_video_module_->QueryConfiguration()->QueryTracking()->IsPersonOrientationEnabled()))
    {
      ROS_INFO_STREAM(nodelet_name_ << " - Disabling orientation");
      pt_video_module_->QueryConfiguration()->QueryTracking()->DisablePersonOrientation();
    }

    if ((config.enable_head_pose) &&
        (!pt_video_module_->QueryConfiguration()->QueryTracking()->IsHeadPoseEnabled()))
    {
      ROS_INFO_STREAM(nodelet_name_ << " - Enabling head pose");
      pt_video_module_->QueryConfiguration()->QueryTracking()->EnableHeadPose();
    }
    else if ((!config.enable_head_pose) &&
        (pt_video_module_->QueryConfiguration()->QueryTracking()->IsHeadPoseEnabled()))
    {
      ROS_INFO_STREAM(nodelet_name_ << " - Disabling head pose");
      pt_video_module_->QueryConfiguration()->QueryTracking()->DisableHeadPose();
    }
*/

    if ((config.enable_head_bounding_box) &&
        (!pt_video_module_->QueryConfiguration()->QueryTracking()->IsHeadBoundingBoxEnabled()))
    {
      ROS_INFO_STREAM(nodelet_name_ << " - Enabling head bounding box");
      pt_video_module_->QueryConfiguration()->QueryTracking()->EnableHeadBoundingBox();
    }
    else if ((!config.enable_head_bounding_box) &&
        (pt_video_module_->QueryConfiguration()->QueryTracking()->IsHeadBoundingBoxEnabled()))
    {
      ROS_INFO_STREAM(nodelet_name_ << " - Disabling head bounding box");
      pt_video_module_->QueryConfiguration()->QueryTracking()->DisableHeadBoundingBox();
    }

    if ((config.enable_face_landmarks) &&
        (!pt_video_module_->QueryConfiguration()->QueryFace()->IsFaceLandmarksEnabled()))
    {
      ROS_INFO_STREAM(nodelet_name_ << " - Enabling face landmarks");
      pt_video_module_->QueryConfiguration()->QueryFace()->EnableFaceLandmarks();
    }
    else if ((!config.enable_face_landmarks) &&
        (pt_video_module_->QueryConfiguration()->QueryFace()->IsFaceLandmarksEnabled()))
    {
      ROS_INFO_STREAM(nodelet_name_ << " - Disabling face landmarks");
      pt_video_module_->QueryConfiguration()->QueryFace()->DisableFaceLandmarks();
    }

    if ((config.enable_gestures) &&
        (!pt_video_module_->QueryConfiguration()->QueryGestures()->IsEnabled()))
    {
      ROS_INFO_STREAM(nodelet_name_ << " - Enabling gestures");
      pt_video_module_->QueryConfiguration()->QueryGestures()->Enable();
      pt_video_module_->QueryConfiguration()->QueryGestures()->EnableAllGestures();
    }
    else if ((!config.enable_gestures) &&
        (pt_video_module_->QueryConfiguration()->QueryGestures()->IsEnabled()))
    {
      ROS_INFO_STREAM(nodelet_name_ << " - Disabling gestures");
      pt_video_module_->QueryConfiguration()->QueryGestures()->Disable();
      pt_video_module_->QueryConfiguration()->QueryGestures()->DisableAllGestures();
    }

    if ((config.enable_skeleton_joints) &&
        (!pt_video_module_->QueryConfiguration()->QuerySkeletonJoints()->IsEnabled()))
    {
      ROS_INFO_STREAM(nodelet_name_ << " - Enabling skeleton joints");
      pt_video_module_->QueryConfiguration()->QuerySkeletonJoints()->Enable();
    }
    else if ((!config.enable_skeleton_joints) &&
        (pt_video_module_->QueryConfiguration()->QuerySkeletonJoints()->IsEnabled()))
    {
      ROS_INFO_STREAM(nodelet_name_ << " - Disabling skeleton joints");
      pt_video_module_->QueryConfiguration()->QuerySkeletonJoints()->Disable();
    }

  }

  /*
   * Define Publishers and Subscribers.
   */
  void PersonNodelet::definePubSub()
  {
    ros::NodeHandle color_nh(nh_, COLOR_NAMESPACE);
    ros::NodeHandle depth_nh(nh_, DEPTH_NAMESPACE);
    ros::NodeHandle person_nh(nh_, PERSON_NAMESPACE);

    color_caminfo_sub_ = std::unique_ptr<message_filters::Subscriber<sensor_msgs::CameraInfo>>
        (new message_filters::Subscriber<sensor_msgs::CameraInfo>(color_nh, CAMERA_INFO_TOPIC, 1));
    depth_caminfo_sub_ = std::unique_ptr<message_filters::Subscriber<sensor_msgs::CameraInfo>>
        (new message_filters::Subscriber<sensor_msgs::CameraInfo>(depth_nh, CAMERA_INFO_TOPIC, 1));

    color_image_sub_ = std::unique_ptr<message_filters::Subscriber<sensor_msgs::Image>>
        (new message_filters::Subscriber<sensor_msgs::Image>(color_nh, COLOR_TOPIC, 1));
    depth_image_sub_ = std::unique_ptr<message_filters::Subscriber<sensor_msgs::Image>>
        (new message_filters::Subscriber<sensor_msgs::Image>(depth_nh, DEPTH_TOPIC, 1));

    detection_pub_ = person_nh.advertise<realsense_person::PersonDetection>(DETECTION_TOPIC, 1);
    detection_image_pub_ = person_nh.advertise<sensor_msgs::Image>(DETECTION_IMAGE_TOPIC, 1);

    tracking_pub_ = person_nh.advertise<realsense_person::PersonTracking>(TRACKING_TOPIC, 1);
    tracking_image_pub_ = person_nh.advertise<sensor_msgs::Image>(TRACKING_IMAGE_TOPIC, 1);
  }

  /*
   * Subscribe to Camera Info Topics.
   */
  void PersonNodelet::subscribeToCamInfoTopics()
  {
    caminfo_tsync_ =
        std::unique_ptr<message_filters::TimeSynchronizer<sensor_msgs::CameraInfo, sensor_msgs::CameraInfo>>
        (new message_filters::TimeSynchronizer<sensor_msgs::CameraInfo, sensor_msgs::CameraInfo>
        (*color_caminfo_sub_, *depth_caminfo_sub_, 40));
    caminfo_connection_ = message_filters::Connection(caminfo_tsync_->registerCallback(
        boost::bind(&PersonNodelet::caminfoCallback, this, _1, _2)));
  }

  /*
   * Advertise Services.
   */
  void PersonNodelet::advertiseServices()
  {
    get_tracking_state_server_ = pnh_.advertiseService(realsense_person::GET_TRACKING_STATE_SERVICE,
        &PersonNodelet::getTrackingStateServiceHandler, this);
    start_tracking_server_ = pnh_.advertiseService(realsense_person::START_TRACKING_SERVICE,
        &PersonNodelet::startTrackingServiceHandler, this);
    stop_tracking_server_ = pnh_.advertiseService(realsense_person::STOP_TRACKING_SERVICE,
        &PersonNodelet::stopTrackingServiceHandler, this);
    register_server_ = pnh_.advertiseService(realsense_person::REGISTER_SERVICE,
        &PersonNodelet::registerServiceHandler, this);
    recognize_server_ = pnh_.advertiseService(realsense_person::RECOGNIZE_SERVICE,
        &PersonNodelet::recognizeServiceHandler, this);
    reinforce_server_ = pnh_.advertiseService(realsense_person::REINFORCE_SERVICE,
        &PersonNodelet::reinforceServiceHandler, this);

/*  TODO: Adverstise these services once they are implemented in the next release.
    serialize_server_ = pnh_.advertiseService(realsense_person::SERIALIZE_SERVICE,
        &PersonNodelet::serializeServiceHandler, this);
    deserialize_server_ = pnh_.advertiseService(realsense_person::DESERIALIZE_SERVICE,
        &PersonNodelet::deserializeServiceHandler, this);
*/
  }

  /*
   * Callback for Camera Info Topics.
   */
  void PersonNodelet::caminfoCallback(const sensor_msgs::CameraInfoConstPtr& color_caminfo,
      const sensor_msgs::CameraInfoConstPtr& depth_caminfo)
  {
    ROS_INFO_STREAM(nodelet_name_ << " - color_caminfo:" << " width = " << color_caminfo->width <<
        " height = " << color_caminfo->height);
    ROS_INFO_STREAM(nodelet_name_ << " - depth_caminfo:" << " width = " << depth_caminfo->width <<
        " height = " << depth_caminfo->height);

    RSCore::intrinsics color_intrinsics = {};
    RSCore::intrinsics depth_intrinsics = {};
    RSCore::extrinsics extrinsics = {};
    memset(&color_intrinsics, 0, sizeof(color_intrinsics));
    memset(&depth_intrinsics, 0, sizeof(depth_intrinsics));
    memset(&extrinsics, 0, sizeof(extrinsics));

    getImageIntrinsics(color_caminfo, color_intrinsics);
    getImageIntrinsics(depth_caminfo, depth_intrinsics);
    getImageExtrinsics(depth_caminfo, extrinsics);

    RSCore::video_module_interface::actual_module_config module_config = {};

    setModuleConfig(static_cast<int>(RSCore::stream_type::color), color_intrinsics, module_config);
    setModuleConfig(static_cast<int>(RSCore::stream_type::depth), depth_intrinsics, module_config);

    module_config.projection = RSCore::projection_interface::create_instance(&color_intrinsics,
        &depth_intrinsics, &extrinsics);

    projection_interface_ = module_config.projection;
    pt_video_module_->set_module_config(module_config);

    ROS_INFO_STREAM(nodelet_name_ << " - Subscribing to image topics...");
    subscribeToImageTopics();
  }

  /*
   * Extract Intrinsics from Camera Info Message.
   */
  void PersonNodelet::getImageIntrinsics(const sensor_msgs::CameraInfoConstPtr& caminfo,
      RSCore::intrinsics& intrinsics)
  {
    intrinsics.width = caminfo->width;
    intrinsics.height = caminfo->height;
    intrinsics.fx = caminfo->K[0];
    intrinsics.fy = caminfo->K[4];
    intrinsics.ppx = caminfo->K[2];
    intrinsics.ppy = caminfo->K[5];
  }

  /*
   * Extract Extrinsics from Camera Info Message.
   */
  void PersonNodelet::getImageExtrinsics(const sensor_msgs::CameraInfoConstPtr& caminfo,
      RSCore::extrinsics& extrinsics)
  {
    extrinsics.translation[0] = caminfo->P[3];
    extrinsics.translation[1] = caminfo->P[7];
    extrinsics.translation[2] = caminfo->P[11];
    for (int i = 0; i < 9; ++i)
    {
      extrinsics.rotation[i] = caminfo->R[i];
    }
  }

  /*
   * Set Video Module Interface configuration.
   */
  void PersonNodelet::setModuleConfig(int image_type, RSCore::intrinsics intrinsics,
      RSCore::video_module_interface::actual_module_config& module_config)
  {
    module_config.image_streams_configs[image_type].size.height = intrinsics.height;
    module_config.image_streams_configs[image_type].size.width = intrinsics.width;
    module_config.image_streams_configs[image_type].frame_rate = subscribe_rate_;
    module_config.image_streams_configs[image_type].is_enabled = true;
  }

  /*
   * Subscribe to Image Topics.
   */
  void PersonNodelet::subscribeToImageTopics()
  {
    image_timesync_ = std::unique_ptr<message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image>>
        (new message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image>
        (*color_image_sub_, *depth_image_sub_, 40));
    image_timesync_->registerCallback(boost::bind(&PersonNodelet::imageCallback, this, _1, _2));
  }

  /*
   * Callback for Image Topics.
   */
  void PersonNodelet::imageCallback(const sensor_msgs::ImageConstPtr& color_image,
      const sensor_msgs::ImageConstPtr& depth_image)
  {
    RSCore::correlated_sample_set sample_set;
    generateSampleSet(RSCore::stream_type::color, RSCore::pixel_format::rgb8, color_image, sample_set);
    generateSampleSet(RSCore::stream_type::depth, RSCore::pixel_format::z16, depth_image, sample_set);

    processFrame(sample_set);

    current_time_ = (std::chrono::duration_cast<std::chrono::milliseconds>
        (std::chrono::system_clock::now().time_since_epoch()).count() / 1000.00);
    int current_detection_rate = 1.0 / (double)(current_time_ - last_detection_time_);
    int current_tracking_rate = 1.0 / (double)(current_time_ - last_tracking_time_);

    publish_detection_ = ((detection_pub_.getNumSubscribers() > 0) &&
        (current_detection_rate <= detection_rate_));
    publish_detection_image_ = ((detection_image_pub_.getNumSubscribers() > 0) &&
        (current_detection_rate <= detection_rate_));
    publish_tracking_ = ((tracking_id_ != -1) && (tracking_pub_.getNumSubscribers() > 0) &&
        (current_tracking_rate <= tracking_rate_));
    publish_tracking_image_ = ((tracking_id_ != -1) && (tracking_image_pub_.getNumSubscribers() > 0) &&
        (current_tracking_rate <= tracking_rate_));

    if (publish_detection_ || publish_detection_image_ || publish_tracking_ || publish_tracking_image_)
    {
      auto person_data = getPersonData();
      if (!person_data)
      {
        ROS_WARN_STREAM(nodelet_name_ << " - Could not get person data");
      }
      else
      {
        prepareMsgs(person_data, color_image, current_time_);
      }
    }

    // Disconnect Camera Info callback after it has been processed once
    if (caminfo_disconnected_ == false)
    {
      caminfo_connection_.disconnect();
      caminfo_disconnected_ = true;
    }
  }

  /*
   * Generate sample set.
   */
  void PersonNodelet::generateSampleSet(RSCore::stream_type image_type, RSCore::pixel_format image_format,
      const sensor_msgs::ImageConstPtr& image, RSCore::correlated_sample_set &sample_set)
  {
    RSCore::image_info image_info = {static_cast<int32_t>(image->width), static_cast<int32_t>(image->height),
        image_format, image->step};
    sample_set[image_type] = RSCore::image_interface::create_instance_from_raw_data(&image_info,
        RSCore::image_interface::image_data_with_data_releaser(image->data.data(), nullptr),
        RSCore::stream_type::color, RSCore::image_interface::flag::any, 0, 0);
  }

  /*
   * Process Frame.
   */
  void PersonNodelet::processFrame(RSCore::correlated_sample_set sample_set)
  {
    std::unique_lock<std::mutex> lock(frame_lock_);
    auto status = pt_video_module_->process_sample_set(sample_set);

    if (status != RSCore::status::status_no_error)
    {
      ROS_ERROR_STREAM(nodelet_name_ << " - Error while processing input frames: " << status);
    }
  }

  /*
   * Get Person Data.
   */
  PersonModule::PersonTrackingData* PersonNodelet::getPersonData()
  {
    std::unique_lock<std::mutex> lock(frame_lock_);
    return pt_video_module_->QueryOutput();
  }

  /*
   * Prepare and publish messages.
   */
  void PersonNodelet::prepareMsgs(PersonModule::PersonTrackingData* person_data,
      const sensor_msgs::ImageConstPtr& color_image, double msg_received_time)
  {
    PersonDetection detection_msg;
    PersonTracking tracking_msg;
    ros::Time header_stamp = ros::Time::now();

    auto detected_count = person_data->QueryNumberOfPeople();
    if (detected_count > 0)
    {
      for (int i = 0; i < detected_count; ++i)
      {
        auto single_person_data =
            person_data->QueryPersonData(PersonModule::PersonTrackingData::ACCESS_ORDER_BY_INDEX, i);
        auto detection_data = single_person_data->QueryTracking();
        Person person_msg = preparePersonMsg(detection_data);

        if (publish_detection_ || publish_detection_image_)
        {
          last_detection_time_ = msg_received_time;
          detection_msg.persons.push_back(person_msg);
        }

        if ((publish_tracking_ || publish_tracking_image_) && (tracking_id_ == detection_data->QueryId()))
        {
          last_tracking_time_ = msg_received_time;
          tracking_msg.person = person_msg;
          tracking_msg.face = prepareFaceMsg(single_person_data);
          tracking_msg.body = prepareBodyMsg(single_person_data);

        }
      }
    }

    // Publish Detection message
    if (publish_detection_)
    {
      detection_msg.header.stamp = header_stamp;
      detection_msg.header.frame_id = DETECTION_FRAME_ID;
      detection_pub_.publish(detection_msg);
    }

    // Publish Tracking message
    if (publish_tracking_)
    {
      tracking_msg.header.stamp = header_stamp;
      tracking_msg.header.frame_id = TRACKING_FRAME_ID;
      tracking_pub_.publish(tracking_msg);
    }

    // Publish Detection Image message
    if (publish_detection_image_)
    {
      try
      {
        cv_bridge::CvImagePtr detection_cv_img_ptr;
        detection_cv_img_ptr = cv_bridge::toCvCopy(color_image);
        prepareDetectionImageMsg(detection_msg, detection_cv_img_ptr);
        sensor_msgs::ImagePtr detection_image_msg = detection_cv_img_ptr->toImageMsg();
        detection_image_msg->header.stamp = header_stamp;
        detection_image_msg->header.frame_id = DETECTION_IMAGE_FRAME_ID;
        detection_image_pub_.publish(detection_image_msg);
      }
      catch(cv_bridge::Exception& e)
      {
        ROS_ERROR_STREAM(nodelet_name_ << " - Error converting color_msg to cv: " << e.what());
      }
    }

    // Publish Tracking Image message
    if (publish_tracking_image_)
    {
      try
      {
        cv_bridge::CvImagePtr tracking_cv_img_ptr;
        tracking_cv_img_ptr = cv_bridge::toCvCopy(color_image);
        prepareTrackingImageMsg(tracking_msg, tracking_cv_img_ptr);
        sensor_msgs::ImagePtr tracking_image_msg = tracking_cv_img_ptr->toImageMsg();
        tracking_image_msg->header.stamp = header_stamp;
        tracking_image_msg->header.frame_id = TRACKING_IMAGE_FRAME_ID;
        tracking_image_pub_.publish(tracking_image_msg);
      }
      catch(cv_bridge::Exception& e)
      {
        ROS_ERROR_STREAM(nodelet_name_ << " - Error converting color_msg to cv: " << e.what());
      }
    }
  }

  /*
   * Prepare Person Message.
   */
  Person PersonNodelet::preparePersonMsg(PersonModule::PersonTrackingData::PersonTracking* detection_data)
  {
    Person person_msg;

    auto b_box = detection_data->Query2DBoundingBox();
    auto com = detection_data->QueryCenterMass();

    person_msg.person_id.tracking_id = detection_data->QueryId();
    person_msg.person_id.recognition_id = -1; // Default value of recognition_id

    person_msg.bounding_box.top_left.x = b_box.rect.x;
    person_msg.bounding_box.top_left.y = b_box.rect.y;
    person_msg.bounding_box.bottom_right.x = b_box.rect.x + b_box.rect.w;
    person_msg.bounding_box.bottom_right.y = b_box.rect.y + b_box.rect.h;

    person_msg.center_of_mass.image.x = com.image.point.x;
    person_msg.center_of_mass.image.y = com.image.point.y;
    person_msg.center_of_mass.world.x = com.world.point.x; // MW unit already in meters
    person_msg.center_of_mass.world.y = com.world.point.y; // MW unit already in meters
    person_msg.center_of_mass.world.z = com.world.point.z; // MW unit already in meters

    return person_msg;
  }

  /*
   * Prepare Face Message.
   */
  Face PersonNodelet::prepareFaceMsg(PersonModule::PersonTrackingData::Person* single_person_data)
  {
    Face face_msg;

/*
    TODO: Temporarily removed in MW Beta3 version
    if (pt_video_module_->QueryConfiguration()->QueryTracking()->IsPersonOrientationEnabled())
    {
      auto person_orientation = single_person_data->QueryTracking()->QueryPersonOrientation();
      if (person_orientation.confidence > ORIENTATION_CONFIDENCE_THRESHOLD)
      {
        face_msg.orientation = ORIENTATION_DESC[person_orientation.orientation];
      }
    }
*/

/*
    TODO: Temporarily removed in MW Beta3 version
    if (pt_video_module_->QueryConfiguration()->QueryTracking()->IsHeadPoseEnabled())
    {
      PersonModule::PersonTrackingData::PoseEulerAngles head_angles;
      if (single_person_data->QueryFace()->QueryHeadPose(head_angles))
      {
        face_msg.head_pose.x = head_angles.pitch;
        face_msg.head_pose.y = head_angles.roll;
        face_msg.head_pose.z = head_angles.yaw;
      }
    }
*/
    if (pt_video_module_->QueryConfiguration()->QueryTracking()->IsHeadBoundingBoxEnabled())
    {
      auto head_b_box = single_person_data->QueryTracking()->QueryHeadBoundingBox();
      if (head_b_box.confidence > HEAD_BOX_CONFIDENCE_THRESHOLD)
      {
        face_msg.head_bounding_box.top_left.x = head_b_box.rect.x;
        face_msg.head_bounding_box.top_left.y = head_b_box.rect.y;
        face_msg.head_bounding_box.bottom_right.x = head_b_box.rect.x + head_b_box.rect.w;
        face_msg.head_bounding_box.bottom_right.y = head_b_box.rect.y + head_b_box.rect.h;
      }
    }

    if (pt_video_module_->QueryConfiguration()->QueryFace()->IsFaceLandmarksEnabled())
    {
      auto landmarks_info = single_person_data->QueryFace()->QueryLandmarks();
      auto num_of_landmarks = single_person_data->QueryFace()->QueryNumLandmarks();
      if ((num_of_landmarks > 0) && (landmarks_info->confidence > LANDMARKS_CONFIDENCE_THRESHOLD))
      {
        auto landmark = landmarks_info->landmarks;
        for (int i = 0; i < landmarks_info->numLandmarks; ++i)
        {
          RegisteredPoint reg_point;
          reg_point.image.x = landmark->image.x;
          reg_point.image.y = landmark->image.y;
          reg_point.world.x = landmark->world.x/1000; // convert mm to meters
          reg_point.world.y = landmark->world.y/1000; // convert mm to meters
          reg_point.world.z = landmark->world.z/1000; // convert mm to meters
          face_msg.landmarks.push_back(reg_point);
          landmark = ++landmark;
        }
      }
    }

    return face_msg;
  }

  /*
   * Prepare Body Message.
   */
  Body PersonNodelet::prepareBodyMsg(PersonModule::PersonTrackingData::Person* single_person_data)
  {
    Body body_msg;

    if (pt_video_module_->QueryConfiguration()->QueryGestures()->IsEnabled())
    {
      auto person_gestures = single_person_data->QueryGestures();
      if (person_gestures)
      {
        // Only Pointing gesture is supported in this version of the MW
        if ((person_gestures->IsPointing())
            && (person_gestures->QueryPointingInfo().confidence > GESTURE_CONFIDENCE_THRESHOLD))
        {
          body_msg.gesture.type = "Pointing";
          auto pointing_info = person_gestures->QueryPointingInfo();
          body_msg.gesture.origin.image.x = pointing_info.colorPointingData.origin.x;
          body_msg.gesture.origin.image.y = pointing_info.colorPointingData.origin.y;
          body_msg.gesture.origin.world.x = pointing_info.worldPointingData.origin.x/1000; // convert mm to meters
          body_msg.gesture.origin.world.y = pointing_info.worldPointingData.origin.y/1000; // convert mm to meters
          body_msg.gesture.origin.world.z = pointing_info.worldPointingData.origin.z/1000; // convert mm to meters
          body_msg.gesture.direction.image.x = pointing_info.colorPointingData.direction.x;
          body_msg.gesture.direction.image.y = pointing_info.colorPointingData.direction.y;
          body_msg.gesture.direction.world.x = pointing_info.worldPointingData.direction.x/1000; // convert mm to meters
          body_msg.gesture.direction.world.y = pointing_info.worldPointingData.direction.y/1000; // convert mm to meters
          body_msg.gesture.direction.world.z = pointing_info.worldPointingData.direction.z/1000; // convert mm to meters
        }
      }
    }

    if (pt_video_module_->QueryConfiguration()->QuerySkeletonJoints()->IsEnabled())
    {
      auto person_joints = single_person_data->QuerySkeletonJoints();
      auto num_of_joints = person_joints->QueryNumJoints();
      if (num_of_joints > 0)
      {
        std::vector<PersonModule::PersonTrackingData::PersonJoints::SkeletonPoint> skeleton_points(num_of_joints);
        person_joints->QueryJoints(skeleton_points.data());
        for (const auto& skeleton_point: skeleton_points)
        {
          // Based on MW sample program, checking only Image confidence and not World confidence
          if (skeleton_point.confidenceImage > SKELETON_CONFIDENCE_THRESHOLD)
          {
            SkeletonJoint skeleton_joint;
            skeleton_joint.type = JOINT_TYPE_DESC[skeleton_point.jointType];
            skeleton_joint.point.image.x = skeleton_point.image.x;
            skeleton_joint.point.image.y = skeleton_point.image.y;
            skeleton_joint.point.world.x = skeleton_point.world.x/1000; // convert mm to meters
            skeleton_joint.point.world.y = skeleton_point.world.y/1000; // convert mm to meters
            skeleton_joint.point.world.z = skeleton_point.world.z/1000; // convert mm to meters
            body_msg.skeleton_joints.push_back(skeleton_joint);
          }
        }
      }
    }

    return body_msg;
  }

  /*
   * Draw Person.
   */
  void PersonNodelet::drawPerson(Person person_msg, cv_bridge::CvImagePtr& cv_ptr)
  {
    cv::Point pt1(person_msg.bounding_box.top_left.x, person_msg.bounding_box.top_left.y);
    cv::Point pt2(person_msg.bounding_box.bottom_right.x, person_msg.bounding_box.bottom_right.y);
    cv::Rect rect(pt1, pt2);
    cv::rectangle(cv_ptr->image, rect, cv::Scalar(0, 255, 0), 3);

    cv::circle(cv_ptr->image, cv::Point(person_msg.center_of_mass.image.x,
        person_msg.center_of_mass.image.y), 5, cv::Scalar(0, 255, 0), 3);

    std::stringstream id;
    id << "tid: " << person_msg.person_id.tracking_id;
    /* TODO: Get the real recognition id from the db once that feature is implemented in the next release.
    if (person_msg.person_id.recognition_id >= 0)
    {
      id << ", rid: " << person_msg.person_id.recognition_id;
    }
    */

    cv::putText(cv_ptr->image, id.str(), cv::Point(person_msg.bounding_box.top_left.x,
        (person_msg.bounding_box.top_left.y - 1.0)), cv::FONT_HERSHEY_PLAIN, 1.5, cv::Scalar(0, 255, 0), 2, 8);
  }

  /*
   * Draw Orientation.
   */
  void PersonNodelet::drawOrientation(std::string orientation, cv_bridge::CvImagePtr& cv_ptr)
  {
    std::stringstream orientation_txt;
    orientation_txt << std::setprecision(1) << std::fixed << " Orientation: " << orientation;
    cv::putText(cv_ptr->image, orientation_txt.str(), cv::Point(5, 35),
        cv::FONT_HERSHEY_PLAIN, 1.5, cv::Scalar(255, 0, 0), 2, 8);
  }

  /*
   * Draw HeadBoundingBox.
   */
  void PersonNodelet::drawHeadBoundingBox(BoundingBox b_box, cv_bridge::CvImagePtr& cv_ptr)
  {
    cv::Point pt1(b_box.top_left.x, b_box.top_left.y);
    cv::Point pt2(b_box.bottom_right.x, b_box.bottom_right.y);
    cv::Rect rect(pt1, pt2);
    cv::rectangle(cv_ptr->image, rect, cv::Scalar(255, 0, 0), 2);
  }

  /*
   * Draw HeadPose.
   */
  void PersonNodelet::drawHeadPose(geometry_msgs::Point head_pose, cv_bridge::CvImagePtr& cv_ptr)
  {
    std::stringstream head_pose_txt;
    head_pose_txt << std::setprecision(1) << std::fixed << " Head Pose: " <<
        "(" << head_pose.x << ", " << head_pose.y << ", " << head_pose.z << ")";
    cv::putText(cv_ptr->image, head_pose_txt.str(), cv::Point(5, 65),
        cv::FONT_HERSHEY_PLAIN, 1.5, cv::Scalar(255, 0, 0), 2, 8);
  }

  /*
   * Draw Landmarks.
   */
  void PersonNodelet::drawLandmarks(std::vector<RegisteredPoint> landmarks, cv_bridge::CvImagePtr& cv_ptr)
  {
    for (const auto& reg_point: landmarks)
    {
      cv::Point point(reg_point.image.x, reg_point.image.y);
      cv::circle(cv_ptr->image, point, 3, cv::Scalar(0, 255, 0), -1, 8, 0);
    }
  }

  /*
   * Draw Gestures.
   */
  void PersonNodelet::drawGestures(Gesture gesture, cv_bridge::CvImagePtr& cv_ptr)
  {
    cv::Point origin(gesture.origin.image.x, gesture.origin.image.y);
    cv::circle(cv_ptr->image, origin, 3, cv::Scalar(0, 0, 255), -1, 8, 0);
    cv::Point gesture_vector(origin.x + 400 * gesture.direction.image.x,
        origin.y + 400 * gesture.direction.image.y);
    cv::arrowedLine(cv_ptr->image, origin, gesture_vector, cv::Scalar(0, 0, 255), 2, 8, 0, 0.15);
    cv::putText(cv_ptr->image, gesture.type, cv::Point(origin.x, origin.y),
        cv::FONT_HERSHEY_PLAIN, 1.5, cv::Scalar(0, 0, 255), 2, 8);
  }

  /*
   * Draw SkeletonJoints.
   */
  void PersonNodelet::drawSkeletonJoints(std::vector<SkeletonJoint> skeleton_joints, cv_bridge::CvImagePtr& cv_ptr)
  {
    for (const auto& skeleton_joint: skeleton_joints)
    {
      cv::Point point(skeleton_joint.point.image.x, skeleton_joint.point.image.y);
      cv::circle(cv_ptr->image, point, 5, cv::Scalar(255, 0, 0), -1, 8, 0);
      cv::putText(cv_ptr->image, skeleton_joint.type, cv::Point(point.x, point.y),
          cv::FONT_HERSHEY_PLAIN, 1.5, cv::Scalar(0, 0, 255), 2, 8);
    }
  }

  /*
   * Prepare DetectionImage Message.
   */
  void PersonNodelet::prepareDetectionImageMsg(PersonDetection detection_msg, cv_bridge::CvImagePtr& cv_ptr)
  {
    for (Person person_msg: detection_msg.persons)
    {
      drawPerson(person_msg, cv_ptr);
    }
  }

  /*
   * Prepare TrackingImage Message.
   */
  void PersonNodelet::prepareTrackingImageMsg(PersonTracking tracking_msg, cv_bridge::CvImagePtr& cv_ptr)
  {
    drawPerson(tracking_msg.person, cv_ptr);
/*  TODO: Temporarily removed from MW Beta3 version
    drawOrientation(tracking_msg.face.orientation, cv_ptr);
    drawHeadPose(tracking_msg.face.head_pose, cv_ptr);
*/
    drawHeadBoundingBox(tracking_msg.face.head_bounding_box, cv_ptr);
    drawLandmarks(tracking_msg.face.landmarks, cv_ptr);
    drawGestures(tracking_msg.body.gesture, cv_ptr);
    drawSkeletonJoints(tracking_msg.body.skeleton_joints, cv_ptr);
  }

  /*
   * Handle GetTrackingState Service call.
   */
  bool PersonNodelet::getTrackingStateServiceHandler(realsense_person::GetTrackingState::Request &req,
      realsense_person::GetTrackingState::Response &res)
  {
    ROS_INFO_STREAM(nodelet_name_ << " - Calling service: " << GET_TRACKING_STATE_SERVICE);
    auto person_data = getPersonData();
    if (!person_data)
    {
      res.state = -1;
      res.state_desc = "Could not get person data";
    }
    else
    {
      bool tracking_id_found = false;
      auto detected_count = person_data->QueryNumberOfPeople();
      for (int i = 0; i < detected_count; ++i)
      {
        auto single_person_data =
            person_data->QueryPersonData(PersonModule::PersonTrackingData::ACCESS_ORDER_BY_INDEX, i);
        auto detection_data = single_person_data->QueryTracking();
        auto tracking_id = detection_data->QueryId();
        res.tracking_ids.push_back(tracking_id);
        if (tracking_id == tracking_id_)
        {
          tracking_id_found = true;
        }
      }
      if (tracking_id_found)
      {
        res.state = 1;
        res.state_desc = "Tracking";
      }
      else
      {
        // Currently, the MW does not have an API to check if tracking is disabled.
        // So using the tracking_id_ variable to determine the same.
        if (tracking_id_ != -1)
        {
          res.state = 2;
          std::stringstream desc;
          desc << "Tracking but tracking_id " << tracking_id_ << " not in frame.";
          res.state_desc = desc.str();
        }
        else
        {
          res.state = 0;
          res.state_desc = "Not Tracking";
        }
      }
    }
    return true;
  }

  /*
   * Handle StartTracking Service call.
   */
  bool PersonNodelet::startTrackingServiceHandler(realsense_person::StartTracking::Request &req,
      realsense_person::StartTracking::Response &res)
  {
    ROS_INFO_STREAM(nodelet_name_ << " - Calling service: " << START_TRACKING_SERVICE);
    auto person_data = getPersonData();
    if (!person_data)
    {
      res.status = -1;
      res.status_desc = "Could not get person_data";
    }
    else
    {
      // Currently, the MW API does not indicate if start tracking was successful.
      // So adding this logic to determine if atleast the input tracking_id is valid
      bool tracking_id_found = false;
      auto detected_count = person_data->QueryNumberOfPeople();
      for (int i = 0; i < detected_count; ++i)
      {
        auto single_person_data =
            person_data->QueryPersonData(PersonModule::PersonTrackingData::ACCESS_ORDER_BY_INDEX, i);
        auto detection_data = single_person_data->QueryTracking();
        auto tracking_id = detection_data->QueryId();
        if (tracking_id == req.tracking_id)
        {
          tracking_id_found = true;
          break;
        }
      }
      if (!tracking_id_found)
      {
        res.status = 1;
        res.status_desc = "tracking_id not found";
      }
      else
      {
        // Currently, the MW does not have an API to check if tracking is enabled.
        // So using the tracking_id_ variable to determine the same.
        if (tracking_id_ == -1)
        {
          pt_video_module_->QueryConfiguration()->QueryTracking()->Enable();
        }
        tracking_id_ = req.tracking_id;
        person_data->StartTracking(tracking_id_);
        res.status = 0;
        res.status_desc = "Started Tracking";
      }
    }
    return true;
  }

  /*
   * Handle StopTracking Service call.
   */
  bool PersonNodelet::stopTrackingServiceHandler(realsense_person::StopTracking::Request &req,
      realsense_person::StopTracking::Response &res)
  {
    ROS_INFO_STREAM(nodelet_name_ << " - Calling service: " << STOP_TRACKING_SERVICE);
    auto person_data = getPersonData();
    if (!person_data)
    {
      res.status = -1;
      res.status_desc = "Could not get person data";
    }
    else
    {
      // Currently, the MW does not have an API to check if tracking is disabled.
      // So using the tracking_id_ variable to determine the same.
      if (tracking_id_ == -1)
      {
        res.status = 1;
        res.status_desc = "Already not Tracking";
      }
      else
      {
        person_data->StopTracking(tracking_id_);
        res.status = 0;
        res.status_desc = "Stopped Tracking";
        tracking_id_ = -1;
      }
    }
    return true;
  }

  /*
   * Handle Register Service call.
   */
  bool PersonNodelet::registerServiceHandler(realsense_person::Register::Request &req,
      realsense_person::Register::Response &res)
  {
    ROS_INFO_STREAM(nodelet_name_ << " - Calling service: " << REGISTER_SERVICE);
    res.status = -1;
    res.recognition_id = -1;
    if (!pt_video_module_->QueryConfiguration()->QueryRecognition()->IsEnabled())
    {
      res.status_desc = "enable_recognition param not set";
    }
    else
    {
      auto person_data = getPersonData();
      if (!person_data)
      {
        res.status = -1;
        res.status_desc = "Could not get person data";
      }
      else
      {
        auto person = person_data->QueryPersonDataById(req.tracking_id);
        if (!person)
        {
          res.status_desc = "tracking_id not found";
        }
        else
        {
          int32_t o_recognition_id = -1;
          int32_t o_tracking_id;
          int32_t o_descriptor_id;
          res.status = person->QueryRecognition()->RegisterUser(&o_recognition_id, &o_tracking_id, &o_descriptor_id);
          res.status_desc = REGISTRATION_DESC[res.status];
          res.recognition_id = o_recognition_id;
        }
      }
    }
    return true;
  }

  /*
   * Handle Recognize Service call.
   */
  bool PersonNodelet::recognizeServiceHandler(realsense_person::Recognize::Request &req,
      realsense_person::Recognize::Response &res)
  {
    ROS_INFO_STREAM(nodelet_name_ << " - Calling service: " << RECOGNIZE_SERVICE);
    if (!pt_video_module_->QueryConfiguration()->QueryRecognition()->IsEnabled())
    {
      res.status = -1;
      res.status_desc = "enable_recognition param not set";
      res.recognition_id = -1;
    }
    else
    {
      auto person_data = getPersonData();
      if (!person_data)
      {
        res.status = -1;
        res.status_desc = "Could not get person data";
      }
      else
      {
        auto person = person_data->QueryPersonDataById(req.tracking_id);
        if (!person)
        {
          res.status = -1;
          res.status_desc = "tracking_id not found";
          res.recognition_id = -1;
        }
        else
        {
          PersonModule::PersonTrackingData::PersonRecognition::RecognizerData result;
          res.status = person->QueryRecognition()->RecognizeUser(&result);
          res.status_desc = RECOGNITION_DESC[res.status];
          res.recognition_id = result.recognitionId;
        }
      }
    }
    return true;
  }

  /*
   * Handle Reinforce Service call.
   */
  bool PersonNodelet::reinforceServiceHandler(realsense_person::Reinforce::Request &req,
      realsense_person::Reinforce::Response &res)
  {
    ROS_INFO_STREAM(nodelet_name_ << " - Calling service: " << REINFORCE_SERVICE);
    if (!pt_video_module_->QueryConfiguration()->QueryRecognition()->IsEnabled())
    {
      res.status = -1;
      res.status_desc = "enable_recognition param not set";
    }
    else
    {
      auto person_data = getPersonData();
      if (!person_data)
      {
        res.status = -1;
        res.status_desc = "Could not get person data";
      }
      else
      {
        auto person = person_data->QueryPersonDataById(req.tracking_id);
        if (!person)
        {
          res.status = -1;
          res.status_desc = "tracking_id not found";
        }
        else
        {
          int32_t o_tracking_id;
          int32_t o_descriptor_id;
          res.status = person->QueryRecognition()->ReinforceUserRegistration(req.recognition_id, &o_tracking_id,
                &o_descriptor_id);
          res.status_desc = REGISTRATION_DESC[res.status];
        }
      }
    }
    return true;
  }

  /*
   * Handle Serialize Service call.
   */
  bool PersonNodelet::serializeServiceHandler(realsense_person::Serialize::Request &req,
        realsense_person::Serialize::Response &res)
  {
    //TODO: Implement the logic in the next release.
    return true;
  }

  /*
   * Handle Deserialize Service call
   */
  bool PersonNodelet::deserializeServiceHandler(realsense_person::Deserialize::Request &req,
        realsense_person::Deserialize::Response &res)
  {
    //TODO: Implement the logic in the next release.
    return true;
  }

}

