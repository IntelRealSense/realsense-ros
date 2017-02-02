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

#pragma once
#ifndef PERSON_NODELET
#define PERSON_NODELET

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <dynamic_reconfigure/server.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <realsense_person/constants.h>
#include <realsense_person/person_paramsConfig.h>
#include <realsense_person/PersonDetection.h>
#include <realsense_person/PersonTracking.h>
#include <realsense_person/PersonId.h>
#include <realsense_person/Person.h>
#include <realsense_person/Face.h>
#include <realsense_person/Body.h>
#include <realsense_person/Gesture.h>
#include <realsense_person/SkeletonJoint.h>
#include <realsense_person/RegisteredPoint.h>
#include <realsense_person/BoundingBox.h>
#include <realsense_person/Pixel.h>
#include <realsense_person/GetTrackingState.h>
#include <realsense_person/Recognize.h>
#include <realsense_person/Register.h>
#include <realsense_person/Reinforce.h>
#include <realsense_person/Serialize.h>
#include <realsense_person/Deserialize.h>
#include <realsense_person/StartTracking.h>
#include <realsense_person/StopTracking.h>

#include <RealSense/PersonTracking/PersonTrackingConfiguration.h>
#include <person_tracking_video_module_interface.h>
#include <person_tracking_video_module_factory.h>
#include <rs/core/projection_interface.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <mutex>


namespace PersonModule = Intel::RealSense::PersonTracking;
namespace PersonModuleInterface = rs::person_tracking;
namespace RSCore = rs::core;

namespace realsense_person
{
  class PersonNodelet: public nodelet::Nodelet
  {
  public:
    PersonNodelet();
    ~PersonNodelet();
    void onInit();

  protected:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    std::string nodelet_name_;
    int tracking_id_;
    int subscribe_rate_;
    int detection_rate_;
    int tracking_rate_;

    double current_time_;
    double last_detection_time_;
    double last_tracking_time_;
    bool publish_detection_;
    bool publish_detection_image_;
    bool publish_tracking_;
    bool publish_tracking_image_;

    std::unique_ptr<message_filters::Subscriber<sensor_msgs::CameraInfo>> color_caminfo_sub_;
    std::unique_ptr<message_filters::Subscriber<sensor_msgs::CameraInfo>> depth_caminfo_sub_;
    std::unique_ptr<message_filters::TimeSynchronizer<sensor_msgs::CameraInfo, sensor_msgs::CameraInfo>> caminfo_tsync_;
    message_filters::Connection caminfo_connection_;
    bool caminfo_disconnected_;

    std::unique_ptr<message_filters::Subscriber<sensor_msgs::Image>> color_image_sub_;
    std::unique_ptr<message_filters::Subscriber<sensor_msgs::Image>> depth_image_sub_;
    std::unique_ptr<message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image>> image_timesync_;
    message_filters::Connection image_connection_;

    RSCore::projection_interface* projection_interface_;
    std::unique_ptr<PersonModuleInterface::person_tracking_video_module_interface> pt_video_module_;

    ros::ServiceServer get_tracking_state_server_;
    ros::ServiceServer register_server_;
    ros::ServiceServer recognize_server_;
    ros::ServiceServer reinforce_server_;
    ros::ServiceServer start_tracking_server_;
    ros::ServiceServer stop_tracking_server_;
    ros::ServiceServer serialize_server_;
    ros::ServiceServer deserialize_server_;
    
    ros::Publisher detection_pub_;
    ros::Publisher detection_image_pub_;
    ros::Publisher tracking_pub_;
    ros::Publisher tracking_image_pub_;

    boost::shared_ptr<dynamic_reconfigure::Server<realsense_person::person_paramsConfig>> dynamic_reconf_server_;
    std::mutex frame_lock_;

    virtual bool getTrackingStateServiceHandler(realsense_person::GetTrackingState::Request &req,
        realsense_person::GetTrackingState::Response &res);
    virtual bool startTrackingServiceHandler(realsense_person::StartTracking::Request &req,
        realsense_person::StartTracking::Response &res);
    virtual bool stopTrackingServiceHandler(realsense_person::StopTracking::Request &req,
        realsense_person::StopTracking::Response &res);
    virtual bool recognizeServiceHandler(realsense_person::Recognize::Request &req,
        realsense_person::Recognize::Response &res);
    virtual bool registerServiceHandler(realsense_person::Register::Request &req,
        realsense_person::Register::Response &res);
    virtual bool reinforceServiceHandler(realsense_person::Reinforce::Request &req,
        realsense_person::Reinforce::Response &res);
    virtual bool serializeServiceHandler(realsense_person::Serialize::Request &req,
        realsense_person::Serialize::Response &res);
    virtual bool deserializeServiceHandler(realsense_person::Deserialize::Request &req,
        realsense_person::Deserialize::Response &res);

    void dynamicReconfCallback(realsense_person::person_paramsConfig &config, uint32_t level);
    void getStaticParameters();
    void startDynamicReconfServer();
    void definePubSub();
    void subscribeToCamInfoTopics();
    void advertiseServices();
    void caminfoCallback(const sensor_msgs::CameraInfoConstPtr& color_caminfo,
        const sensor_msgs::CameraInfoConstPtr& depth_caminfo);
    void getImageIntrinsics(const sensor_msgs::CameraInfoConstPtr& caminfo, RSCore::intrinsics& intrinsics);
    void getImageExtrinsics(const sensor_msgs::CameraInfoConstPtr& caminfo, RSCore::extrinsics& extrinsics);
    void setModuleConfig(int image_type, RSCore::intrinsics intrinsics,
        RSCore::video_module_interface::actual_module_config& module_config);
    void subscribeToImageTopics();
    void imageCallback(const sensor_msgs::ImageConstPtr& color_image, const sensor_msgs::ImageConstPtr& depth_image);
    void generateSampleSet(RSCore::stream_type image_type, RSCore::pixel_format image_format,
        const sensor_msgs::ImageConstPtr& image, RSCore::correlated_sample_set &sample_set);
    void processFrame(RSCore::correlated_sample_set sample_set);
    PersonModule::PersonTrackingData* getPersonData();
    void prepareMsgs(PersonModule::PersonTrackingData* person_data, const sensor_msgs::ImageConstPtr& color_image,
        double msg_received_time);
    Person preparePersonMsg(PersonModule::PersonTrackingData::PersonTracking* detection_data);
    Face prepareFaceMsg(PersonModule::PersonTrackingData::Person* single_person_data);
    Body prepareBodyMsg(PersonModule::PersonTrackingData::Person* single_person_data);
    void drawPerson(Person person_msg, cv_bridge::CvImagePtr& cv_ptr);
    void drawOrientation(std::string orientation, cv_bridge::CvImagePtr& cv_ptr);
    void drawHeadPose(geometry_msgs::Point head_pose, cv_bridge::CvImagePtr& cv_ptr);
    void drawHeadBoundingBox(BoundingBox b_box, cv_bridge::CvImagePtr& cv_ptr);
    void drawLandmarks(std::vector<RegisteredPoint> landmarks, cv_bridge::CvImagePtr& cv_ptr);
    void drawGestures(Gesture person_gesture, cv_bridge::CvImagePtr& cv_ptr);
    void drawSkeletonJoints(std::vector<SkeletonJoint> skeleton_joints, cv_bridge::CvImagePtr& cv_ptr);
    void prepareDetectionImageMsg(PersonDetection detection_msg, cv_bridge::CvImagePtr& cv_ptr);
    void prepareTrackingImageMsg(PersonTracking tracking_msg, cv_bridge::CvImagePtr& cv_ptr);
  };
}
#endif
