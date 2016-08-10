/******************************************************************************
 Copyright (c) 2016, Intel Corporation
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
#ifndef BASE_NODELET
#define BASE_NODELET

#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32MultiArray.h>

#include <opencv2/core/core.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>

#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>

#include <iostream>
#include <boost/thread.hpp>
#include <thread>
#include <cstdlib>
#include <cctype>
#include <algorithm>
#include <sstream>
#include <memory>
#include <map>

#include <librealsense/rs.hpp>
#include <realsense_camera/cameraConfiguration.h>
#include <pluginlib/class_list_macros.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <realsense_camera/constants.h>

namespace realsense_camera
{
  class BaseNodelet: public nodelet::Nodelet
  {
  public:

    // Interfaces.
    virtual void onInit();
    virtual ~BaseNodelet();
    virtual void prepareTopics();
    virtual bool getCameraOptionValues(realsense_camera::cameraConfiguration::Request & req,
        realsense_camera::cameraConfiguration::Response & res);

  protected:

    // Member Variables.
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Time topic_ts_;
    ros::Publisher pointcloud_publisher_;
    ros::ServiceServer get_options_service_;
    tf2_ros::StaticTransformBroadcaster static_tf_broadcaster_;
    rs_error *rs_error_ = 0;
    rs_context *rs_context_;
    rs_device *rs_device_;
    std::string nodelet_name_;
    std::string serial_no_;
    std::string usb_port_id_;
    std::string camera_type_;
    std::string mode_;
    bool enable_[STREAM_COUNT];
    int width_[STREAM_COUNT];
    int height_[STREAM_COUNT];
    int fps_[STREAM_COUNT];
    rs_format format_[STREAM_COUNT];
    std::string encoding_[STREAM_COUNT];
    int cv_type_[STREAM_COUNT];
    int unit_step_size_[STREAM_COUNT];
    int step_[STREAM_COUNT];
    int ts_[STREAM_COUNT];
    std::string frame_id_[STREAM_COUNT];
    cv::Mat image_[STREAM_COUNT] = {};
    image_transport::CameraPublisher camera_publisher_[STREAM_COUNT] = {};
    sensor_msgs::CameraInfoPtr camera_info_ptr_[STREAM_COUNT] = {};
    std::string base_frame_id_;
    std::string depth_frame_id_;
    std::string color_frame_id_;
    float max_z_ = -1.0f;
    bool enable_pointcloud_;
    bool enable_tf_;
    bool duplicate_depth_color_;
    const uint16_t *image_depth16_;
    boost::shared_ptr<boost::thread> topic_thread_;

    struct CameraOptions
    {
      rs_option opt;
      double min, max, step, value;
    };
    std::vector<CameraOptions> camera_options_;

    // Member Functions.
    virtual void getParameters();
    virtual bool connectToCamera();
    virtual std::vector<int> listCameras(int num_of_camera);
    virtual void advertiseTopics();
    virtual void advertiseServices();
    virtual std::vector<std::string> setDynamicReconfServer() { return {}; }; // must be defined in derived class
    virtual void startDynamicReconfCallback() { return; } // must be defined in derived class
    virtual void getCameraOptions();
    virtual void setStaticCameraOptions(std::vector<std::string> dynamic_params);
    virtual void setStreams();
    virtual void enableStream(rs_stream stream_index, int width, int height, rs_format format, int fps);
    virtual void getStreamCalibData(rs_stream stream_index);
    virtual void disableStream(rs_stream stream_index);
    virtual void startCamera();
    virtual void stopCamera();
    virtual void publishTopics();
    virtual void publishTopic(rs_stream stream_index);
    virtual void getStreamData(rs_stream stream_index);
    virtual void publishPCTopic();
    virtual void publishStaticTransforms();
    virtual void checkError();
  };
}
#endif
