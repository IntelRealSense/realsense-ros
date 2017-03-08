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

#include <realsense_camera/sync_nodelet.h>

PLUGINLIB_EXPORT_CLASS(realsense_camera::SyncNodelet, nodelet::Nodelet)

namespace realsense_camera
{
  SyncNodelet::~SyncNodelet()
  {
    topic_thread_->join();
  }

  /*
   * Initialize the nodelet.
   */
  void SyncNodelet::onInit()
  {
    BaseNodelet::onInit();

    // start thread to publish topics
    topic_thread_ =
            boost::shared_ptr <boost::thread>(new boost::thread (boost::bind(&SyncNodelet::publishSyncTopics, this)));
  }

  void SyncNodelet::publishSyncTopics() try
  {
    while (ros::ok())
    {
      if (start_stop_srv_called_ == true)
      {
        if (start_camera_ == true)
        {
          ROS_INFO_STREAM(nodelet_name_ << " - " << startCamera());
        }
        else
        {
          ROS_INFO_STREAM(nodelet_name_ << " - " << stopCamera());
        }

        start_stop_srv_called_ = false;
      }

      if (enable_[RS_STREAM_DEPTH] != rs_is_stream_enabled(rs_device_, RS_STREAM_DEPTH, 0))
      {
        stopCamera();
        setStreams();
        startCamera();
      }

      if (rs_is_device_streaming(rs_device_, 0) == 1)
      {
        rs_wait_for_frames(rs_device_, &rs_error_);
        checkError();
        topic_ts_ = ros::Time::now();
        duplicate_depth_color_ = false;

        for (int stream=0; stream < STREAM_COUNT; stream++)
        {
          if (enable_[stream] == true)
          {
            publishTopic(static_cast<rs_stream>(stream));
          }
        }

        if (pointcloud_publisher_.getNumSubscribers() > 0 &&
            rs_is_stream_enabled(rs_device_, RS_STREAM_DEPTH, 0) == 1 && enable_pointcloud_ == true &&
            (duplicate_depth_color_ == false))  // Skip publishing PointCloud if Depth or Color frame was duplicate
        {
          if (camera_publisher_[RS_STREAM_DEPTH].getNumSubscribers() <= 0)
          {
            setImageData(RS_STREAM_DEPTH);
          }

          if (camera_publisher_[RS_STREAM_COLOR].getNumSubscribers() <= 0)
          {
            setImageData(RS_STREAM_COLOR);
          }

          publishPCTopic();
        }
      }
    }
  }
  catch(const rs::error & e)
  {
    ROS_ERROR_STREAM(nodelet_name_ << " - " << "RealSense error calling "
        << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    "
        << e.what());
    ros::shutdown();
  }
  catch(const std::exception & e)
  {
    ROS_ERROR_STREAM(nodelet_name_ << " - " << e.what());
    ros::shutdown();
  }
  catch(...)
  {
    ROS_ERROR_STREAM(nodelet_name_ << " - Caught unknown expection...shutting down!");
    ros::shutdown();
  }

  /*
   * Publish topic.
   */
  void SyncNodelet::publishTopic(rs_stream stream_index)
  {
    // Publish stream only if there is at least one subscriber.
    if (camera_publisher_[stream_index].getNumSubscribers() > 0 &&
        rs_is_stream_enabled(rs_device_, (rs_stream) stream_index, 0) == 1)
    {
      double frame_ts = rs_get_frame_timestamp(rs_device_, (rs_stream) stream_index, 0);
      if (ts_[stream_index] != frame_ts)  // Publish frames only if its not duplicate
      {
        setImageData(stream_index);

        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(),
            encoding_[stream_index],
            image_[stream_index]).toImageMsg();

        msg->header.frame_id = optical_frame_id_[stream_index];
        msg->header.stamp = topic_ts_;  // Publish timestamp to synchronize frames.
        msg->width = image_[stream_index].cols;
        msg->height = image_[stream_index].rows;
        msg->is_bigendian = false;
        msg->step = step_[stream_index];

        camera_info_ptr_[stream_index]->header.stamp = msg->header.stamp;
        camera_publisher_[stream_index].publish(msg, camera_info_ptr_[stream_index]);
      }
      else
      {
        if ((stream_index == RS_STREAM_DEPTH) || (stream_index == RS_STREAM_COLOR))
        {
          duplicate_depth_color_ = true;  // Set this flag to true if Depth and/or Color frame is duplicate
        }
      }

      ts_[stream_index] = frame_ts;
    }
  }

  void SyncNodelet::setImageData(rs_stream stream_index)
    {
      if (stream_index == RS_STREAM_DEPTH)
      {
        // fill depth buffer
        image_depth16_ = reinterpret_cast<const uint16_t *>(rs_get_frame_data(rs_device_, stream_index, 0));
        float depth_scale_meters = rs_get_device_depth_scale(rs_device_, &rs_error_);
        if (depth_scale_meters == MILLIMETER_METERS)
        {
          image_[stream_index].data = (unsigned char *) image_depth16_;
        }
        else
        {
          cvWrapper_ = cv::Mat(image_[stream_index].size(), cv_type_[stream_index],
                               const_cast<void *>(reinterpret_cast<const void *>(image_depth16_)),
                               step_[stream_index]);
          cvWrapper_.convertTo(image_[stream_index], cv_type_[stream_index],
                static_cast<double>(depth_scale_meters) / static_cast<double>(MILLIMETER_METERS));
        }
      }
      else
      {
        image_[stream_index].data = (unsigned char *) (rs_get_frame_data(rs_device_, stream_index, 0));
      }
    }
}  // namespace realsense_camera
