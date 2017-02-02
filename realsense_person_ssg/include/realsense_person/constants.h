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

#include <iostream>

#pragma once
#ifndef NODELET_CONSTANTS
#define NODELET_CONSTANTS

namespace realsense_person
{
  // Default Constants.
  const int SUBSCRIBE_RATE = 30;

  const std::string COLOR_NAMESPACE = "color";
  const std::string COLOR_TOPIC = "image_raw";
  const std::string DEPTH_NAMESPACE = "depth";
  const std::string DEPTH_TOPIC = "image_raw";
  const std::string CAMERA_INFO_TOPIC = "camera_info";
  const std::string PERSON_NAMESPACE = "person";

  const std::string DETECTION_TOPIC = "detection_data";
  const std::string DETECTION_FRAME_ID = "detection_data";

  const std::string DETECTION_IMAGE_TOPIC = "detection_image";
  const std::string DETECTION_IMAGE_FRAME_ID = "detection_image";

  const std::string TRACKING_TOPIC = "tracking_data";
  const std::string TRACKING_FRAME_ID = "tracking_data";

  const std::string TRACKING_IMAGE_TOPIC = "tracking_image";
  const std::string TRACKING_IMAGE_FRAME_ID = "tracking_image";

  const std::string GET_TRACKING_STATE_SERVICE = "get_tracking_state";
  const std::string REGISTER_SERVICE = "register_person";
  const std::string RECOGNIZE_SERVICE = "recognize_person";
  const std::string REINFORCE_SERVICE = "reinforce_person";
  const std::string START_TRACKING_SERVICE = "start_tracking_person";
  const std::string STOP_TRACKING_SERVICE = "stop_tracking";
  const std::string SERIALIZE_SERVICE = "serialize_db";
  const std::string DESERIALIZE_SERVICE = "deserialize_db";

  const std::string REGISTRATION_DESC[7] = {"Registration Successful", "Failed", "Already Registered",
      "Face Not Detected", "Face Not Clear", "Person Too Far", "Person Too Close"};
  const std::string RECOGNITION_DESC[8] = {"Recognition Successful", "Not Recognized", "Failed", "Face Not Detected",
      "Face Not Clear", "Person Too Far", "Person Too Close", "Face Ambiguity"};
  const std::string ORIENTATION_DESC[6] = {"frontal", "45_degree_right", "45_degree_left", "right", "left", "rear"};
  const std::string JOINT_TYPE_DESC[25] = {"joint_ankle_left", "joint_ankle_right", "joint_elbow_left",
      "joint_elbow_right", "joint_foot_left", "joint_foot_right", "joint_hand_left", "joint_hand_right",
      "joint_hand_tip_left", "joint_hand_tip_right", "joint_head", "joint_hip_left", "joint_hip_right",
      "joint_knee_left", "joint_knee_right", "joint_neck", "joint_shoulder_left", "joint_shoulder_right",
      "joint_spine_base", "joint_spine_mid", "joint_spine_shoulder", "joint_thumb_left", "joint_thumb_right",
      "joint_wrist_left", "joint_wrist_right"};

  const int32_t ORIENTATION_CONFIDENCE_THRESHOLD = 1; // TODO: Review the ideal threshold value
  const int32_t HEAD_BOX_CONFIDENCE_THRESHOLD = 50; // TODO: Review the ideal threshold value
  const int32_t LANDMARKS_CONFIDENCE_THRESHOLD = 50;  // TODO: Review the ideal threshold value
  const int32_t GESTURE_CONFIDENCE_THRESHOLD = 0; // TODO: Review the ideal threshold value
  const int32_t SKELETON_CONFIDENCE_THRESHOLD = 50; // TODO: Review the ideal threshold value

  const wchar_t* PERSON_MODULE_DATA_PATH = L"/usr/share/librealsense/pt/data/";
}
#endif
