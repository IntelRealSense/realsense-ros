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
#ifndef NODELET_CONSTANTS
#define NODELET_CONSTANTS

namespace realsense_camera
{
    // Default Constants.
    const int STREAM_COUNT = 4;
    const int DEPTH_HEIGHT = 360;
    const int DEPTH_WIDTH = 480;
    const int COLOR_HEIGHT = 480;
    const int COLOR_WIDTH = 640;
    const int DEPTH_FPS = 60;
    const int COLOR_FPS = 60;
    const bool ENABLE_DEPTH = true;
    const bool ENABLE_COLOR = true;
    const bool ENABLE_PC = false;
    const bool ENABLE_TF = true;
    const rs_format DEPTH_FORMAT = RS_FORMAT_Z16;
    const rs_format COLOR_FORMAT = RS_FORMAT_RGB8;
    const rs_format IR_FORMAT = RS_FORMAT_Y8;
    const std::string DEFAULT_MODE = "preset";
    const std::string DEFAULT_BASE_FRAME_ID = "camera_link";
    const std::string DEFAULT_DEPTH_FRAME_ID = "camera_depth_frame";
    const std::string DEFAULT_COLOR_FRAME_ID = "camera_rgb_frame";
    const std::string DEFAULT_DEPTH_OPTICAL_FRAME_ID = "camera_depth_optical_frame";
    const std::string DEFAULT_COLOR_OPTICAL_FRAME_ID = "camera_rgb_optical_frame";
    const std::string DEFAULT_IR_FRAME_ID = "camera_ir_frame";
    const std::string DEFAULT_IR2_FRAME_ID = "camera_ir2_frame";
    const std::string DEPTH_TOPIC = "camera/depth/image_raw";
    const std::string COLOR_TOPIC = "camera/color/image_raw";
    const std::string IR_TOPIC = "camera/ir/image_raw";
    const std::string IR2_TOPIC = "camera/ir2/image_raw";
    const std::string PC_TOPIC = "camera/depth/points";
    const std::string SETTINGS_SERVICE = "camera/get_settings";
    const std::string STREAM_DESC[STREAM_COUNT] = {"Depth", "Color", "IR", "IR2"};
    const double ROTATION_IDENTITY[] = {1, 0, 0, 0, 1, 0, 0, 0, 1};

    // R200 Constants.
    const int R200_STREAM_COUNT = 4;
    const int R200_MAX_Z = 10;      // in meters
}
#endif
