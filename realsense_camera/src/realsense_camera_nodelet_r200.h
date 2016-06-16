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
#ifndef REALSENSE_NODELET_R200
#define REALSENSE_NODELET_R200

#include <dynamic_reconfigure/server.h>

#include "realsense_camera/camera_params_r200Config.h"
#include "realsense_camera_nodelet.h"

namespace realsense_camera
{
  class RealsenseNodeletR200: public realsense_camera::RealsenseNodelet
  {
  public:

    // Initialize camera
    void onInit();

  protected:
    // R200 Constants.
    const rs_format IR2_FORMAT = RS_FORMAT_Y8;
    const std::string DEFAULT_IR2_FRAME_ID = "camera_infrared2_frame";
    const char *IR2_TOPIC = "camera/infrared2/image_raw";
    const int NUM_STREAMS_R200 = 4;

    // Member Variables.
    std::string ir2_frame_id_;
    boost::shared_ptr<dynamic_reconfigure::Server<realsense_camera::camera_params_r200Config>> dynamic_reconf_server_;

    // Member Functions.
    void enableDepthStream();
    void disableDepthStream();
    void enableInfrared2Stream();
    void disableInfrared2Stream();
    void setStreamOptions();
    void fillStreamEncoding();
    void allocateResources();
    void setStaticCameraOptions();
    void configCallback(realsense_camera::camera_params_r200Config &config, uint32_t level);
  };
}

#endif
