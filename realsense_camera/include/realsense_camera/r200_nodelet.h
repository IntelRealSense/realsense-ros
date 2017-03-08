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
#ifndef REALSENSE_CAMERA_R200_NODELET_H
#define REALSENSE_CAMERA_R200_NODELET_H

#include <string>
#include <vector>

#include <dynamic_reconfigure/server.h>

#include <realsense_camera/r200_paramsConfig.h>
#include <realsense_camera/sync_nodelet.h>

namespace realsense_camera
{
class R200Nodelet: public realsense_camera::SyncNodelet
{
public:
  void onInit();

protected:
  // Member Variables.
  rs_option edge_options_[4] =
  {
    RS_OPTION_R200_AUTO_EXPOSURE_LEFT_EDGE,
    RS_OPTION_R200_AUTO_EXPOSURE_TOP_EDGE,
    RS_OPTION_R200_AUTO_EXPOSURE_RIGHT_EDGE,
    RS_OPTION_R200_AUTO_EXPOSURE_BOTTOM_EDGE
  };
  boost::shared_ptr<dynamic_reconfigure::Server<realsense_camera::r200_paramsConfig>> dynamic_reconf_server_;

  rs_extrinsics color2ir2_extrinsic_;  // color frame is base frame

  // Member Functions.
  void getParameters();
  void advertiseTopics();
  std::vector<std::string> setDynamicReconfServer();
  void startDynamicReconfCallback();
  void setDynamicReconfigDepthControlPreset(int preset);
  std::string setDynamicReconfigDepthControlIndividuals();
  void configCallback(realsense_camera::r200_paramsConfig &config, uint32_t level);
  void getCameraExtrinsics();
  void publishStaticTransforms();
  void publishDynamicTransforms();
};
}  // namespace realsense_camera
#endif  // REALSENSE_CAMERA_R200_NODELET_H
