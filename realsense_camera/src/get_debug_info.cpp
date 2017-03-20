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

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <librealsense/rs.hpp>
#include <librealsense/rs.h>
#include <string>
#include <sstream>
#include <ros/package.h>
#include <sys/utsname.h>
#include <cstdlib>
#include <stdlib.h>
#include <array>

rs_context *rs_context_ = NULL;
rs_error *rs_error_ = NULL;

std::string getSystemCallOutput(const char* cmd)
{
  std::array<char, 128> buffer;
  std::string result;
  std::shared_ptr<FILE>pipe(popen(cmd, "r"), pclose);

  if (!pipe)
  {
    throw std::runtime_error("popen(\"" + std::string(cmd) + "\", \"r\") failed");
  }

  while (!feof(pipe.get()))
  {
    if (fgets(buffer.data(), 128, pipe.get()) != NULL)
    {
      result += buffer.data();
    }
  }

  std::string::size_type pos = result.find_last_not_of("\n \t");
  if (pos != std::string::npos)
  {
    result = result.substr(0, pos + 1);
  }
  return result;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "debug_tool");
  ros::NodeHandle n;
  std::string message = "\n----------CUT-PASTE-THIS----------\n";
  message = message + "|Version | Your Configuration | \n|:----------|:----------| \n";

  /* Acquire OS */
  message = message + "|Operating System | ";
  std::string operating_system;
  try
  {
    operating_system = getSystemCallOutput("lsb_release -ds");
    if (operating_system.empty())
    {
      operating_system = "Unable to detect operating system";
    }
  }
  catch(const std::exception& e)
  {
    operating_system = e.what();
  }
  message = message + operating_system + "|\n";

  /* Acquire Kernel Release */
  struct utsname* buf = new utsname();
  int uname_return = uname(buf);
  std::string kernel;
  if (uname_return == 0)
  {
    kernel = buf->release;
  }
  else
  {
    kernel = "Unable to detect Kernel";
  }
  message = message + "|Kernel | ";
  message = message + kernel + "|\n";

  /* Acquire ROS Version */
  message = message + "|ROS | ";
  std::string ros_version;
  char* ros_version_char = getenv("ROS_DISTRO");
  if (ros_version_char == NULL)
  {
    ros_version = "Unable to detect ROS version";
  }
  else
  {
    ros_version = ros_version_char;
  }
  message = message + ros_version + "| \n";

  /* Acquire ROS RealSense Version */
  message = message + "|ROS RealSense | ";
  std::string realsense_version;
  try
  {
    realsense_version = getSystemCallOutput("rosversion realsense_camera");
    if (realsense_version.empty())
    {
      realsense_version = "Unable to detect realsense version";
    }
  }
  catch(const std::exception& e)
  {
    realsense_version = e.what();
  }
  message = message + realsense_version + "|\n";

  /* Acquire librealsense version */
  message = message + "|librealsense | ";
  std::string librealsense_version;
  try
  {
    librealsense_version = getSystemCallOutput("rosversion librealsense");
    if (librealsense_version.empty())
    {
      librealsense_version = "Unable to detect librealsense version";
    }
  }
  catch(const std::exception& e)
  {
    librealsense_version = e.what();
  }
  message = message + librealsense_version + "|";

  /* Acquire detected cameras' types and firmwares */
  rs_context_ = rs_create_context(RS_API_VERSION, &rs_error_);

  int num_of_cameras = rs_get_device_count(rs_context_, &rs_error_);

  if (rs_error_)
  {
    message = message + "\n|Camera Type-Firmware | Error detecting cameras |";
    rs_error_ = NULL;
  }
  else
  {
    if (num_of_cameras == 0)
    {
      message = message + "\n|Camera Type-Firmware | No Cameras Detected |";
    }

    for (int i = 0; i < num_of_cameras; i++)
    {
      rs_device* rs_detected_device = rs_get_device(rs_context_, i, &rs_error_);
      if (rs_error_)
      {
        message = message + "\n|Camera Type-Firmware | Error detecting camera|";
        rs_error_ = NULL;
      }
      else
      {
        std::string camera_name = rs_get_device_name(rs_detected_device, &rs_error_);
        if (rs_error_)
        {
          camera_name = "Error detecting camera name";
          rs_error_ = NULL;
        }
        std::string camera_fw = rs_get_device_firmware_version(rs_detected_device, &rs_error_);
        if (rs_error_)
        {
          camera_fw = "Error detecting camera firmware";
          rs_error_ = NULL;
        }
        message = message + "\n|Camera Type | " + camera_name + "|\n|Camera Firmware | " + camera_fw + "|";
        if (rs_supports(rs_detected_device, RS_CAPABILITIES_ADAPTER_BOARD, &rs_error_))
        {
          std::string adapter_fw = rs_get_device_info(rs_detected_device,
              RS_CAMERA_INFO_ADAPTER_BOARD_FIRMWARE_VERSION, &rs_error_);
          if (rs_error_)
          {
            adapter_fw = "Error detecting adapter firmware";
            rs_error_ = NULL;
          }
          message = message + "\n|Adapter Firmware | " + adapter_fw + "|";
        }
        if (rs_supports(rs_detected_device, RS_CAPABILITIES_MOTION_EVENTS, &rs_error_))
        {
          std::string motion_module_fw = rs_get_device_info(rs_detected_device,
              RS_CAMERA_INFO_MOTION_MODULE_FIRMWARE_VERSION, &rs_error_);
          if (rs_error_)
          {
            motion_module_fw = "Error detecting motion module firmware";
            rs_error_ = NULL;
          }
          message = message + "\n|Motion Module Firmware | " + motion_module_fw + "|";
        }
      }
    }
  }
  message = message + "\n----------CUT-PASTE-THIS----------";

  ROS_INFO_STREAM(message);

  return 0;
}
