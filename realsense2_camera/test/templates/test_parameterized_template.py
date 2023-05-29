# Copyright 2023 Intel Corporation. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


import os
import sys
import subprocess
import time

import pytest
from launch import LaunchDescription
import launch_ros.actions
import launch_pytest
import rclpy
from rclpy import qos
from rclpy.node import Node
from sensor_msgs.msg import Image as msg_Image
from sensor_msgs.msg import Imu as msg_Imu

sys.path.append(os.path.abspath(os.path.dirname(__file__)+"/../utils"))
import pytest_rs_utils
from pytest_rs_utils import launch_descr_with_yaml
from pytest_rs_utils import launch_descr_with_parameters



test_params = {"rosbag_filename":os.getenv("ROSBAG_FILE_PATH")+"/outdoors_1color.bag",
    'color_width': '0',
    'color_height': '0',
    'depth_width': '0',
    'depth_height': '0',
    'infra_width': '0',
    'infra_height': '0',
    }

test_params1 = {"rosbag_filename":os.getenv("ROSBAG_FILE_PATH")+"/D435i_Depth_and_IMU_Stands_still.bag",
    'color_width': '0',
    'color_height': '0',
    'depth_width': '0',
    'depth_height': '0',
    'infra_width': '0',
    'infra_height': '0',
    }
@pytest.mark.rosbag
@pytest.mark.parametrize("launch_descr_with_parameters", [test_params,test_params1],indirect=True)
@pytest.mark.launch(fixture=launch_descr_with_parameters)
class TestCamera2(pytest_rs_utils.RsTestBaseClass):
    def test_node_start(self, launch_descr_with_parameters):
        print(launch_descr_with_parameters[1])
        themes = [
            {'topic':'/camera/depth/image_rect_raw',
                'msg_type':msg_Image,
                'expected_data_chunks':1, 
                'frame_id':'camera_depth_optical_frame',
            },
        ]
        ''' # TODO: find a rosbag file that has accel/sample to test this
                    {'topic': '/camera/accel/sample', 
                        'msg_type': msg_Imu,
                        'expected_data_chunks':1, 
                        },
        '''
        try:
            self.init_test()
            assert self.run_test(themes)
            assert self.process_data(themes)
        finally:
            self.shutdown()

    ''' 
    override the process_data and check if the data is correct or not 
    '''
    def process_data(self, themes):
        for theme in themes:
            data = self.node.pop_first_chunk(theme['topic'])
            #message format can be found at /opt/ros/humble/share/sensor_msgs/msg/Image.msg
            print(data.header)
            if (data.header.frame_id!=theme['frame_id']):
                return False
        return True
