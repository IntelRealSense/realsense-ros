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
import itertools


import pytest
import rclpy

from sensor_msgs.msg import Image as msg_Image
from sensor_msgs.msg import Imu as msg_Imu
from sensor_msgs.msg import PointCloud2 as msg_PointCloud2

import numpy as np

sys.path.append(os.path.abspath(os.path.dirname(__file__)+"/../utils"))
import pytest_rs_utils
from pytest_rs_utils import launch_descr_with_parameters
from pytest_rs_utils import delayed_launch_descr_with_parameters
from pytest_rs_utils import get_rosbag_file_path
import pytest_live_camera_utils
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterValue
from rcl_interfaces.srv import SetParameters, GetParameters, ListParameters

test_params_all_profiles = {
    'camera_name': 'D455',
    'device_type': 'D455',
    }
'''
This test was implemented as a template to set the parameters and run the test.
This directory is not added to the CMakeLists.txt so as to avoid the colcon failure in the
machines that don't have the D455 connected.
1. Only a subset of parameter types are implemented in py_rs_utils, it has to be extended for others
2. After setting the param, rclpy.spin_once may be needed.Test passes even without this though.
'''
@pytest.mark.parametrize("launch_descr_with_parameters", [test_params_all_profiles],indirect=True)
@pytest.mark.launch(fixture=launch_descr_with_parameters)
class TestD455_Change_Resolution(pytest_rs_utils.RsTestBaseClass):
    def test_D455_Change_Resolution(self,launch_descr_with_parameters):
        params = launch_descr_with_parameters[1]
        themes = [
        {'topic':'/'+params['camera_name']+'/color/image_raw',
         'msg_type':msg_Image,
         'expected_data_chunks':1,
         #'data':data
        }
        ]
        try:
            ''' 
            initialize, run and check the data 
            '''
            serial_no = None
            if 'serial_no' in params:
                serial_no = params['serial_no']
            cap = pytest_live_camera_utils.get_camera_capabilities(params['device_type'], serial_no)
            self.init_test("RsTest"+params['camera_name'])
            self.create_param_ifs(params['camera_name'] + '/' + params['camera_name'])
            for profile in cap["color_profile"]:
                if profile[0] == 'Color':
                    print("Testing :" + profile[1] + " " + profile[2])
                    if "BGRA8" == profile[2]:
                        print("Skipping BGRA8" )
                        continue
                    if "RGBA8" == profile[2]:
                        print("Skipping RGBA8" )
                        continue
                    if "Y8" == profile[2]:
                        print("Skipping Y8" )
                        continue
                    self.set_string_param('rgb_camera.profile', profile[1])
                    self.set_string_param('rgb_camera.color_format', profile[2])
                    self.set_bool_param('enable_color', True)
                    themes[0]['width'] = int(profile[1].split('x')[0])
                    themes[0]['height'] = int(profile[1].split('x')[1])
                    ret = self.run_test(themes)
                    assert ret[0], ret[1]
                    assert self.process_data(themes)

            ret = self.run_test(themes)
            assert self.process_data(themes)
        finally:
            #this step is important because the test will fail next time
            pytest_rs_utils.kill_realsense2_camera_node()
            self.shutdown()


