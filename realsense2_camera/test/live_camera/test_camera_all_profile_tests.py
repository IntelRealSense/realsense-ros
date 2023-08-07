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

test_params_all_profiles_d455 = {
    'camera_name': 'D455',
    'device_type': 'D455',
    }
test_params_all_profiles_d415 = {
    'camera_name': 'D415',
    'device_type': 'D415',
    }
test_params_all_profiles_d435 = {
    'camera_name': 'D435',
    'device_type': 'D435',
    }
'''
This test was implemented as a template to set the parameters and run the test.
This directory is not added to the CMakeLists.txt so as to avoid the colcon failure in the
machines that don't have the D455 connected.
1. Only a subset of parameter types are implemented in py_rs_utils, it has to be extended for others
2. After setting the param, rclpy.spin_once may be needed.Test passes even without this though.
'''
@pytest.mark.parametrize("launch_descr_with_parameters", [test_params_all_profiles_d435, test_params_all_profiles_d415,test_params_all_profiles_d455],indirect=True)
@pytest.mark.launch(fixture=launch_descr_with_parameters)
class TestLiveCamera_Change_Resolution(pytest_rs_utils.RsTestBaseClass):
    def test_LiveCamera_Change_Resolution(self,launch_descr_with_parameters):
        skipped_tests = []
        params = launch_descr_with_parameters[1]
        themes = [
        {'topic':'/'+params['camera_name']+'/color/image_raw',
         'msg_type':msg_Image,
         'expected_data_chunks':1,
         #'data':data
        }
        ]
        depth_themes = [
        {'topic':'/'+params['camera_name']+'/depth/image_rect_raw',
         'msg_type':msg_Image,
         'expected_data_chunks':1,
         #'data':data
        }
        ]  
        infra_themes = [
        {'topic':'/'+params['camera_name']+'/infra/image_rect_raw',
         'msg_type':msg_Image,
         'expected_data_chunks':1,
         #'data':data
        }
        ]        
        infra1_themes = [
        {'topic':'/'+params['camera_name']+'/infra1/image_rect_raw',
         'msg_type':msg_Image,
         'expected_data_chunks':1,
         #'data':data
        }
        ]   
        infra2_themes = [
        {'topic':'/'+params['camera_name']+'/infra2/image_rect_raw',
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
            self.init_test("RsTest"+params['camera_name'])
            cap = pytest_live_camera_utils.get_camera_capabilities(params['device_type'], serial_no)
            if cap == None:
                print("Device not found? : " + params['device_type'])
                return
            self.create_param_ifs(params['camera_name'] + '/' + params['camera_name'])
            for profile in cap["color_profile"]:
                if profile[0] == 'Color':
                    print("Testing :"+ profile[0] + " " + profile[1] + " " + profile[2])
                    if "BGRA8" == profile[2]:
                        print("Skipping " +profile[2])
                        skipped_tests.append(profile[0] + " " + profile[1] + " " + profile[2])
                        continue
                    if "RGBA8" == profile[2]:
                        skipped_tests.append(profile[0] + " " + profile[1] + " " + profile[2])
                        print("Skipping " +profile[2])
                        continue
                    if "Y8" == profile[2]:
                        skipped_tests.append(profile[0] + " " + profile[1] + " " + profile[2])
                        print("Skipping " +profile[2])
                        continue
                    self.set_bool_param('enable_color', False)
                    self.set_bool_param('enable_depth', False)
                    self.set_bool_param('enable_infra', False)
                    self.set_bool_param('enable_infra1', False)
                    self.set_bool_param('enable_infra2', False)
                    self.set_string_param('rgb_camera.profile', profile[1])
                    self.set_string_param('rgb_camera.color_format', profile[2])
                    self.set_bool_param('enable_color', True)
                    themes[0]['width'] = int(profile[1].split('x')[0])
                    themes[0]['height'] = int(profile[1].split('x')[1])
                    ret = self.run_test(themes)
                    assert ret[0], ret[1]
                    assert self.process_data(themes)
            for profile in cap["depth_profile"]:
                self.set_bool_param('enable_color', False)
                self.set_bool_param('enable_depth', False)
                self.set_bool_param('enable_infra', False)
                self.set_bool_param('enable_infra1', False)
                self.set_bool_param('enable_infra2', False)
                if profile[0] == 'Depth':
                    print("Testing :"+ profile[0] + " " + profile[1] + " " + profile[2])
                    if "Z16" == profile[2]:
                        skipped_tests.append(profile[0] + " " + profile[1] + " " + profile[2])
                        print("Skipping " +profile[2])
                        continue
                    self.set_string_param('depth_module.profile', profile[1])
                    self.set_string_param('depth_module.depth_format', profile[2])
                    self.set_bool_param('enable_depth', True)
                    depth_themes[0]['width'] = int(profile[1].split('x')[0])
                    depth_themes[0]['height'] = int(profile[1].split('x')[1])
                    ret = self.run_test(depth_themes)
                    assert ret[0], ret[1]
                    assert self.process_data(depth_themes)


                if profile[0] == 'Infrared':
                    print("Testing :"+ profile[0] + " " + profile[1] + " " + profile[2])
                    if "Y8" == profile[2]:
                        skipped_tests.append( profile[0] + " " + profile[1] + " " + profile[2])
                        print("Skipping " +profile[2])
                        continue
                    if "Y16" == profile[2]:
                        skipped_tests.append(profile[0] + " " + profile[1] + " " + profile[2])
                        print("Skipping " +profile[2])
                        continue
                    if "BGRA8" == profile[2]:
                        skipped_tests.append(profile[0] + " " + profile[1] + " " + profile[2])
                        print("Skipping " +profile[2])
                        continue
                    if "RGBA8" == profile[2]:
                        skipped_tests.append(profile[0] + " " + profile[1] + " " + profile[2])
                        print("Skipping " +profile[2])
                        continue
                    if "Y10BPACK" == profile[2]:
                        skipped_tests.append(profile[0] + " " + profile[1] + " " + profile[2])
                        print("Skipping " +profile[2])
                        continue
                    if "UYVY" == profile[2]:
                        skipped_tests.append(profile[0] + " " + profile[1] + " " + profile[2])
                        print("Skipping " +profile[2])
                        continue
                    if "BGR8" == profile[2]:
                        print("Skipping " +profile[2])
                        skipped_tests.append(profile[0] + " " + profile[1] + " " + profile[2])
                        continue
                    if "RGB8" == profile[2]:
                        print("Skipping " +profile[2])
                        skipped_tests.append(profile[0] + " " + profile[1] + " " + profile[2])
                        continue
                    if "RAW10" == profile[2]:
                        print("Skipping " +profile[2])
                        skipped_tests.append(profile[0] + " " + profile[1] + " " + profile[2])
                        continue
                    self.set_string_param('depth_module.profile', profile[1])
                    self.set_string_param('depth_module.infra_format', profile[2])
                    self.set_bool_param('enable_infra', True)
                    infra_themes[0]['width'] = int(profile[1].split('x')[0])
                    infra_themes[0]['height'] = int(profile[1].split('x')[1])
                    ret = self.run_test(infra_themes)
                    assert ret[0], ret[1]
                    assert self.process_data(infra_themes)
                if profile[0] == 'Infrared1':
                    print("Testing :"+ profile[0] + " "  + profile[1] + " " + profile[2])
                    if "Y8" == profile[2]:                    
                        skipped_tests.append(profile[0] + " " + profile[1] + " " + profile[2])
                        print("Skipping " +profile[2])
                        continue
                    if "Y16" == profile[2]:
                        skipped_tests.append(profile[0] + " " + profile[1] + " " + profile[2])
                        print("Skipping " +profile[2])
                        continue
                    if "Y10BPACK" == profile[2]:
                        skipped_tests.append(profile[0] + " " + profile[1] + " " + profile[2])
                        print("Skipping " +profile[2])
                        continue
                    if "UYVY" == profile[2]:
                        skipped_tests.append(profile[0] + " " + profile[1] + " " + profile[2])
                        print("Skipping " +profile[2])
                        continue
                    if "BGR8" == profile[2]:
                        skipped_tests.append(profile[0] + " " + profile[1] + " " + profile[2])
                        print("Skipping " +profile[2])
                        continue
                    if "RGB8" == profile[2]:
                        skipped_tests.append(profile[0] + " " + profile[1] + " " + profile[2])
                        print("Skipping " +profile[2])
                        continue
                    if "RAW10" == profile[2]:
                        skipped_tests.append(profile[0] + " " + profile[1] + " " + profile[2])
                        print("Skipping " +profile[2])
                        continue
                    self.set_string_param('depth_module.profile', profile[1])
                    self.set_string_param('depth_module.infra1_format', profile[2])
                    self.set_bool_param('enable_infra1', True)
                    infra1_themes[0]['width'] = int(profile[1].split('x')[0])
                    infra1_themes[0]['height'] = int(profile[1].split('x')[1])
                    ret = self.run_test(infra1_themes)
                    assert ret[0], ret[1]
                    assert self.process_data(infra1_themes)
                if profile[0] == 'Infrared2':                    
                    if "Y8" == profile[2]:
                        skipped_tests.append(profile[0] + " " + profile[1] + " " + profile[2])
                        print("Skipping " +profile[2])
                        continue
                    if "Y16" == profile[2]:
                        skipped_tests.append(profile[0] + " " + profile[1] + " " + profile[2])
                        print("Skipping " +profile[2])
                        continue
                    if "Y10BPACK" == profile[2]:
                        skipped_tests.append(profile[0] + " " + profile[1] + " " + profile[2])
                        print("Skipping " +profile[2])
                        continue
                    if "UYVY" == profile[2]:
                        skipped_tests.append(profile[0] + " " + profile[1] + " " + profile[2])
                        print("Skipping " +profile[2])
                        continue
                    if "BGR8" == profile[2]:
                        skipped_tests.append(profile[0] + " " + profile[1] + " " + profile[2])
                        print("Skipping " +profile[2])
                        continue
                    if "RGB8" == profile[2]:
                        skipped_tests.append(profile[0] + " " + profile[1] + " " + profile[2])
                        print("Skipping " +profile[2])
                        continue
                    if "RAW10" == profile[2]:
                        skipped_tests.append(profile[0] + " " + profile[1] + " " + profile[2])
                        print("Skipping " +profile[2])
                        continue
                    self.set_string_param('depth_module.profile', profile[1])
                    self.set_string_param('depth_module.infra2_format', profile[2])
                    self.set_bool_param('enable_infra2', True)
                    infra2_themes[0]['width'] = int(profile[1].split('x')[0])
                    infra2_themes[0]['height'] = int(profile[1].split('x')[1])
                    ret = self.run_test(infra2_themes)
                    assert ret[0], ret[1]
                    assert self.process_data(infra2_themes)
        finally:
            #this step is important because the test will fail next time
            pytest_rs_utils.kill_realsense2_camera_node()
            self.shutdown()
            print("\nSkipped tests:" + params['device_type'])
            print("\n".join(skipped_tests))


