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
from pytest_live_camera_utils import debug_print
def check_if_skip_test(profile, format):
    if profile == 'Color':
        if "BGRA8" == format:
            return True
        if "RGBA8" == format:
            return True
        if "Y8" == format:
            return True
    '''
    elif profile == 'Depth':
        if "Z16" == format:
            return True
    elif profile == 'Infrared':
        if "Y8" == format:
            return True
        if "Y16" == format:
            return True
        if "BGRA8" == format:
            return True
        if "RGBA8" == format:
            return True
        if "Y10BPACK" == format:
            return True
        if "UYVY" == format:
            return True
        if "BGR8" == format:
            return True
        if "RGB8" == format:
            return True
        if "RAW10" == format:
            return True
    elif profile == 'Infrared1':
        if "Y8" ==format:
            return True
        if "Y16" ==format:
            return True
        if "Y10BPACK"  == format:
            return True
        if "UYVY" ==format:
            return True
        if "BGR8" ==format:
            return True
        if "RGB8" ==format:
            return True
        if "RAW10" ==format:
            return True
    if profile == 'Infrared2':                    
        if "Y8" == format:
            return True
        if "Y16" == format:
            return True
        if "Y10BPACK" == format:
            return True
        if "UYVY" == format:
            return True
        if "BGR8" == format:
            return True
        if "RGB8" == format:
            return True
        if "RAW10" == format:
            return True
    '''
    return False


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
@pytest.mark.parametrize("launch_descr_with_parameters", [
    pytest.param(test_params_all_profiles_d455, marks=pytest.mark.d455),
    pytest.param(test_params_all_profiles_d415, marks=pytest.mark.d415),
    pytest.param(test_params_all_profiles_d435, marks=pytest.mark.d435),]
    ,indirect=True)
@pytest.mark.launch(fixture=launch_descr_with_parameters)
class TestLiveCamera_Change_Resolution(pytest_rs_utils.RsTestBaseClass):
    def test_LiveCamera_Change_Resolution(self,launch_descr_with_parameters):
        skipped_tests = []
        num_passed = 0
        num_failed = 0
        params = launch_descr_with_parameters[1]
        themes = [{'topic':'/'+params['camera_name']+'/color/image_raw', 'msg_type':msg_Image,'expected_data_chunks':1}]
        config = {
            "Color":{"profile":"rgb_camera.profile", "format":'rgb_camera.color_format', "param":"enable_color", "topic":'/'+params['camera_name']+'/color/image_raw',},
            "Depth":{"profile":"depth_module.profile", "format":'depth_module.depth_format', "param":"enable_depth", 'topic':'/'+params['camera_name']+'/depth/image_rect_raw'},
            "Infrared":{"profile":"depth_module.profile", "format":'depth_module.infra_format', "param":"enable_infra", 'topic':'/'+params['camera_name']+'/infra/image_rect_raw'},
            "Infrared1":{"profile":"depth_module.profile", "format":'depth_module.infra1_format',"param":"enable_infra1", 'topic':'/'+params['camera_name']+'/infra/image_rect_raw'},
            "Infrared2":{"profile":"depth_module.profile", "format":'depth_module.infra2_format',"param":"enable_infra2", 'topic':'/'+params['camera_name']+'/infra/image_rect_raw'},
        }
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
                debug_print("Device not found? : " + params['device_type'])
                return
            self.create_param_ifs(params['camera_name'] + '/' + params['camera_name'])
            config["Color"]["default_profile1"],config["Color"]["default_profile2"] = pytest_live_camera_utils.get_default_profiles(cap["color_profile"], "Color")
            config["Depth"]["default_profile1"],config["Depth"]["default_profile2"] = pytest_live_camera_utils.get_default_profiles(cap["depth_profile"], "Depth")
            config["Infrared"]["default_profile1"],config["Infrared"]["default_profile2"] = pytest_live_camera_utils.get_default_profiles(cap["depth_profile"], "Infrared")
            config["Infrared1"]["default_profile1"],config["Infrared1"]["default_profile2"] = pytest_live_camera_utils.get_default_profiles(cap["depth_profile"], "Infrared1")
            config["Infrared2"]["default_profile1"],config["Infrared2"]["default_profile2"] = pytest_live_camera_utils.get_default_profiles(cap["depth_profile"], "Infrared2")
            for key in cap["color_profile"]:
                profile_type = key[0]
                profile = key[1]
                format = key[2]
                if check_if_skip_test(profile_type, format):
                    skipped_tests.append(" ".join(key))
                    continue
                themes[0]['topic'] = config[profile_type]['topic']
                themes[0]['width'] = int(profile.split('x')[0])
                themes[0]['height'] = int(profile.split('x')[1])
                if themes[0]['width'] == int(config[profile_type]["default_profile2"].split('x')[0]):
                    self.set_string_param(config[profile_type]["profile"], config[profile_type]["default_profile1"])
                else:
                    self.set_string_param(config[profile_type]["profile"], config[profile_type]["default_profile2"])
                self.set_bool_param(config[profile_type]["param"], True)
                self.disable_all_params()
                self.set_string_param(config[profile_type]["profile"], profile)
                self.set_string_param(config[profile_type]["format"], format)
                self.set_bool_param(config[profile_type]["param"], True)
                ret = self.run_test(themes)
                assert ret[0], ret[1]
                assert self.process_data(themes), " ".join(key) + " failed"
            debug_print("Color tests completed")
            for key in cap["depth_profile"]:
                profile_type = key[0]
                profile = key[1]
                format = key[2]
                if check_if_skip_test(profile_type, format):
                    skipped_tests.append(" ".join(key))
                    continue
                debug_print("Testing " + " ".join(key))
                
                themes[0]['topic'] = config[profile_type]['topic']
                themes[0]['width'] = int(profile.split('x')[0])
                themes[0]['height'] = int(profile.split('x')[1])
                if themes[0]['width'] == int(config[profile_type]["default_profile2"].split('x')[0]):
                    self.set_string_param(config[profile_type]["profile"], config[profile_type]["default_profile1"])
                else:
                    self.set_string_param(config[profile_type]["profile"], config[profile_type]["default_profile2"])
                self.set_bool_param(config[profile_type]["param"], True)


                self.disable_all_params()
                self.set_string_param(config[profile_type]["profile"], profile)
                self.set_string_param(config[profile_type]["format"], format)
                self.set_bool_param(config[profile_type]["param"], True)
                try:
                    ret = self.run_test(themes)
                    assert ret[0], ret[1]
                    assert self.process_data(themes), " ".join(key) + " failed"
                    num_passed += 1
                except Exception as e:
                    print("Test failed")
                    print(e)
                    num_failed += 1
            debug_print("Depth tests completed")
        finally:
            #this step is important because the test will fail next time
            pytest_rs_utils.kill_realsense2_camera_node()
            self.shutdown()
            debug_print("\nSkipped tests:" + params['device_type'])
            debug_print("\n".join(skipped_tests))
            print("Tests passed " + str(num_passed))
            print("Tests skipped " + str(len(skipped_tests)))
            print("Tests failed " + str(num_failed))
    def disable_all_params(self):
        self.set_bool_param('enable_color', False)
        self.set_bool_param('enable_depth', False)
        self.set_bool_param('enable_infra', False)
        self.set_bool_param('enable_infra1', False)
        self.set_bool_param('enable_infra2', False)

