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
    '''
    if profile == 'Color':
        if "BGRA8" == format:
            return True
        if "RGBA8" == format:
            return True
        if "Y8" == format:
            return True
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
    'enable_accel':True,
    'enable_gyro':True,
    'unite_imu_method':1,
    }

@pytest.mark.parametrize("launch_descr_with_parameters", [
    pytest.param(test_params_all_profiles_d455, marks=pytest.mark.d455)
    ],indirect=True)
@pytest.mark.launch(fixture=launch_descr_with_parameters)
class TestLiveCamera_TestMotionSensor(pytest_rs_utils.RsTestBaseClass):
    def test_LiveCamera_check_motion_sensor(self,launch_descr_with_parameters):
        skipped_tests = []
        num_passed = 0
        num_failed = 0
        params = launch_descr_with_parameters[1]
        themes = [{'topic':'/'+params['camera_name']+'/imu', 'msg_type':msg_Imu,'expected_data_chunks':1},
                {'topic':'/'+params['camera_name']+'/gyro/sample', 'msg_type':msg_Imu,'expected_data_chunks':1},
                {'topic':'/'+params['camera_name']+'/accel/sample', 'msg_type':msg_Imu,'expected_data_chunks':1}]
        IMU_TOPIC = 0
        GYRO_TOPIC = 1
        ACCEL_TOPIC = 2
        if params['unite_imu_method'] == '0':
            themes[IMU_TOPIC]['expected_data_chunks'] = 0
        try:
            ''' 
            initialize, run and check the data 
            '''
            msg = "Test with the default params "
            self.init_test("RsTest"+params['camera_name'])
            self.create_param_ifs(params['camera_name'] + '/' + params['camera_name'])
            print(msg)
            ret = self.run_test(themes)
            assert ret[0], msg + ret[1]
            assert self.process_data(themes), msg + " failed"

            
            msg = "Test with the accel false "
            self.set_bool_param('enable_accel', False)
            self.set_bool_param('enable_gyro', True)
            self.set_integer_param('unite_imu_method', 0) #this shouldn't matter because the unite_imu_method cannot be changed
            themes[ACCEL_TOPIC]['expected_data_chunks'] = 0

            print(msg)
            ret = self.run_test(themes)
            assert ret[0], msg + ret[1]
            assert self.process_data(themes), msg + " failed"

            
            msg = "Test with both gyro and accel false "
            self.set_bool_param('enable_accel', False)
            self.set_bool_param('enable_gyro', False)
            themes[ACCEL_TOPIC]['expected_data_chunks'] = 0
            themes[GYRO_TOPIC]['expected_data_chunks'] = 0
            themes[IMU_TOPIC]['expected_data_chunks'] = 0
            print(msg)
            ret = self.run_test(themes, initial_wait_time=1.0) #wait_time added as test doesn't have to wait for any data
            assert ret[0], msg + " failed"
            assert self.process_data(themes), msg 

            msg =  "Test with the gyro false "
            self.set_bool_param('enable_accel', True)
            self.set_bool_param('enable_gyro', False)
            self.set_integer_param('unite_imu_method', 1)
            themes[ACCEL_TOPIC]['expected_data_chunks'] = 5
            themes[GYRO_TOPIC]['expected_data_chunks'] = 0
            print(msg)
            ret = self.run_test(themes)
            assert ret[0], msg + ret[1]
            assert self.process_data(themes), msg + " failed"
            
        finally:
            #this step is important because the test will fail next time
            pytest_rs_utils.kill_realsense2_camera_node()
            self.shutdown()
