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
'''
This test was implemented as a template to set the parameters and run the test.
This directory is not added to the CMakeLists.txt so as to avoid the colcon failure in the
machines that don't have the D455 connected.
1. Only a subset of parameter types are implemented in py_rs_utils, it has to be extended for others
2. After setting the param, rclpy.spin_once may be needed.Test passes even without this though.
'''
@pytest.mark.parametrize("launch_descr_with_parameters", [test_params_all_profiles_d455],indirect=True)
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
            #cap = pytest_live_camera_utils.get_camera_capabilities(params['device_type'], serial_no)
            cap = {}
            cap['color_profile'] = [
                ['Color', '1280x800x30', 'Y8'],
                #['Color', '1280x800x15', 'Y8'],
                #['Color', '1280x800x10', 'Y8'],
                #['Color', '1280x800x5', 'Y8'],
                #['Color', '1280x720x30', 'Y8'],
                #['Color', '1280x720x15', 'Y8'],
                #['Color', '1280x720x10', 'Y8'],
                #['Color', '1280x720x5', 'Y8'],
                #['Color', '848x480x60', 'Y8'],
                #['Color', '848x480x30', 'Y8'],
                #['Color', '848x480x15', 'Y8'],
                #['Color', '848x480x5', 'Y8'],
                #['Color', '640x480x60', 'Y8'],
                #['Color', '640x480x30', 'Y8'],
                #['Color', '640x480x15', 'Y8'],
                #['Color', '640x480x5', 'Y8'],
                #['Color', '640x360x90', 'Y8'],
                #['Color', '640x360x60', 'Y8'],
                #['Color', '640x360x30', 'Y8'],
                #['Color', '640x360x15', 'Y8'],
                #['Color', '640x360x5', 'Y8'],
                #['Color', '480x270x90', 'Y8'],
                #['Color', '480x270x60', 'Y8'],
                #['Color', '480x270x30', 'Y8'],
                #['Color', '480x270x15', 'Y8'],
                #['Color', '480x270x5', 'Y8'],
                #['Color', '424x240x90', 'Y8'],
                #['Color', '424x240x60', 'Y8'],
                #['Color', '424x240x30', 'Y8'],
                #['Color', '424x240x15', 'Y8'],
                #['Color', '424x240x5', 'Y8'],
                #Hanging tests start
                #['Color', '1280x800x30', 'BGRA8'],
                #['Color', '1280x800x30', 'RGBA8'],
                #['Color', '1280x800x15', 'BGRA8'],
                #['Color', '1280x800x15', 'RGBA8'],
                #['Color', '1280x800x10', 'BGRA8'],
                #['Color', '1280x800x10', 'RGBA8'],
                #['Color', '1280x800x5', 'BGRA8'],
                #['Color', '1280x800x5', 'RGBA8'],
                #['Color', '1280x720x30', 'BGRA8'],
                #['Color', '1280x720x30', 'RGBA8'],
                #['Color', '1280x720x15', 'BGRA8'],
                #['Color', '1280x720x15', 'RGBA8'],
                #['Color', '1280x720x10', 'BGRA8'],
                #['Color', '1280x720x10', 'RGBA8'],
                #['Color', '1280x720x5', 'BGRA8'],
                #['Color', '1280x720x5', 'RGBA8'],
                #['Color', '848x480x60', 'BGRA8'],
                #['Color', '848x480x60', 'RGBA8'],
                #['Color', '848x480x30', 'BGRA8'],
                #['Color', '848x480x30', 'RGBA8'],
                #['Color', '848x480x15', 'BGRA8'],
                #['Color', '848x480x15', 'RGBA8'],
                #['Color', '848x480x5', 'BGRA8'],
                #['Color', '848x480x5', 'RGBA8'],
                #['Color', '640x480x60', 'BGRA8'],
                #['Color', '640x480x60', 'RGBA8'],
                #['Color', '640x480x30', 'BGRA8'],
                #['Color', '640x480x30', 'RGBA8'],
                #['Color', '640x480x15', 'BGRA8'],
                #['Color', '640x480x15', 'RGBA8'],
                #['Color', '640x480x5', 'BGRA8'],
                #['Color', '640x480x5', 'RGBA8'],
                #['Color', '640x360x90', 'BGRA8'],
                #['Color', '640x360x90', 'RGBA8'],
                #['Color', '640x360x60', 'BGRA8'],
                #['Color', '640x360x60', 'RGBA8'],
                #['Color', '640x360x30', 'BGRA8'],
                #['Color', '640x360x30', 'RGBA8'],
                #['Color', '640x360x15', 'BGRA8'],
                #['Color', '640x360x15', 'RGBA8'],
                #['Color', '640x360x5', 'BGRA8'],
                #['Color', '640x360x5', 'RGBA8'],
                #['Color', '480x270x90', 'BGRA8'],
                #['Color', '480x270x90', 'RGBA8'],
                #['Color', '480x270x60', 'BGRA8'],
                #['Color', '480x270x60', 'RGBA8'],
                #['Color', '480x270x30', 'BGRA8'],
                #['Color', '480x270x30', 'RGBA8'],
                #['Color', '480x270x15', 'BGRA8'],
                #['Color', '480x270x15', 'RGBA8'],
                #['Color', '480x270x5', 'BGRA8'],
                #['Color', '480x270x5', 'RGBA8'],
                #['Color', '424x240x90', 'BGRA8'],
                #['Color', '424x240x90', 'RGBA8'],
                #['Color', '424x240x60', 'BGRA8'],
                #['Color', '424x240x60', 'RGBA8'],
                #['Color', '424x240x30', 'BGRA8'],
                #['Color', '424x240x30', 'RGBA8'],
                #['Color', '424x240x15', 'BGRA8'],
                #['Color', '424x240x15', 'RGBA8'],
                #['Color', '424x240x5', 'BGRA8'],
                #['Color', '424x240x5', 'RGBA8'],
                #Hanging tests end
                ]
            cap['depth_profile'] = [
                   ['Infrared', '1280x720x30', 'UYVY',],
                   #['Infrared', '1280x720x30', 'BGR8',],
                   #['Infrared', '1280x720x30', 'RGB8',],
                   #['Infrared', '1280x720x15', 'UYVY',],
                   #['Infrared', '1280x720x15', 'BGR8',],
                   #['Infrared', '1280x720x15', 'RGB8',],
                   #['Infrared', '1280x720x5', 'UYVY',],
                   #['Infrared', '1280x720x5', 'BGR8',],
                   #['Infrared', '1280x720x5', 'RGB8',],
                   #['Infrared', '848x480x90', 'UYVY',],
                   #['Infrared', '848x480x90', 'BGR8',],
                   #['Infrared', '848x480x90', 'RGB8',],
                   #['Infrared', '848x480x60', 'UYVY',],
                   #['Infrared', '848x480x60', 'BGR8',],
                   #['Infrared', '848x480x60', 'RGB8',],
                   #['Infrared', '848x480x30', 'UYVY',],
                   #['Infrared', '848x480x30', 'BGR8',],
                   #['Infrared', '848x480x30', 'RGB8',],
                   #['Infrared', '848x480x15', 'UYVY',],
                   #['Infrared', '848x480x15', 'BGR8',],
                   #['Infrared', '848x480x15', 'RGB8',],
                   #['Infrared', '848x480x5', 'UYVY',],
                   #['Infrared', '848x480x5', 'BGR8',],
                   #['Infrared', '848x480x5', 'RGB8',],
                   #['Infrared', '848x100x100', 'UYVY',],
                   #['Infrared', '848x100x100', 'BGR8',],
                   #['Infrared', '848x100x100', 'RGB8',],
                   #['Infrared', '640x480x90', 'UYVY',],
                   #['Infrared', '640x480x90', 'BGR8',],
                   #['Infrared', '640x480x90', 'RGB8',],
                   #['Infrared', '640x480x60', 'UYVY',],
                   #['Infrared', '640x480x60', 'BGR8',],
                   #['Infrared', '640x480x60', 'RGB8',],
                   #['Infrared', '640x480x30', 'UYVY',],
                   #['Infrared', '640x480x30', 'BGR8',],
                   #['Infrared', '640x480x30', 'RGB8',],
                   #['Infrared', '640x480x15', 'UYVY',],
                   #['Infrared', '640x480x15', 'BGR8',],
                   #['Infrared', '640x480x15', 'RGB8',],
                   #['Infrared', '640x480x5', 'UYVY',],
                   #['Infrared', '640x480x5', 'BGR8',],
                   #['Infrared', '640x480x5', 'RGB8',],
                   #['Infrared', '640x360x90', 'UYVY',],
                   #['Infrared', '640x360x90', 'BGR8',],
                   #['Infrared', '640x360x90', 'RGB8',],
                   #['Infrared', '640x360x60', 'UYVY',],
                   #['Infrared', '640x360x60', 'BGR8',],
                   #['Infrared', '640x360x60', 'RGB8',],
                   #['Infrared', '640x360x30', 'UYVY',],
                   #['Infrared', '640x360x30', 'BGR8',],
                   #['Infrared', '640x360x30', 'RGB8',],
                   #['Infrared', '640x360x15', 'UYVY',],
                   #['Infrared', '640x360x15', 'BGR8',],
                   #['Infrared', '640x360x15', 'RGB8',],
                   #['Infrared', '640x360x5', 'UYVY',],
                   #['Infrared', '640x360x5', 'BGR8',],
                   #['Infrared', '640x360x5', 'RGB8',],
                   #['Infrared', '480x270x90', 'UYVY',],
                   #['Infrared', '480x270x90', 'BGR8',],
                   #['Infrared', '480x270x90', 'RGB8',],
                   #['Infrared', '480x270x60', 'UYVY',],
                   #['Infrared', '480x270x60', 'BGR8',],
                   #['Infrared', '480x270x60', 'RGB8',],
                   #['Infrared', '480x270x30', 'UYVY',],
                   #['Infrared', '480x270x30', 'BGR8',],
                   #['Infrared', '480x270x30', 'RGB8',],
                   #['Infrared', '480x270x15', 'UYVY',],
                   #['Infrared', '480x270x15', 'BGR8',],
                   #['Infrared', '480x270x15', 'RGB8',],
                   #['Infrared', '480x270x5', 'UYVY',],
                   #['Infrared', '480x270x5', 'BGR8',],
                   #['Infrared', '480x270x5', 'RGB8',],
                   #['Infrared', '424x240x90', 'UYVY',],
                   #['Infrared', '424x240x90', 'BGR8',],
                   #['Infrared', '424x240x90', 'RGB8',],
                   #['Infrared', '424x240x60', 'UYVY',],
                   #['Infrared', '424x240x60', 'BGR8',],
                   #['Infrared', '424x240x60', 'RGB8',],
                   #['Infrared', '424x240x30', 'UYVY',],
                   #['Infrared', '424x240x30', 'RGB8',],
                   #['Infrared', '424x240x15', 'UYVY',],
                   #['Infrared', '424x240x15', 'BGR8',],
                   #['Infrared', '424x240x15', 'RGB8',],
                   #['Infrared', '424x240x5', 'UYVY',],
                   #['Infrared', '424x240x5', 'BGR8',],
                   #['Infrared', '424x240x5', 'RGB8',],
                   #['Infrared1', '1280x800x30', 'Y8',],
                   #['Infrared1', '1280x800x15', 'Y8',],
                   #['Infrared1', '1280x720x30', 'Y8',],
                   #['Infrared1', '1280x720x15', 'Y8',],
                   #['Infrared1', '1280x720x5', 'Y8',],
                   #['Infrared1', '848x480x90', 'Y8',],
                   #['Infrared1', '848x480x60', 'Y8'],
                   #['Infrared1', '848x480x30', 'Y8'],
                   #['Infrared1', '848x480x15', 'Y8'],
                   #['Infrared1', '848x480x5', 'Y8',],
                   #['Infrared1', '848x100x100', 'Y8',],
                   #['Infrared1', '640x480x90', 'Y8',],
                   #['Infrared1', '640x480x60', 'Y8',],
                   #['Infrared1', '640x480x30', 'Y8',],
                   #['Infrared1', '640x480x15', 'Y8',],
                   #['Infrared1', '640x480x5', 'Y8',],
                   #['Infrared1', '640x360x90', 'Y8',],
                   #['Infrared1', '640x360x60', 'Y8',],
                   #['Infrared1', '640x360x30', 'Y8',],
                   #['Infrared1', '640x360x15', 'Y8',],
                   #['Infrared1', '640x360x5', 'Y8',],
                   #['Infrared1', '480x270x90', 'Y8',],
                   #['Infrared1', '480x270x60', 'Y8',],
                   #['Infrared1', '480x270x30', 'Y8',],
                   #['Infrared1', '480x270x15', 'Y8',],
                   #['Infrared1', '480x270x5', 'Y8',],
                   #['Infrared1', '424x240x90', 'Y8',],
                   #['Infrared1', '424x240x60', 'Y8',],
                   #['Infrared1', '424x240x30', 'Y8',],
                   #['Infrared1', '424x240x15', 'Y8',],
                   #['Infrared1', '424x240x5', 'Y8',],
                   #['Infrared2', '1280x800x30', 'Y8',],
                   #['Infrared2', '1280x800x15', 'Y8',],
                   #['Infrared2', '1280x720x30', 'Y8',],
                   #['Infrared2', '1280x720x15', 'Y8',],
                   #['Infrared2', '1280x720x5', 'Y8',],
                   #['Infrared2', '848x480x90', 'Y8',],
                   #['Infrared2', '848x480x60', 'Y8',],
                   #['Infrared2', '848x480x30', 'Y8',],
                   #['Infrared2', '848x480x15', 'Y8',],
                   #['Infrared2', '848x480x5', 'Y8',],
                   #['Infrared2', '848x100x100', 'Y8',],
                   #['Infrared2', '640x480x90', 'Y8',],
                   #['Infrared2', '640x480x60', 'Y8',],
                   #['Infrared2', '640x480x30', 'Y8',],
                   #['Infrared2', '640x480x15', 'Y8',],
                   #['Infrared2', '640x480x5', 'Y8',],
                   #['Infrared2', '640x360x90', 'Y8',],
                   #['Infrared2', '640x360x60', 'Y8',],
                   #['Infrared2', '640x360x30', 'Y8',],
                   #['Infrared2', '640x360x15', 'Y8',],
                   #['Infrared2', '640x360x5', 'Y8',],
                   #['Infrared2', '480x270x90', 'Y8',],
                   #['Infrared2', '480x270x60', 'Y8',],
                   #['Infrared2', '480x270x30', 'Y8',],
                   #['Infrared2', '480x270x15', 'Y8',],
                   #['Infrared2', '480x270x5', 'Y8',],
                   #['Infrared2', '424x240x90', 'Y8',],
                   #['Infrared2', '424x240x60', 'Y8',],
                   #['Infrared2', '424x240x30', 'Y8',],
                   #['Infrared2', '424x240x15', 'Y8',],
                   #['Infrared2', '424x240x5', 'Y8',],
                   #['Depth','1280x720x30','Z16'],
                   #['Depth','1280x720x15','Z16'],
                   #['Depth','1280x720x5','Z16'],
                   #['Depth','848x480x90','Z16'],
                   #['Depth','848x480x60','Z16'],
                   #['Depth','848x480x30','Z16'],
                   #['Depth','848x480x15','Z16'],
                   #['Depth','848x480x5','Z16'],
                   #['Depth','848x100x10','Z16'],
                   #['Depth','640x480x90','Z16'],
                   #['Depth','640x480x60','Z16'],
                   #['Depth','640x480x30','Z16'],
                   #['Depth','640x480x15','Z16'],
                   #['Depth','640x480x5','Z16'],
                   #['Depth','640x360x90','Z16'],
                   #['Depth','640x360x60','Z16'],
                   #['Depth','640x360x30','Z16'],
                   #['Depth','640x360x15','Z16'],
                   #['Depth','640x360x5','Z16'],
                   #['Depth','480x270x90','Z16'],
                   #['Depth','480x270x60','Z16'],
                   #['Depth','480x270x30','Z16'],
                   #['Depth','480x270x15','Z16'],
                   #['Depth','480x270x5','Z16'],
                   #['Depth','424x240x90','Z16'],
                   #['Depth','424x240x60','Z16'],
                   #['Depth','424x240x30','Z16'],
                   #['Depth','424x240x15','Z16'],
                   #['Depth','424x240x5','Z16'],
                   #['Depth','256x144x90','Z16'],
                   ##Hanging tests start
                   #['Infrared', '1280x720x30', 'BGRA8',],
                   #['Infrared', '1280x720x30', 'RGBA8',],
                   #['Infrared', '1280x720x15', 'BGRA8',],
                   #['Infrared', '1280x720x15', 'RGBA8',],
                   #['Infrared', '1280x720x5', 'BGRA8',],
                   #['Infrared', '1280x720x5', 'RGBA8',],
                   #['Infrared', '848x480x90', 'BGRA8',],
                   #['Infrared', '848x480x90', 'RGBA8',],
                   #['Infrared', '848x480x60', 'BGRA8',],
                   #['Infrared', '848x480x60', 'RGBA8',],
                   #['Infrared', '848x480x30', 'BGRA8',],
                   #['Infrared', '848x480x30', 'RGBA8',],
                   #['Infrared', '848x480x15', 'BGRA8',],
                   #['Infrared', '848x480x15', 'RGBA8',],
                   #['Infrared', '848x480x5', 'BGRA8',],
                   #['Infrared', '848x480x5', 'RGBA8',],
                   #['Infrared', '848x100x100', 'BGRA8',],
                   #['Infrared', '848x100x100', 'RGBA8',],
                   #['Infrared', '640x480x90', 'BGRA8',],
                   #['Infrared', '640x480x90', 'RGBA8',],
                   #['Infrared', '640x480x60', 'BGRA8',],
                   #['Infrared', '640x480x60', 'RGBA8',],
                   #['Infrared', '640x480x30', 'BGRA8',],
                   #['Infrared', '640x480x30', 'RGBA8',],
                   #['Infrared', '640x480x15', 'BGRA8',],
                   #['Infrared', '640x480x15', 'RGBA8',],
                   #['Infrared', '640x480x5', 'BGRA8',],
                   #['Infrared', '640x480x5', 'RGBA8',],
                   #['Infrared', '640x360x90', 'BGRA8',],
                   #['Infrared', '640x360x90', 'RGBA8',],
                   #['Infrared', '640x360x60', 'BGRA8',],
                   #['Infrared', '640x360x60', 'RGBA8',],
                   #['Infrared', '640x360x30', 'BGRA8',],
                   #['Infrared', '640x360x30', 'RGBA8',],
                   #['Infrared', '640x360x15', 'BGRA8',],
                   #['Infrared', '640x360x15', 'RGBA8',],
                   #['Infrared', '640x360x5', 'BGRA8',],
                   #['Infrared', '640x360x5', 'RGBA8',],
                   #['Infrared', '480x270x90', 'BGRA8',],
                   #['Infrared', '480x270x90', 'RGBA8',],
                   #['Infrared', '480x270x60', 'BGRA8',],
                   #['Infrared', '480x270x60', 'RGBA8',],
                   #['Infrared', '480x270x30', 'BGRA8',],
                   #['Infrared', '480x270x30', 'RGBA8',],
                   #['Infrared', '480x270x15', 'BGRA8',],
                   #['Infrared', '480x270x15', 'RGBA8',],
                   #['Infrared', '480x270x5', 'BGRA8',],
                   #['Infrared', '480x270x5', 'RGBA8',],
                   #['Infrared', '424x240x90', 'BGRA8',],
                   #['Infrared', '424x240x90', 'RGBA8',],
                   #['Infrared', '424x240x60', 'BGRA8',],
                   #['Infrared', '424x240x60', 'RGBA8',],
                   #['Infrared', '424x240x30', 'BGRA8',],
                   #['Infrared', '424x240x30', 'RGBA8',],
                   #['Infrared', '424x240x30', 'BGR8',],
                   #['Infrared', '424x240x15', 'BGRA8',],
                   #['Infrared', '424x240x15', 'RGBA8',],
                   #['Infrared', '424x240x5', 'BGRA8',],
                   #['Infrared', '424x240x5', 'RGBA8',],
                   #['Infrared1', '1280x800x25', 'Y16',],
                   #['Infrared1', '1280x800x15', 'Y16',],
                   #['Infrared1', '640x400x25', 'Y16',],
                   #['Infrared1', '640x400x15', 'Y16',],
                   #['Infrared2', '1280x800x25', 'Y16',],
                   #['Infrared2', '1280x800x15', 'Y16',],
                   #['Infrared2', '640x400x25', 'Y16',],
                   #['Infrared2', '640x400x15', 'Y16',],
                   ##Hanging tests end
            ] 
            self.init_test("RsTest"+params['camera_name'])
            self.create_param_ifs(params['camera_name'] + '/' + params['camera_name'])
            num_passed = 0
            num_failed = 0
            failed_tests = []
            if 'color_profile' in cap:
                for profile in cap["color_profile"]:
                    if profile[0] == 'Color':
                        print("Testing :"+ profile[0] + " " + profile[1] + " " + profile[2])
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
                        if  ret[0]:
                            try:
                                assert self.process_data(themes)
                                num_passed += 1
                                print("test passed")
                            except Exception as e:
                                print("test failed in size check for " + profile[0] + " " + profile[1] + " " + profile[2] + " :")
                                print(e)
                                num_failed += 1
                                failed_tests.append(profile[0] + " " + profile[1] + " " + profile[2])
                        else:
                            print("test failed for " + profile[0] + " " + profile[1] + " " + profile[2] + " :" + ret[1])
                            num_failed += 1
                            failed_tests.append(profile[0] + " " + profile[1] + " " + profile[2])

            if 'depth_profile' in cap:
                for profile in cap["depth_profile"]:
                    self.set_bool_param('enable_color', False)
                    self.set_bool_param('enable_depth', False)
                    self.set_bool_param('enable_infra', False)
                    self.set_bool_param('enable_infra1', False)
                    self.set_bool_param('enable_infra2', False)
                    if profile[0] == 'Depth':
                        print("Testing :"+ profile[0] + " " + profile[1] + " " + profile[2])
                        self.set_string_param('depth_module.profile', profile[1])
                        self.set_string_param('depth_module.depth_format', profile[2])
                        self.set_bool_param('enable_depth', True)
                        depth_themes[0]['width'] = int(profile[1].split('x')[0])
                        depth_themes[0]['height'] = int(profile[1].split('x')[1])
                        ret = self.run_test(depth_themes)
                        if  ret[0]:
                            try:
                                assert self.process_data(depth_themes)
                                num_passed += 1
                                print("test passed")
                            except Exception as e:
                                print("test failed in size check for " + profile[0] + " " + profile[1] + " " + profile[2] + " :")
                                print(e)
                                num_failed += 1
                                failed_tests.append(profile[0] + " " + profile[1] + " " + profile[2])
                        else:
                            print("test failed for " + profile[0] + " " + profile[1] + " " + profile[2] + " :" + ret[1])
                            num_failed += 1
                            failed_tests.append(profile[0] + " " + profile[1] + " " + profile[2])


                    if profile[0] == 'Infrared':
                        print("Testing :"+ profile[0] + " " + profile[1] + " " + profile[2])
                        self.set_string_param('depth_module.profile', profile[1])
                        self.set_string_param('depth_module.infra_format', profile[2])
                        self.set_bool_param('enable_infra', True)
                        infra_themes[0]['width'] = int(profile[1].split('x')[0])
                        infra_themes[0]['height'] = int(profile[1].split('x')[1])
                        ret = self.run_test(infra_themes)
                        if  ret[0]:
                            try:
                                assert self.process_data(infra_themes)
                                num_passed += 1
                                print("test passed")
                            except Exception as e:
                                print("test failed in size check for " + profile[0] + " " + profile[1] + " " + profile[2] + " :")
                                print(e)
                                failed_tests.append(profile[0] + " " + profile[1] + " " + profile[2])
                                num_failed += 1
                        else:
                            print("test failed for " + profile[0] + " " + profile[1] + " " + profile[2] + " :" + ret[1])
                            num_failed += 1
                            failed_tests.append(profile[0] + " " + profile[1] + " " + profile[2])
                if profile[0] == 'Infrared1':
                        print("Testing :"+ profile[0] + " "  + profile[1] + " " + profile[2])
                        self.set_string_param('depth_module.profile', profile[1])
                        self.set_string_param('depth_module.infra1_format', profile[2])
                        self.set_bool_param('enable_infra1', True)
                        infra1_themes[0]['width'] = int(profile[1].split('x')[0])
                        infra1_themes[0]['height'] = int(profile[1].split('x')[1])
                        ret = self.run_test(infra1_themes)
                        if  ret[0]:
                            try:
                                assert self.process_data(infra1_themes)
                                num_passed += 1
                                print("test passed")
                            except Exception as e:
                                print("test failed in size check for " + profile[0] + " " + profile[1] + " " + profile[2] + " :")
                                print(e)
                                num_failed += 1
                                failed_tests.append(profile[0] + " " + profile[1] + " " + profile[2])
                        else:
                            print("test failed for " + profile[0] + " " + profile[1] + " " + profile[2] + " :" + ret[1])
                            num_failed += 1
                            failed_tests.append(profile[0] + " " + profile[1] + " " + profile[2])
                if profile[0] == 'Infrared2':                    
                        print("Testing :"+ profile[0] + " "  + profile[1] + " " + profile[2])
                        self.set_string_param('depth_module.profile', profile[1])
                        self.set_string_param('depth_module.infra2_format', profile[2])
                        self.set_bool_param('enable_infra2', True)
                        infra2_themes[0]['width'] = int(profile[1].split('x')[0])
                        infra2_themes[0]['height'] = int(profile[1].split('x')[1])
                        ret = self.run_test(infra2_themes)
                        if  ret[0]:
                            try:
                                assert self.process_data(infra2_themes)
                                num_passed += 1
                                print("test passed")
                            except Exception as e:
                                print("test failed in size check for " + profile[0] + " " + profile[1] + " " + profile[2] + " :")
                                print(e)
                                num_failed += 1
                                failed_tests.append(profile[0] + " " + profile[1] + " " + profile[2])
                        else:
                            print("test failed for " + profile[0] + " " + profile[1] + " " + profile[2] + " :" + ret[1])
                            num_failed += 1
                            failed_tests.append(profile[0] + " " + profile[1] + " " + profile[2])
        finally:
            #this step is important because the test will fail next time
            pytest_rs_utils.kill_realsense2_camera_node()
            if num_failed != 0:
                print("failed tests:")
                print("\n".join(failed_tests))
            else:
                print("All the tests are passing. If there are no commented out, the test skips in all_profiles test can be removed and remove this test")
            print("num_passed: " + str(num_passed) + " num_failed: " + str(num_failed))

            self.shutdown()

