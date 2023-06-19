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

import pytest

import numpy as np
from sensor_msgs.msg import Image as msg_Image
from sensor_msgs.msg import Imu as msg_Imu
from sensor_msgs.msg import PointCloud2 as msg_PointCloud2

from array import array
from builtin_interfaces.msg import Time
from sensor_msgs.msg import RegionOfInterest
from std_msgs.msg import Header
from sensor_msgs.msg import CameraInfo


sys.path.append(os.path.abspath(os.path.dirname(__file__)+"/../utils"))
import pytest_rs_utils
from pytest_rs_utils import launch_descr_with_parameters

from pytest_rs_utils import delayed_launch_descr_with_parameters


test_params_all_topics = {"rosbag_filename":os.getenv("ROSBAG_FILE_PATH")+"/outdoors_1color.bag",
    'camera_name': 'AllTopics',
    'color_width': '0',
    'color_height': '0',
    'depth_width': '0',
    'depth_height': '0',
    'infra_width': '0',
    'infra_height': '0',
    'enable_infra1':'true',
    'enable_infra2':'true',
    'align_depth.enable':'true',
    }
'''
To test all topics published
'''
@pytest.mark.rosbag
@pytest.mark.parametrize("delayed_launch_descr_with_parameters", [test_params_all_topics],indirect=True)
@pytest.mark.launch(fixture=delayed_launch_descr_with_parameters)
class TestAllTopics(pytest_rs_utils.RsTestBaseClass):
    def test_all_topics(self,delayed_launch_descr_with_parameters):
        ''' 
        current rosbag file doesn't have color data 
        '''
        params = delayed_launch_descr_with_parameters[1]
        self.rosbag = params["rosbag_filename"]
        color_data = CameraInfo(header=Header(stamp=Time(sec=1508282881, nanosec=33132324),
                                        frame_id="AllTopics_color_optical_frame"),
                                        width=640,
                                        height=480,
                                        distortion_model='plumb_bob', 
                                        d=[0.0, 0.0, 0.0, 0.0, 0.0], 
                                        k=[616.0769043,0.0,311.48977661,0.0,615.79931641,241.15310669,0.0,0.0,1.0], 
                                        r=[1.0,.0,0.0,0.0,1.0,0.0,0.0,0.0,1.0], 
                                        p=[616.0769043,0.0,311.48977661,0.0,0.0,615.79931641,241.15310669,0.0,0.0,0.0,1.0,0.0],
                                        binning_x=0,
                                        binning_y=0,
                                        roi=RegionOfInterest(x_offset=0, y_offset=0, height=0, width=0, do_rectify=False))
    
        depth_data = CameraInfo(header=Header(stamp=Time(sec=1508282880, nanosec=968727295), 
                                        frame_id='AllTopics_depth_optical_frame'), 
                                        height=720, 
                                        width=1280, 
                                        distortion_model='plumb_bob', 
                                        d=[0.0, 0.0, 0.0, 0.0, 0.0], 
                                        k=[976.7364502,0.0,636.62762451,0.0,976.7364502,373.01535034,0.0,0.0,1.0],
                                        r=[1., 0., 0., 0., 1., 0., 0., 0., 1.], 
                                        p=[976.7364502, 0., 636.62762451, 0., 0., 976.7364502, 373.01535034, 0., 0., 0., 1., 0.], 
                                        binning_x=0, 
                                        binning_y=0, 
                                        roi=RegionOfInterest(x_offset=0, y_offset=0, height=0, width=0, do_rectify=False))
        
        infra1_data =CameraInfo(header=Header(stamp=Time(sec=1508282880, nanosec=964985352), 
                                              frame_id='AllTopics_infra1_optical_frame'), 
                                              height=720, 
                                              width=1280, 
                                              distortion_model='plumb_bob', 
                                              d=[0.0, 0.0, 0.0, 0.0, 0.0], 
                                              k=[976.7364502, 0., 636.62762451, 0., 976.7364502 , 373.01535034, 0., 0., 1.], 
                                              r=[1., 0., 0., 0., 1., 0., 0., 0., 1.], 
                                              p=[976.7364502, 0., 636.62762451, 0., 0., 976.7364502, 373.01535034, 0., 0., 0., 1., 0.], 
                                              binning_x=0, 
                                              binning_y=0, 
                                              roi=RegionOfInterest(x_offset=0, y_offset=0, height=0, width=0, do_rectify=False))
        
        depth_to_color_data = CameraInfo(header=Header(stamp=Time(sec=1508282880, nanosec=968727295), 
                                                frame_id='AllTopics_color_optical_frame'), 
                                                height=480, 
                                                width=640, 
                                                distortion_model='plumb_bob', 
                                                d=[0.0, 0.0, 0.0, 0.0, 0.0], 
                                                k=[616.0769043, 0., 311.48977661, 0., 615.79931641, 241.15310669, 0., 0., 1.], 
                                                r=[1., 0., 0., 0., 1., 0., 0., 0., 1.], 
                                                p=[616.0769043, 0., 311.48977661, 0., 0., 615.79931641, 241.15310669, 0., 0., 0., 1., 0.], 
                                                binning_x=0, 
                                                binning_y=0, 
                                                roi=RegionOfInterest(x_offset=0, y_offset=0, height=0, width=0, do_rectify=False))


        themes = [
        {
         'topic':'/'+params['camera_name']+'/color/camera_info',
         'msg_type':CameraInfo,
         'expected_data_chunks':1,
         'data':color_data
        },
        {
         'topic':'/'+params['camera_name']+'/depth/camera_info',
         'msg_type':CameraInfo,
         'expected_data_chunks':1,
         'data':depth_data
        },
        {
         'topic':'/'+params['camera_name']+'/infra1/camera_info',
         'msg_type':CameraInfo,
         'expected_data_chunks':1,
         'data':infra1_data
        }
        ,
        {
         'topic':'/'+params['camera_name']+'/aligned_depth_to_color/camera_info',
         'msg_type':CameraInfo,
         'expected_data_chunks':1,
         'data':depth_to_color_data
        }
        ]
        try:
            ''' 
            initialize, run and check the data 
            '''
            self.init_test("RsTest"+params['camera_name'])
            assert self.run_test(themes)
            assert self.process_data(themes), "Data check failed, probably the rosbag file changed?"
        finally:
            self.shutdown()
    def process_data(self, themes):
        return super().process_data(themes)

