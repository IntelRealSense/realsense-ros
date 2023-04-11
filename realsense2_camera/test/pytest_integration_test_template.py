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

import pytest_rs_utils
from pytest_rs_utils import launch_descr_with_yaml


''' 
This is a testcase simiar to the integration_fn testcase, the only difference is that
this one uses the launch configuration to launch the nodes.  
'''

@pytest.mark.launch(fixture=pytest_rs_utils.launch_descr_with_yaml)
def test_using_function(launch_context):
    time.sleep(0.1)
    # by now, the camera would have started
    start = time.time()
    timeout = 1.0
    while time.time() - start < timeout:
        service_list = subprocess.check_output(['ros2', 'node', 'list']).decode("utf-8")
        is_node_up = '/camera/camera' in service_list
        if is_node_up == True:
            break
        else:
            print(service_list)
        time.sleep(timeout/5)

    assert is_node_up, 'Node is NOT UP'
    print ('Node is UP')
    print ('*'*8 + ' Killing ROS ' + '*'*9)
    yield
    pytest_rs_utils.kill_realsense2_camera_node()
    time.sleep(0.1)
    assert True


''' 
This is another test that can be used as a template. The expectation is that most of the
integration tests will use this format where the tester can create a test node to subscribe
to different published topics and decide whether the test passed or failed.  
'''

'''
use the launch description from the utils and also inherit from basic test class RsTestBaseClass
'''
@pytest.mark.launch(fixture=pytest_rs_utils.launch_descr_with_yaml)
class TestCamera1(pytest_rs_utils.RsTestBaseClass):
    def test_node_start(self):
        ''' 
        current rosbag file doesn't have color data 
        '''
        themes = [
            #{'topic':'/camera/color/image_raw','type':msg_Image,'expected_data_chunks':1},
        {'topic':'/camera/depth/image_rect_raw','type':msg_Image,'expected_data_chunks':1}
        ]
        try:
            ''' 
            initialize, run and check the data 
            '''
            self.init_test()
            assert self.run_test(themes)
            assert self.process_data(themes)
        finally:
            self.shutdown()
    def process_data(self, themes):
        return super().process_data(themes)

@pytest.mark.launch(fixture=pytest_rs_utils.launch_descr_with_yaml)
class TestCamera2(pytest_rs_utils.RsTestBaseClass):
    def test_node_start(self):
        themes = [
            {'topic':'/camera/depth/image_rect_raw',
                'type':msg_Image,
                'expected_data_chunks':1, 
                'frame_id':'camera_depth_optical_frame',
                'height':720,
                'width':1280},
            {'topic':'/camera/color/image_raw',
                'type':msg_Image,
                'expected_data_chunks':1, 
                'frame_id':'camera_color_optical_frame',
                'height':480,
                'width':640},
        ]
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
            if (data.header.frame_id==theme['frame_id']) and (data.height == theme['height']) and (data.width == theme['width']):
                return True
            else:
                return False