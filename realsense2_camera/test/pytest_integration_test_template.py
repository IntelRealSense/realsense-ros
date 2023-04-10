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
    # by now, the camera would have started.
    service_list = subprocess.check_output(['ros2', 'node', 'list']).decode("utf-8")
    is_node_up = '/camera/camera' in service_list
    print(service_list)
    assert is_node_up, 'Node is NOT UP'
    time.sleep(5)
    print ('Node is UP')
    print ('*'*8 + ' Killing ROS ' + '*'*9)
    pytest_rs_utils.kill_realsense2_camera_node()
    yield
    assert True


''' 
This is another test that can be used as a template. The expectation is that most of the
integration tests will use this format where the tester can create a test node to subscribe
to different published topics and decide whether the test passed or failed.  
'''

@pytest.mark.launch(fixture=pytest_rs_utils.launch_descr_with_yaml)
class TestFixture1():
    def test_node_start(self):
        rclpy.init()
        self.flag = False
        try:
            node = MakeTestNode('test_node')
            #node.create_subscription(msg_Image, '/camera/depth/image_rect_raw', node.static_tf_callback , qos.qos_profile_sensor_data)
            node.create_subscription(msg_Image, '/camera/color/image_raw', node.static_tf_callback , qos.qos_profile_sensor_data)
            print('subscription created... ' )
            
            start = time.time()
            timeout = 1.0
            print('Waiting for topic... ' )
            while time.time() - start < timeout:
                print('Spinning... ' )
                rclpy.spin_once(node)
                if node.flag:
                    break
            assert node.flag
            print('Test Passed... ' )
        finally:
            rclpy.shutdown()

''' 
This is that holds the test node that listens to a subscription created by a test.  
'''
class MakeTestNode(Node):
    def __init__(self, name='test_node'):
        print('\nCreating node... ' + name)
        super().__init__(name)

    def wait_for_node(self, node_name, timeout=8.0):
        start = time.time()
        flag = False
        self.flag = False
        print('Waiting for node... ' + node_name)
        while time.time() - start < timeout:
            flag = node_name in self.get_node_names()
            print(self.get_node_names())
            print( "Flag: " +str(flag))
            if flag:
                return True
            time.sleep(0.1)
        return False 

    def static_tf_callback(self, msg):
        print('Got the callback')
        print(msg.header)
        self.flag = True
