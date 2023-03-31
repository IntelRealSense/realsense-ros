// Copyright 2023 Intel Corporation. All Rights Reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


import os
import sys
import subprocess
import time
import unittest

import launch
import launch.actions
import launch_ros.actions
import launch_testing.actions
import launch_testing.markers
import pytest
import rclpy
from rclpy import qos
from rclpy.node import Node
from sensor_msgs.msg import Image as msg_Image
assert os.getenv("COLCON_PREFIX_PATH")!=None,"COLCON_PREFIX_PATH was not set" 
sys.path.append(os.getenv("COLCON_PREFIX_PATH")+'/realsense2_camera/share/realsense2_camera/launch')
import rs_launch 

import launch
from launch import LaunchDescription
import launch_pytest
from launch_pytest.tools import process as process_tools
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

import pytest

def get_default_params():
    params = {}
    for param in rs_launch.configurable_parameters:
        params[param['name']] = param['default']
    return params

def kill_realsense2_camera_node():
    cmd = "kill -s INT $(ps aux | grep '[r]ealsense2_camera_node' | awk '{print $2}')"
    os.system(cmd)

''' all these to reuse the params from the rs_launch'''
def convert_params(params):
    cparams = {}
    def strtobool (val):
        val = val.lower()
        if val == 'true':
            return True
        elif val == 'false':
            return False
        else:
            raise ValueError("invalid truth value %r" % (val,))
    for key, value in params.items():
        try:
            cparams[key] = int(value)
        except ValueError:
            try:
                cparams[key] = float(value)
            except ValueError:
                try:
                    cparams[key] = strtobool(value)
                except ValueError:
                    cparams[key] = value.replace("'","")
    return cparams

def get_rs_node_description(name, params):
    import tempfile
    import yaml
    tmp_yaml = tempfile.NamedTemporaryFile(prefix='launch_rs_',delete=False)
    params = convert_params(params)
    ros_params = {"ros__parameters":params}
    camera_params = {"camera/"+name: ros_params}
    #tmp_yaml.write(camera_params)
    with open(tmp_yaml.name, 'w') as f:
        yaml.dump(camera_params, f)

    return launch_ros.actions.Node(
        package='realsense2_camera',
        namespace=params["camera_name"],
        name=name,
        #prefix=['xterm -e gdb --args'],
        executable='realsense2_camera_node',
        parameters=[tmp_yaml.name],
        output='screen',
        arguments=['--ros-args', '--log-level', "info"],
        emulate_tty=True,
    )

@launch_pytest.fixture
def launch_descr_with_yaml():
    params = get_default_params()
    rosbag_dir = os.getenv("ROSBAG_FILE_PATH")
    assert rosbag_dir!=None,"ROSBAG_FILE_PATH was not set" 
    rosfile = rosbag_dir+"/outdoors_1color.bag"
    params['rosbag_filename'] = rosfile
    params['color_width'] = '0'
    params['color_height'] = '0'
    params['depth_width'] = '0'
    params['depth_height'] = '0'
    params['infra_width'] = '0'
    params['infra_height'] = '0'
    first_node = get_rs_node_description("camera", params)
    second_node = get_rs_node_description("camera1", params)
    third_node = get_rs_node_description("camera2", params)
    return LaunchDescription([
        first_node,
        second_node,
        third_node,
        #launch_pytest.actions.ReadyToTest(),
    ])

@pytest.mark.launch(fixture=launch_descr_with_yaml)
def test_using_function(launch_context):
    # by now, the camera would have started.
    service_list = subprocess.check_output(['ros2', 'node', 'list']).decode("utf-8")
    is_node_up = '/camera/camera' in service_list
    print(service_list)
    assert is_node_up, 'Node is NOT UP'
    time.sleep(5)
    print ('Node is UP')
    print ('*'*8 + ' Killing ROS ' + '*'*9)
    kill_realsense2_camera_node()
    yield
    assert True



@pytest.mark.launch(fixture=launch_descr_with_yaml)
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
