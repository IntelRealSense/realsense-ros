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

import sys
import os
import subprocess
import time

import launch

import launch_pytest
from launch_pytest.tools import process as process_tools

import pytest
from setuptools import find_packages
packages=find_packages(exclude=['test'])
assert os.getenv("COLCON_PREFIX_PATH")!=None,"COLCON_PREFIX_PATH was not set" 
sys.path.append(os.getenv("COLCON_PREFIX_PATH")+'/realsense2_camera/share/realsense2_camera/launch')
import rs_launch


def get_default_params():
    params = {}
    for param in rs_launch.configurable_parameters:
        params[param['name']] = param['default']
    return params

def kill_realsense2_camera_node():
    cmd = "kill -s INT $(ps aux | grep '[r]ealsense2_camera_node' | awk '{print $2}')"
    os.system(cmd)

@pytest.fixture
def start_camera():
    params = get_default_params()
    rosbag_dir = os.getenv("ROSBAG_FILE_PATH")
    print(rosbag_dir)
    assert rosbag_dir!=None,"ROSBAG_FILE_PATH was not set" 
    rosfile = rosbag_dir+"/../records/outdoors_1color.bag"
    params['rosbag_filename'] = rosfile
    params['color_width'] = '0'
    params['color_height'] = '0'
    params['depth_width'] = '0'
    params['depth_height'] = '0'
    params['infra_width'] = '0'
    params['infra_height'] = '0'
    params_str = ' '.join([key + ':=' + params[key] for key in sorted(params.keys())])
    cmd_params=['ros2', 'launch', 'realsense2_camera', 'rs_launch.py'] + params_str.split(' ')
    time.sleep(1)
    return launch.actions.ExecuteProcess(
        cmd=cmd_params,
        shell=True,
        cached_output=True,
    )

# This function specifies the processes to be run for the test.
@launch_pytest.fixture
def launch_description(start_camera):
    """Launch a simple process to start the camera"""
    return launch.LaunchDescription([
        start_camera,
        # Tell launch when to start the test
        # If no ReadyToTest action is added, one will be appended automatically.
        launch_pytest.actions.ReadyToTest()
    ])


@pytest.mark.launch(fixture=launch_description)
def test_start_camera(start_camera, launch_context):
    def validate_output(output):
        # by now, the camera would have started.
        service_list = subprocess.check_output(['ros2', 'node', 'list']).decode("utf-8")
        is_node_up = '/camera/camera' in service_list
        print(service_list)
        assert is_node_up, 'Node is NOT UP'
        print ('Node is UP')
        print ('*'*8 + ' Killing ROS ' + '*'*9)
        kill_realsense2_camera_node()
    process_tools.assert_output_sync(
        launch_context, start_camera, validate_output, timeout=5)
    yield
    #-2 for shutdown
    assert start_camera.return_code == -2 
