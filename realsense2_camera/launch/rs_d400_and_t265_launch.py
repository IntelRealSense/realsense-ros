# License: Apache 2.0. See LICENSE file in root directory.
# Copyright(c) 2022 Intel Corporation. All Rights Reserved.

"""Launch realsense2_camera node without rviz2."""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import ThisLaunchFileDir
from launch.launch_description_sources import PythonLaunchDescriptionSource
import sys
import pathlib
sys.path.append(str(pathlib.Path(__file__).parent.absolute()))
import rs_launch

local_parameters = [{'name': 'camera_name1', 'default': 'D400', 'description': 'camera unique name'},
                    {'name': 'device_type1', 'default': 'd4.', 'description': 'choose device by type'},
                    {'name': 'camera_name2', 'default': 'T265', 'description': 'camera unique name'},
                    {'name': 'device_type2', 'default': 't265', 'description': 'choose device by type'},
                    {'name': 'enable_fisheye12', 'default': 'false', 'description': 'topic for T265 wheel odometry'},
                    {'name': 'enable_fisheye22', 'default': 'false', 'description': 'topic for T265 wheel odometry'},
                   ]

def generate_launch_description():
    return LaunchDescription(
        rs_launch.declare_configurable_parameters(local_parameters) + 
        [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/rs_multi_camera_launch.py']),
            launch_arguments=rs_launch.set_configurable_parameters(local_parameters).items(),
        ),
    ])
