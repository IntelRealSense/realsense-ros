# Copyright (c) 2018 Intel Corporation
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

"""Launch realsense2_camera node without rviz2."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
import launch_ros.actions
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition


configurable_parameters = [{'name': 'camera_name', 'default': 'camera', 'description': 'camera unique name'},
                        {'name': 'serial_no', 'default': '', 'description': 'choose device by serial number'},
                        {'name': 'usb_port_id', 'default': '', 'description': 'choose device by usb port id'},
                        {'name': 'device_type', 'default': '', 'description': 'choose device by type'},
                        ]

def declare_configurable_parameters():
    aa = [DeclareLaunchArgument('config_file', default_value='', description='yaml config file')]
    return aa + [DeclareLaunchArgument(param['name'], default_value=param['default'], description=param['description']) for param in configurable_parameters]

def set_configurable_parameters():
    aa = dict([(param['name'], LaunchConfiguration(param['name'])) for param in configurable_parameters])
    aa['serial_no'] = LaunchConfiguration('serial_no')
    return aa

def generate_launch_description():
    DeclareLaunchArgument('config_file', default_value='', description='yaml config file'),
    return LaunchDescription(declare_configurable_parameters() + [
        # Realsense
        launch_ros.actions.Node(
            condition=IfCondition(PythonExpression(["'", LaunchConfiguration('config_file'), "' == ''"])),
            package='realsense2_camera', 
            node_namespace=LaunchConfiguration("camera_name"),
            name=LaunchConfiguration("camera_name"),
            node_executable='realsense2_camera_node',
            parameters = [set_configurable_parameters()
                          ],
            output='screen',
            emulate_tty=True,
            ),
        launch_ros.actions.Node(
            condition=IfCondition(PythonExpression(["'", LaunchConfiguration('config_file'), "' != ''"])),
            package='realsense2_camera', 
            node_namespace=LaunchConfiguration("camera_name"),
            name=LaunchConfiguration("camera_name"),
            node_executable='realsense2_camera_node',
            parameters = [set_configurable_parameters()
                          ,{LaunchConfiguration("config_file")}
                          ],
            output='screen',
            emulate_tty=True,
            ),
    ])
