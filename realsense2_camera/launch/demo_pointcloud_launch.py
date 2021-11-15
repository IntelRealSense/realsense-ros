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


# DESCRIPTION #
# ----------- #
# Use this launch file to launch realsense2_camera node and rviz2 to view the published pointcloud.
# The Parameters available for definition in the command line for each camera are described in rs_launch.configurable_parameters
# command line example:
# ros2 launch realsense2_camera demo_pointcloud_launch.py 

"""Launch realsense2_camera node."""
import copy
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os
import sys
import pathlib
sys.path.append(str(pathlib.Path(__file__).parent.absolute()))
import rs_launch


local_parameters = [{'name': 'enable_pointcloud', 'default': 'true', 'description': 'enable pointcloud'},
                   ]

def set_configurable_parameters(local_params):
    return dict([(param['original_name'], LaunchConfiguration(param['name'])) for param in local_params])

def generate_launch_description():
    rviz_config_dir = os.path.join(get_package_share_directory('realsense2_camera'), 'rviz', 'pointcloud.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output = 'screen',
        arguments=['-d', rviz_config_dir],
        parameters=[{'use_sim_time': False}]
        )


    return LaunchDescription(
        rs_launch.declare_configurable_parameters(local_parameters) + 
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/rs_launch.py']),
                launch_arguments=rs_launch.set_configurable_parameters(local_parameters).items(),
                ),
            rviz_node
        ])
