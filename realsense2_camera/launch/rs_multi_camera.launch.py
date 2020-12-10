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
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    config = os.path.join(
        get_package_share_directory('realsense2_camera'),
        'config',
        'd435.yaml')
    
    rviz_config_dir = os.path.join(
            get_package_share_directory('realsense2_camera'),
            'rviz',
            'rs_multi_camera.rviz')

    return LaunchDescription([
        # Realsense 1
        Node(
            package='realsense2_camera', 
            node_executable='realsense2_camera_node',
            node_namespace='camera1',
            parameters = [
                {config},
                {'serial_no': '728612070689',
                 'color_height': 720,
                 'color_width': 1280}],
            output='screen',
            emulate_tty=True,
            ),

        # Realsense 2
        Node(
            package='realsense2_camera', 
            node_executable='realsense2_camera_node',
            node_namespace='camera2',
            parameters = [
                {config},
                {'serial_no': '831612070734',
                 'color_height': 720,
                 'color_width': 1280}],                       
            output='screen',
            emulate_tty=True,
            ),            

        # rviz2
        DeclareLaunchArgument(
            'open_rviz',
            default_value='false',
            description='Launch Rviz?'),            
        Node(
            package='rviz2',
            node_executable='rviz2',
            node_name='rviz2',
            arguments=['-d', rviz_config_dir],
            condition=IfCondition(LaunchConfiguration("open_rviz"))
            ),            
    ])
