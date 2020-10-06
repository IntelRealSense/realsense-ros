# Copyright (c) 2020 Intel Corporation
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

# /* Author: Doron Hirshberg */
import os
import launch
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import sys
sys.path.append(os.path.dirname(os.path.realpath(__file__)))
from launch_utils import to_urdf

def generate_launch_description():
    rviz_config_dir = os.path.join(get_package_share_directory('realsense2_description'), 'rviz', 'urdf.rviz')
    xacro_path = os.path.join(get_package_share_directory('realsense2_description'), 'urdf', 'test_d435i_camera.urdf.xacro')
    urdf = to_urdf(xacro_path, {'use_nominal_extrinsics' : 'true', 'add_plug' : 'true'})
    rviz_node = Node(
        package='rviz2',
        node_executable='rviz2',
        node_name='rviz2',
        output = 'screen',
        arguments=['-d', rviz_config_dir],
        parameters=[{'use_sim_time': False}]
        )
    model_node = Node(
        node_name='model_node',
        package='robot_state_publisher',
        node_executable='robot_state_publisher',
        node_namespace='',
        output='screen',
        arguments = [urdf]
        )
    return launch.LaunchDescription([rviz_node, model_node])
