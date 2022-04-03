# License: Apache 2.0. See LICENSE file in root directory.
# Copyright (c) 2022 Intel Corporation

# /* Author: Doron Hirshberg */
import os
import launch
from launch_ros.actions import Node
import launch.events
from ament_index_python.packages import get_package_share_directory
import sys
sys.path.append(os.path.dirname(os.path.realpath(__file__)))
from launch_utils import to_urdf

def generate_launch_description():
    available_urdf_files = [f for f in os.listdir(os.path.join(get_package_share_directory('realsense2_description'), 'urdf')) if f.startswith('test_')]
    params = dict([aa for aa in [aa.split(':=') for aa in sys.argv] if len(aa)==2])
    if ('model' not in params or params['model'] not in available_urdf_files):
        print ('USAGE:')
        print ('ros2 launch realsense2_description view_model.launch.py model:=<model>')
        print ('Available argumants for <model> are as follows:')
        print ('\n'.join(available_urdf_files))
        return launch.LaunchDescription()

    rviz_config_dir = os.path.join(get_package_share_directory('realsense2_description'), 'rviz', 'urdf.rviz')
    xacro_path = os.path.join(get_package_share_directory('realsense2_description'), 'urdf', params['model'])
    urdf = to_urdf(xacro_path, {'use_nominal_extrinsics' : 'true', 'add_plug' : 'true'})
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        node_name='rviz2',
        output = 'screen',
        arguments=['-d', rviz_config_dir],
        parameters=[{'use_sim_time': False}]
        )
    model_node = Node(
        node_name='model_node',
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='',
        output='screen',
        arguments = [urdf]
        )
    return launch.LaunchDescription([rviz_node, model_node])
