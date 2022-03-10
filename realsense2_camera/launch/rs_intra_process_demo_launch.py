# Copyright (c) 2022 Intel Corporation
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

"""Launch realsense2_camera node & a listener node."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


configurable_parameters = [{'name': 'camera_name',                  'default': 'camera', 'description': 'camera unique name'},
                           {'name': 'serial_no',                    'default': "''", 'description': 'choose device by serial number'},
                           {'name': 'usb_port_id',                  'default': "''", 'description': 'choose device by usb port id'},
                           {'name': 'device_type',                  'default': "''", 'description': 'choose device by type'},
                           {'name': 'log_level',                    'default': 'info', 'description': 'debug log level [DEBUG|INFO|WARN|ERROR|FATAL]'},                     
                           {'name': 'enable_color',                 'default': 'true', 'description': 'enable color stream'},
                           {'name': 'enable_depth',                 'default': 'false', 'description': 'enable depth stream'},
                           {'name': 'enable_infra1',                'default': 'false', 'description': 'enable infra1 stream'},
                           {'name': 'enable_infra2',                'default': 'false', 'description': 'enable infra2 stream'},
                           {'name': 'enable_gyro',                  'default': 'false', 'description': "enable gyro stream"},                           
                           {'name': 'enable_accel',                 'default': 'false', 'description': "enable accel stream"}, 
                          ]

def declare_configurable_parameters(parameters):
    return [DeclareLaunchArgument(param['name'], default_value=param['default'], description=param['description']) for param in parameters]

def set_configurable_parameters(parameters):
    return dict([(param['name'], LaunchConfiguration(param['name'])) for param in parameters])

def generate_launch_description():
    """Generate launch description with multiple components."""
    declare_configurable_parameters(configurable_parameters)
    container = ComposableNodeContainer(
            name='my_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='realsense2_camera',
                    plugin='realsense2_camera::RealSenseNodeFactory',
                    name="camera",
                    extra_arguments=[{'use_intra_process_comms': True}]) ,
                ComposableNode(
                    package='realsense2_camera',
                    plugin='rs2_ros::tools::frame_latency::FrameLatencyNode',
                    name='frame_latency',
                    extra_arguments=[{'use_intra_process_comms': True}])
                ],
            output='screen',
            #arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
            arguments=['--ros-args', '--log-level', 'info'],
    )
    return LaunchDescription([container])


