# License: Apache 2.0. See LICENSE file in root directory.
# Copyright(c) 2022 Intel Corporation. All Rights Reserved

'''
Launch realsense2_camera node & a frame latency printer node, 
This tool allow the user the evaluate the reduction of the frame latency when intra-process communication is used.
Note: currently default for color stream only
Run syntax: ros2 launch realsense2_camera rs_intra_process_demo_launch.py intra_process_comms:=true
'''
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
                           {'name': 'intra_process_comms',          'default': 'true', 'description': "enable intra-process communication"}, 
                          ]

def declare_configurable_parameters(parameters):
    return [DeclareLaunchArgument(param['name'], default_value=param['default'], description=param['description']) for param in parameters]

def set_configurable_parameters(parameters):
    return dict([(param['name'], LaunchConfiguration(param['name'])) for param in parameters])


def generate_launch_description():


    return LaunchDescription(declare_configurable_parameters(configurable_parameters) + [
        ComposableNodeContainer(
            name='my_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='realsense2_camera',
                    namespace='',
                    plugin='realsense2_camera::RealSenseNodeFactory',
                    name="camera",
                    parameters=[set_configurable_parameters(configurable_parameters)],
                    extra_arguments=[{'use_intra_process_comms': LaunchConfiguration("intra_process_comms")}]) ,
                ComposableNode(
                    package='realsense2_camera',
                    namespace='',
                    plugin='rs2_ros::tools::frame_latency::FrameLatencyNode',
                    name='frame_latency',
                    parameters=[set_configurable_parameters(configurable_parameters)],
                    extra_arguments=[{'use_intra_process_comms': LaunchConfiguration("intra_process_comms")}]) ,
                ],
            output='screen',
            emulate_tty=True, # needed for display of logs
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
    )])


