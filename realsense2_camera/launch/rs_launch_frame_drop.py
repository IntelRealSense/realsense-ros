# Copyright 2023 Intel Corporation. All Rights Reserved.
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

'''
Launch realsense2_camera node & a frame latency printer node, 
This tool allow the user the evaluate the reduction of the frame latency when intra-process communication is used.
Run syntax: ros2 launch realsense2_camera rs_intra_process_demo_launch.py intra_process_comms:=true
Note: 
*   Running this tool require building with build tools flag on (colcon build --cmake-args '-DBUILD_TOOLS=ON')
*   Currently default for color stream only
'''
import sys
import subprocess
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
import launch_ros.actions

# Make sure required packages can be found
process = subprocess.run(['ros2','component', 'types'],
                         stdout=subprocess.PIPE, 
                         universal_newlines=True)

rs_node_class=  'RealSenseNodeFactory'
rs_latency_tool_class = 'FrameLatencyNode'

if process.stdout.find(rs_node_class) == -1 or process.stdout.find(rs_latency_tool_class) == -1 :
    sys.exit('Cannot locate all required node components (' + rs_node_class + ',' + rs_latency_tool_class + ') on the available component list\n' + process.stdout + \
    '\nplease make sure you run "colcon build --cmake-args \'-DBUILD_TOOLS=ON\'" command before running this launch file')


configurable_parameters = [{'name': 'camera_name',                  'default': 'camera', 'description': 'camera unique name'},
                           {'name': 'serial_no',                    'default': "''", 'description': 'choose device by serial number'},
                           {'name': 'usb_port_id',                  'default': "''", 'description': 'choose device by usb port id'},
                           {'name': 'device_type',                  'default': "''", 'description': 'choose device by type'},
                           {'name': 'log_level',                    'default': 'info', 'description': 'debug log level [DEBUG|INFO|WARN|ERROR|FATAL]'},                     
                           {'name': 'rgb_camera.profile',           'default': '0,0,0', 'description': 'color image width'},
                           {'name': 'enable_color',                 'default': 'true', 'description': 'enable color stream'},
                           {'name': 'enable_depth',                 'default': 'true', 'description': 'enable depth stream'},
                           {'name': 'enable_infra1',                'default': 'true', 'description': 'enable infra1 stream'},
                           {'name': 'enable_infra2',                'default': 'true', 'description': 'enable infra2 stream'},
                           {'name': 'enable_gyro',                  'default': 'true', 'description': "enable gyro stream"},                           
                           {'name': 'enable_accel',                 'default': 'true', 'description': "enable accel stream"}, 
                           {'name': 'intra_process_comms',          'default': 'true', 'description': "enable intra-process communication"}, 
                           {'name': 'publish_tf',                   'default': 'true', 'description': '[bool] enable/disable publishing static & dynamic TF'},
                           {'name': 'tf_publish_rate',              'default': '0.0', 'description': '[double] rate in HZ for publishing dynamic TF'},
                           {'name': 'gyro_fps',                     'default': '200', 'description': "''"},
                           {'name': 'accel_fps',                    'default': '200', 'description': "''"},
                           {'name': 'rgb_camera.profile',           'default': '640x360x30', 'description': 'color image width'},
                           {'name': 'depth_module.profile',         'default': '640x360x30', 'description': 'color image width'},
                          ]

def declare_configurable_parameters(parameters):
    return [DeclareLaunchArgument(param['name'], default_value=param['default'], description=param['description']) for param in parameters]

def set_configurable_parameters(parameters):
    return dict([(param['name'], LaunchConfiguration(param['name'])) for param in parameters])


def generate_launch_description():


    return LaunchDescription(declare_configurable_parameters(configurable_parameters) + [
            launch_ros.actions.Node(
                package='realsense2_camera',
                namespace="",
                name="camera",
                executable='realsense2_camera_node',
                parameters=[set_configurable_parameters(configurable_parameters)],
                output='screen',
                arguments=['--ros-args', '--log-level', 'info'],
                emulate_tty=True,
                ),
            launch_ros.actions.Node(
                package='realsense2_camera',
                namespace='',
                executable='frame_drop_org.py',
                name='frame_drop',
                arguments=[],
                output='screen',
                ),
    ])


