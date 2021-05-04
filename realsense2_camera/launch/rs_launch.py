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

"""Launch realsense2_camera node."""
import os
from launch import LaunchDescription
import launch_ros.actions
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition


configurable_parameters = [{'name': 'camera_name',                  'default': 'camera', 'description': 'camera unique name'},
                           {'name': 'serial_no',                    'default': "''", 'description': 'choose device by serial number'},
                           {'name': 'usb_port_id',                  'default': "''", 'description': 'choose device by usb port id'},
                           {'name': 'device_type',                  'default': "''", 'description': 'choose device by type'},
                           {'name': 'config_file',                  'default': "''", 'description': 'yaml config file'},
                           {'name': 'unite_imu_method',             'default': "''", 'description': '[copy|linear_interpolation]'},                           
                           {'name': 'json_file_path',               'default': "''", 'description': 'allows advanced configuration'},                           
                           {'name': 'output',                       'default': 'screen', 'description': 'pipe node output [screen|log]'},                           
                           {'name': 'depth_module.width',           'default': '640', 'description': 'depth image width'},                           
                           {'name': 'depth_module.height',          'default': '480', 'description': 'depth image height'},                           
                           {'name': 'depth_module.fps',             'default': '30', 'description': "''"},                           
                           {'name': 'enable_depth',                 'default': 'true', 'description': 'enable depth stream'},
                           {'name': 'rgb_camera.width',             'default': '640', 'description': 'color image width'},                           
                           {'name': 'rgb_camera.height',            'default': '480', 'description': 'color image height'},                           
                           {'name': 'rgb_camera.fps',               'default': '30', 'description': "''"},                           
                           {'name': 'enable_color',                 'default': 'true', 'description': 'enable color stream'},
                           {'name': 'enable_infra1',                'default': 'false', 'description': 'enable infra1 stream'},
                           {'name': 'enable_infra2',                'default': 'false', 'description': 'enable infra2 stream'},
                           {'name': 'infra_rgb',                    'default': 'false', 'description': 'enable infra2 stream'},
                           {'name': 'tracking_module.width',        'default': '848', 'description': 'fisheye width'},
                           {'name': 'tracking_module.height',       'default': '800', 'description': 'fisheye width'},
                           {'name': 'tracking_module.fps',          'default': '30', 'description': "''"},                           
                           {'name': 'enable_fisheye1',              'default': 'true', 'description': 'enable fisheye1 stream'},
                           {'name': 'enable_fisheye2',              'default': 'true', 'description': 'enable fisheye2 stream'},
                           {'name': 'enable_confidence',            'default': 'true', 'description': 'enable depth stream'},
                           {'name': 'gyro_fps',                     'default': '400.', 'description': "''"},                           
                           {'name': 'accel_fps',                    'default': '0.', 'description': "''"},                           
                           {'name': 'enable_gyro',                  'default': 'false', 'description': "''"},                           
                           {'name': 'enable_accel',                 'default': 'false', 'description': "''"},                           
                           {'name': 'enable_pose',                  'default': 'true', 'description': "''"},                           
                           {'name': 'pose_fps',                     'default': '200.', 'description': "''"},                           
                           {'name': 'pointcloud.enable',     'default': 'true', 'description': ''}, 
                           {'name': 'pointcloud.stream_filter',     'default': '2', 'description': 'texture stream for pointcloud'}, 
                           {'name': 'pointcloud_texture_index',     'default': '0', 'description': 'texture stream index for pointcloud'},                           
                           {'name': 'enable_sync',                  'default': 'false', 'description': "''"},                           
                           {'name': 'align_depth',                  'default': 'false', 'description': "''"},                           
                           {'name': 'clip_distance',                'default': '-2.', 'description': "''"},                           
                           {'name': 'linear_accel_cov',             'default': '0.01', 'description': "''"},                           
                           {'name': 'initial_reset',                'default': 'false', 'description': "''"},                           
                           {'name': 'allow_no_texture_points',      'default': 'false', 'description': "''"},                           
                           {'name': 'calib_odom_file',              'default': "''", 'description': "''"},                           
                           {'name': 'topic_odom_in',                'default': "''", 'description': 'topic for T265 wheel odometry'},
                           {'name': 'tf_publish_rate',              'default': '0.0', 'description': 'Rate of publishing static_tf'},
                           {'name': 'decimation_filter.enable',     'default': 'false', 'description': 'Rate of publishing static_tf'},
                           {'name': 'rosbag_filename',              'default': "''", 'description': 'A realsense bagfile to run from as a device'},
                           {'name': 'depth_module.global_time_enabled',              'default': "false", 'description': ''},
                           {'name': 'motion_module.global_time_enabled',              'default': "false", 'description': ''},
                           {'name': 'rgb_camera.global_time_enabled',              'default': "false", 'description': ''},
                           
                          ]

def declare_configurable_parameters(parameters):
    return [DeclareLaunchArgument(param['name'], default_value=param['default'], description=param['description']) for param in parameters]

def set_configurable_parameters(parameters):
    return dict([(param['name'], LaunchConfiguration(param['name'])) for param in parameters])

def generate_launch_description():
    log_level = 'info'
    if (os.getenv('ROS_DISTRO') == "dashing") or (os.getenv('ROS_DISTRO') == "eloquent"):
        return LaunchDescription(declare_configurable_parameters(configurable_parameters) + [
            # Realsense
            launch_ros.actions.Node(
                condition=IfCondition(PythonExpression([LaunchConfiguration('config_file'), " == ''"])),
                package='realsense2_camera', 
                node_namespace=LaunchConfiguration("camera_name"),
                node_name=LaunchConfiguration("camera_name"),
                node_executable='realsense2_camera_node',
                prefix=['stdbuf -o L'],
                parameters = [set_configurable_parameters(configurable_parameters)
                            ],
                output='screen',
                arguments=['--ros-args', '--log-level', log_level],
                ),
            launch_ros.actions.Node(
                condition=IfCondition(PythonExpression([LaunchConfiguration('config_file'), " != ''"])),
                package='realsense2_camera', 
                node_namespace=LaunchConfiguration("camera_name"),
                node_name=LaunchConfiguration("camera_name"),
                node_executable='realsense2_camera_node',
                prefix=['stdbuf -o L'],
                parameters = [set_configurable_parameters(configurable_parameters)
                            ,{LaunchConfiguration("config_file")}
                            ],
                output='screen',
                arguments=['--ros-args', '--log-level', log_level],
                ),
            ])
    else:
        return LaunchDescription(declare_configurable_parameters(configurable_parameters) + [
            # Realsense
            launch_ros.actions.Node(
                condition=IfCondition(PythonExpression([LaunchConfiguration('config_file'), " == ''"])),
                package='realsense2_camera', 
                namespace=LaunchConfiguration("camera_name"),
                name=LaunchConfiguration("camera_name"),
                executable='realsense2_camera_node',
                parameters = [set_configurable_parameters(configurable_parameters)
                            ],
                output='screen',
                arguments=['--ros-args', '--log-level', log_level],
                emulate_tty=True,
                ),
            launch_ros.actions.Node(
                condition=IfCondition(PythonExpression([LaunchConfiguration('config_file'), " != ''"])),
                package='realsense2_camera', 
                namespace=LaunchConfiguration("camera_name"),
                name=LaunchConfiguration("camera_name"),
                executable='realsense2_camera_node',
                parameters = [set_configurable_parameters(configurable_parameters)
                            ,{LaunchConfiguration("config_file")}
                            ],
                output='screen',
                arguments=['--ros-args', '--log-level', log_level],
                emulate_tty=True,
                ),
        ])
