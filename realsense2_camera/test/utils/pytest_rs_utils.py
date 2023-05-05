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
import os
import sys
import time
from collections import deque
import pytest
from launch import LaunchDescription
import launch_ros.actions
import launch_pytest
import rclpy
from rclpy import qos
from rclpy.node import Node
assert os.getenv("COLCON_PREFIX_PATH")!=None,"COLCON_PREFIX_PATH was not set" 
sys.path.append(os.getenv("COLCON_PREFIX_PATH")+'/realsense2_camera/share/realsense2_camera/launch')
import rs_launch

'''
get the default parameters from the launch script so that the test doesn't have to
get updated for each change to the parameter or default values 
'''
def get_default_params():
    params = {}
    for param in rs_launch.configurable_parameters:
        params[param['name']] = param['default']
    return params

'''
function taken from rs_launch to kill the camera node. kept as a local copy so that when the template is 
used, it can be changed to kill say, a particular node alone depending on the test scenario
'''

def kill_realsense2_camera_node():
    cmd = "kill -s INT $(ps aux | grep '[r]ealsense2_camera_node' | awk '{print $2}')"
    os.system(cmd)

'''
get the default parameters from the launch script so that the test doesn't have to
get updated for each change to the parameter or default values 
'''

def get_default_params():
    params = {}
    for param in rs_launch.configurable_parameters:
        params[param['name']] = param['default']
    return params

''' 
The format used by rs_launch.py and the LuanchConfiguration yaml files are different,
so the params reused from the rs_launch has to be reformated to be added to yaml file.
'''
def convert_params(params):
    cparams = {}
    def strtobool (val):
        val = val.lower()
        if val == 'true':
            return True
        elif val == 'false':
            return False
        else:
            raise ValueError("invalid truth value %r" % (val,))
    for key, value in params.items():
        try:
            cparams[key] = int(value)
        except ValueError:
            try:
                cparams[key] = float(value)
            except ValueError:
                try:
                    cparams[key] = strtobool(value)
                except ValueError:
                    cparams[key] = value.replace("'","")
    return cparams

def get_params_string_for_launch(params):
    params_str = ' '.join(["" if params[key]=="''" else key + ':=' + params[key] for key in sorted(params.keys())])
    return params_str

''' 
The get_rs_node_description file is used to create a node description of an rs
camera with a temporary yaml file to hold the parameters.  
'''

def get_rs_node_description(name, params):
    import tempfile
    import yaml
    tmp_yaml = tempfile.NamedTemporaryFile(prefix='launch_rs_',delete=False)
    params = convert_params(params)
    ros_params = {"ros__parameters":params}
    camera_params = {"camera/"+name: ros_params}
    with open(tmp_yaml.name, 'w') as f:
        yaml.dump(camera_params, f)

    '''
    comment out the '#prefix' line, if you like gdb and want to debug the code, you may have to do more
    if you have more than one rs node.
    '''
    return launch_ros.actions.Node(
        package='realsense2_camera',
        namespace=params["camera_name"],
        name=name,
        #prefix=['xterm -e gdb --args'],
        executable='realsense2_camera_node',
        parameters=[tmp_yaml.name],
        output='screen',
        arguments=['--ros-args', '--log-level', "info"],
        emulate_tty=True,
    )

''' 
This function returns a launch description with three rs nodes that
use the same rosbag file. Test developer can use this as a reference and 
create a function that creates as many nodes (s)he wants for the test  
'''

@launch_pytest.fixture
def launch_descr_with_yaml():
    params = get_default_params()
    rosbag_dir = os.getenv("ROSBAG_FILE_PATH")
    assert rosbag_dir!=None,"ROSBAG_FILE_PATH was not set" 
    rosfile = rosbag_dir+"/outdoors_1color.bag"
    params['rosbag_filename'] = rosfile
    params['color_width'] = '0'
    params['color_height'] = '0'
    params['depth_width'] = '0'
    params['depth_height'] = '0'
    params['infra_width'] = '0'
    params['infra_height'] = '0'
    first_node = get_rs_node_description("camera", params)
    return LaunchDescription([
        first_node,
        launch_pytest.actions.ReadyToTest(),
    ])

''' 
This function returns a launch description with a single rs node instance built based on the parameter
passed, use the test_paramterized.py as example
'''
@launch_pytest.fixture
def launch_descr_with_parameters(request):
    changed_params = request.param
    params = get_default_params()
    for key, value in changed_params.items():
        params[key] = value    
    first_node = get_rs_node_description("camera", params)
    return LaunchDescription([
        first_node,
        launch_pytest.actions.ReadyToTest(),
    ])


''' 
This function returns a launch description with three rs nodes that
use the same rosbag file. Test developer can use this as a reference and 
create a function that creates as many nodes (s)he wants for the test  
'''

@launch_pytest.fixture
def launch_descr_with_yaml_multi_camera_instances():
    params = get_default_params()
    rosbag_dir = os.getenv("ROSBAG_FILE_PATH")
    assert rosbag_dir!=None,"ROSBAG_FILE_PATH was not set" 
    rosfile = rosbag_dir+"/outdoors_1color.bag"
    params['rosbag_filename'] = rosfile
    params['color_width'] = '0'
    params['color_height'] = '0'
    params['depth_width'] = '0'
    params['depth_height'] = '0'
    params['infra_width'] = '0'
    params['infra_height'] = '0'
    first_node = get_rs_node_description("camera", params)
    second_node = get_rs_node_description("camera1", params)
    third_node = get_rs_node_description("camera2", params)
    return LaunchDescription([
        first_node,
        second_node,
        third_node,
        #launch_pytest.actions.ReadyToTest(),
    ])

''' 
This is that holds the test node that listens to a subscription created by a test.  
'''
class RsTestNode(Node):
    def __init__(self, name='test_node'):
        print('\nCreating node... ' + name)
        super().__init__(name)
        self.flag = False
        self.data = {}

    def wait_for_node(self, node_name, timeout=8.0):
        start = time.time()
        flag = False
        print('Waiting for node... ' + node_name)
        while time.time() - start < timeout:
            flag = node_name in self.get_node_names()
            print(self.get_node_names())
            print( "Flag: " +str(flag))
            if flag:
                return True
            time.sleep(0.1)
        return False
    def create_subscription(self, msg_type, topic , data_type):
        super().create_subscription(msg_type, topic , self.rsCallback(topic), data_type)
        self.data[topic] = deque()
    def get_num_chunks(self,topic):
        return len(self.data[topic])
    def pop_first_chunk(self, topic):
        return self.data[topic].popleft()

    def rsCallback(self, topic):
        print("RSCallback")
        def _rsCallback(data):
            print('Got the callback for ' + topic)
            print(data.header)
            self.flag = True
            self.data[topic].append(data)
            #print(len(self.data[topic]))
        return _rsCallback
    def _callback(self, msg):
        print('Got the callback')
        print(msg.header)
        self.flag = True
        

class RsTestBaseClass():
    def init_test(self):
        rclpy.init()
        self.flag = False
        self.node = RsTestNode('RsTestNode')
        self.subscribed_topics = []
    def create_subscription(self, msg_type, topic , data_type):
        if not topic in self.subscribed_topics:
            self.node.create_subscription(msg_type, topic, data_type)
            self.subscribed_topics.append(topic)

    def spin_for_data(self,themes):
        start = time.time()
        timeout = 4.0
        print('Waiting for topic... ' )
        flag = False
        while time.time() - start < timeout:
            print('Spinning... ' )
            rclpy.spin_once(self.node)
            all_found = True 
            for theme in themes:
                if theme['expected_data_chunks'] > int(self.node.get_num_chunks(theme['topic'])):
                    all_found = False
                    break
            if all_found == True:
                flag =True
                break
        else:
            assert False, "run_test timedout"
        return flag

    def spin_for_time(self,wait_time):
        start = time.time()
        print('Waiting for topic... ' )
        flag = False
        while time.time() - start < wait_time:
            print('Spinning... ' )
            rclpy.spin_once(self.node)

    def run_test(self, themes):
        try:
            for theme in themes:
                self.create_subscription(theme['msg_type'], theme['topic'] , qos.qos_profile_sensor_data)
                print('subscription created for ' + theme['topic'])
            self.flag = self.spin_for_data(themes)                
        except  Exception as e:
            exc_type, exc_obj, exc_tb = sys.exc_info()
            fname = os.path.split(exc_tb.tb_frame.f_code.co_filename)[1]
            print(exc_type, fname, exc_tb.tb_lineno)
            if hasattr(e, 'message'):
                print(e.message)
            else:
                print(e)
            self.flag =False
        return self.flag
    def process_data(self, themes):
        for theme in themes:
            data = self.node.pop_first_chunk(theme['topic'])
            print("first chunck of data for"+ theme['topic'] + ":")
            print(data.header)
        return True
    def shutdown(self):
        self.node.destroy_node()
        rclpy.shutdown()