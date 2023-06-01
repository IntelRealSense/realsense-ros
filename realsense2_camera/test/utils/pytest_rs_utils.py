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

import numpy as np

from launch import LaunchDescription
import launch_ros.actions
import launch_pytest
import rclpy
from rclpy import qos
from rclpy.node import Node
from rclpy.utilities import ok

from sensor_msgs.msg import Image as msg_Image
from sensor_msgs.msg import Imu as msg_Imu
import quaternion
if (os.getenv('ROS_DISTRO') != "dashing"):
    import tf2_ros


assert os.getenv("COLCON_PREFIX_PATH")!=None,"COLCON_PREFIX_PATH was not set" 
sys.path.append(os.getenv("COLCON_PREFIX_PATH")+'/realsense2_camera/share/realsense2_camera/launch')
import rs_launch
sys.path.append(os.path.abspath(os.path.dirname(__file__)+"/../../scripts"))
'''
Copied from the old code in scripts folder
'''
from importRosbag.importRosbag import importRosbag

def ImuGetData(rec_filename, topic):
    # res['value'] = first value of topic.
    # res['max_diff'] = max difference between returned value and all other values of topic in recording.

    res = dict()
    res['value'] = None
    res['max_diff'] = [0,0,0]
    data = importRosbag(rec_filename, importTopics=[topic], log='ERROR', disable_bar=True)[topic]
    res['value'] = data['acc'][0,:]
    res['max_diff'] = data['acc'].max(0) - data['acc'].min(0)
    return res

def AccelGetData(rec_filename):
    return ImuGetData(rec_filename, '/device_0/sensor_2/Accel_0/imu/data')

def AccelGetDataDeviceStandStraight(rec_filename):
    gt_data = AccelGetData(rec_filename)
    gt_data['ros_value'] = np.array([0.63839424, 0.05380408, 9.85343552])
    gt_data['ros_max_diff'] = np.array([0.06940174, 0.04032778, 0.05982018])
    return gt_data

def ImageGetData(rec_filename, topic):
    all_avg = []
    ok_percent = []
    res = dict()

    data = importRosbag(rec_filename, importTopics=[topic], log='ERROR', disable_bar=True)[topic]
    for pyimg in data['frames']:
        ok_number = (pyimg != 0).sum()
        ok_percent.append(float(ok_number) / (pyimg.shape[0] * pyimg.shape[1]))
        all_avg.append(pyimg.sum() / ok_number)

    all_avg = np.array(all_avg)

    channels = pyimg.shape[2] if len(pyimg.shape) > 2 else 1
    res['num_channels'] = channels
    res['shape'] = pyimg.shape
    res['avg'] = all_avg.mean()
    res['ok_percent'] = {'value': (np.array(ok_percent).mean()) / channels, 'epsilon': 0.01}
    res['epsilon'] = max(all_avg.max() - res['avg'], res['avg'] - all_avg.min())
    res['reported_size'] = [pyimg.shape[1], pyimg.shape[0], pyimg.shape[1]*pyimg.dtype.itemsize*channels]

    return res

def ImageColorGetData(rec_filename):
    return ImageGetData(rec_filename, '/device_0/sensor_1/Color_0/image/data')


def ImageDepthGetData(rec_filename):
    return ImageGetData(rec_filename, '/device_0/sensor_0/Depth_0/image/data')


def ImageColorTest(data, gt_data):
    # check that all data['num_channels'] are the same as gt_data['num_channels'] and that avg value of all
    # images are within epsilon of gt_data['avg']
    try:
        if ('num_channels' not in data):
            msg = 'No data received'
            return False, msg
        channels = list(set(data['num_channels']))
        msg = 'Expect %d channels. Got %d channels.' % (gt_data['num_channels'], channels[0])
        print (msg)
        if len(channels) > 1 or channels[0] != gt_data['num_channels']:
            return False, msg
        msg = 'Expected all received images to be the same shape. Got %s' % str(set(data['shape']))
        print (msg)
        if len(set(data['shape'])) > 1:
            return False, msg
        msg = 'Expected shape to be %s. Got %s' % (gt_data['shape'], list(set(data['shape']))[0])
        print (msg)
        if (np.array(list(set(data['shape']))[0]) != np.array(gt_data['shape'])).any():
            return False, msg
        msg = 'Expected header [width, height, step] to be %s. Got %s' % (gt_data['reported_size'], list(set(data['reported_size']))[0])
        print (msg)
        if (np.array(list(set(data['reported_size']))[0]) != np.array(gt_data['reported_size'])).any():
            return False, msg
        msg = 'Expect average of %.3f (+-%.3f). Got average of %.3f.' % (gt_data['avg'].mean(), gt_data['epsilon'], np.array(data['avg']).mean())
        print (msg)
        if abs(np.array(data['avg']).mean() - gt_data['avg'].mean()) > gt_data['epsilon']:
            return False, msg

        msg = 'Expect no holes percent > %.3f. Got %.3f.' % (gt_data['ok_percent']['value']-gt_data['ok_percent']['epsilon'], np.array(data['ok_percent']).mean())
        print (msg)
        if np.array(data['ok_percent']).mean() < gt_data['ok_percent']['value']-gt_data['ok_percent']['epsilon']:
            return False, msg
    except Exception as e:
        msg = '%s' % e
        print ('Test Failed: %s' % msg)
        return False, msg
    return True, ''

def ImuTest(data, gt_data):
    # check that the imu data received is the same as in the recording. 
    # check that in the rotated imu received the g-accelartation is pointing up according to ROS standards.
    try:
        v_data = np.array([data['value'][0].x, data['value'][0].y, data['value'][0].z])
        v_gt_data = gt_data['value']
        diff = v_data - v_gt_data
        max_diff = abs(diff).max()
        msg = 'original accel: Expect max diff of %.3f. Got %.3f.' % (gt_data['max_diff'].max(), max_diff)
        print (msg)
        if max_diff > gt_data['max_diff'].max():
            return False, msg

        v_data = np.array(data['ros_value']).mean(0)
        v_gt_data = gt_data['ros_value']
        diff = v_data - v_gt_data
        max_diff = abs(diff).max()
        msg = 'rotated to ROS: Expect max diff of %.3f. Got %.3f.' % (gt_data['ros_max_diff'].max(), max_diff)
        print (msg)
        if max_diff > gt_data['ros_max_diff'].max():
            return False, msg
    except Exception as e:
        msg = '%s' % e
        print ('Test Failed: %s' % msg)
        return False, msg
    return True, ''


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
    pass

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
    camera_params = {name+"/"+name: ros_params}
    with open(tmp_yaml.name, 'w') as f:
        yaml.dump(camera_params, f)

    '''
    comment out the '#prefix' line, if you like gdb and want to debug the code, you may have to do more
    if you have more than one rs node.
    '''
    return launch_ros.actions.Node(
        package='realsense2_camera',
        #namespace=LaunchConfiguration("camera_name"),
        #name=LaunchConfiguration("camera_name"),
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
def launch_descr_with_yaml(request):
    changed_params = request.param
    params = get_default_params()
    for key, value in changed_params.items():
        params[key] = value   
    if  'camera_name' not in changed_params:
        params['camera_name'] = 'camera_with_yaml'
    first_node = get_rs_node_description(params['camera_name'], params)
    return LaunchDescription([
        first_node,
        launch_pytest.actions.ReadyToTest(),
    ]),request.param

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
    if  'camera_name' not in changed_params:
        params['camera_name'] = 'camera_with_params'
    first_node = get_rs_node_description(params['camera_name'], params)
    return LaunchDescription([
        first_node,
        launch_pytest.actions.ReadyToTest(),
    ]),request.param


''' 
This is that holds the test node that listens to a subscription created by a test.  
'''
class RsTestNode(Node):
    def __init__(self, name='test_node'):
        print('\nCreating node... ' + name)
        super().__init__(name)
        self.flag = False
        self.data = {}
        self.tfBuffer = None

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
    def create_subscription(self, msg_type, topic , data_type, store_raw_data):
        super().create_subscription(msg_type, topic , self.rsCallback(topic,msg_type, store_raw_data), data_type)
        self.data[topic] = deque()
        if (os.getenv('ROS_DISTRO') != "dashing") and (self.tfBuffer == None):
            self.tfBuffer = tf2_ros.Buffer()
            self.tf_listener = tf2_ros.TransformListener(self.tfBuffer, super())

    def get_num_chunks(self,topic):
        return len(self.data[topic])
    
    def pop_first_chunk(self, topic):
        return self.data[topic].popleft()
    
    def image_msg_to_numpy(self, data):
        fmtString = data.encoding
        if fmtString in ['mono8', '8UC1', 'bgr8', 'rgb8', 'bgra8', 'rgba8']:
            img = np.frombuffer(data.data, np.uint8)
        elif fmtString in ['mono16', '16UC1', '16SC1']:
            img = np.frombuffer(data.data, np.uint16)
        elif fmtString == '32FC1':
            img = np.frombuffer(data.data, np.float32)
        else:
            print('image format not supported:' + fmtString)
            return None

        depth = data.step / (data.width * img.dtype.itemsize)
        if depth > 1:
            img = img.reshape(data.height, data.width, int(depth))
        else:
            img = img.reshape(data.height, data.width)
        return img
    '''
    if the store_raw_data is not enabled, the call back will process the data.
    The processing of data is taken from the existing testcase in scripts rs2_test
    '''
    def rsCallback(self, topic, msg_type, store_raw_data):
        print("RSCallback")
        def _rsCallback(data):
            print('Got the callback for ' + topic)
            print(data.header)
            self.flag = True
            if store_raw_data == True:
                self.data[topic].append(data)
            elif msg_type == msg_Image:
                func_data = dict()
                func_data.setdefault('avg', [])
                func_data.setdefault('ok_percent', [])
                func_data.setdefault('num_channels', [])
                func_data.setdefault('shape', [])
                func_data.setdefault('reported_size', [])

                pyimg = self.image_msg_to_numpy(data)
                channels = pyimg.shape[2] if len(pyimg.shape) > 2 else 1
                ok_number = (pyimg != 0).sum()
                func_data['avg'].append(pyimg.sum() / ok_number)
                func_data['ok_percent'].append(float(ok_number) / (pyimg.shape[0] * pyimg.shape[1]) / channels)
                func_data['num_channels'].append(channels)
                func_data['shape'].append(pyimg.shape)
                func_data['reported_size'].append((data.width, data.height, data.step))

                self.data[topic].append(func_data)
            elif msg_type == msg_Imu:
                func_data = dict()
                func_data.setdefault('value', [])
                func_data.setdefault('ros_value', [])
                try:
                    frame_id = data.header.frame_id
                    value = data.linear_acceleration
                    func_data['value'].append(value)
                    
                    if (self.tfBuffer.can_transform('camera_link', frame_id, rclpy.time.Time(), rclpy.time.Duration(nanoseconds=3e6))):
                        msg = self.tfBuffer.lookup_transform('camera_link', frame_id, rclpy.time.Time(), rclpy.time.Duration(nanoseconds=3e6)).transform
                        quat = np.quaternion(msg.rotation.x, msg.rotation.y, msg.rotation.z, msg.rotation.w)
                        point = np.matrix([value.x, value.y, value.z], dtype='float32')
                        point.resize((3, 1))
                        rotated = quaternion.as_rotation_matrix(quat) * point
                        rotated.resize(1,3)
                        func_data['ros_value'].append(rotated)
                    
                    self.data[topic].append(func_data)

                except Exception as e:
                    print(e)
                    return
            else:
                self.data[topic].append(data)
            #print(len(self.data[topic]))
        return _rsCallback
    def _callback(self, msg):
        print('Got the callback')
        print(msg.header)
        self.flag = True
        

class RsTestBaseClass():
    def init_test(self,name='RsTestNode'):
        if not ok():
            rclpy.init()
        self.flag = False
        self.node = RsTestNode(name)
        self.subscribed_topics = []
    def create_subscription(self, msg_type, topic, data_type, store_raw_data=False):
        if not topic in self.subscribed_topics:
            self.node.create_subscription(msg_type, topic, data_type, store_raw_data)
            self.subscribed_topics.append(topic)

    def spin_for_data(self,themes):
        start = time.time()
        '''
        timeout value varies depending upon the system, it needs to be more if
        the access is over the network
        '''
        timeout = 8.0
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
                store_raw_data = False
                if 'store_raw_data' in theme:
                    store_raw_data = theme['store_raw_data']
                self.create_subscription(theme['msg_type'], theme['topic'] , qos.qos_profile_sensor_data,store_raw_data)
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
    '''
    Please override and use your own process_data if the default check is not suitable.
    Please also store_raw_data parameter in the themes as True, if you want the
    callback to store the data as is.  
    '''
    def process_data(self, themes):
        for theme in themes:
            data = self.node.pop_first_chunk(theme['topic'])
            if theme['msg_type'] == msg_Image:
                if 'data' in theme:
                    ret = ImageColorTest(data, theme['data'])
                    assert ret[0], ret[1]
            elif theme['msg_type'] == msg_Imu:
                    ret = ImuTest(data, theme['data'])
                    assert ret[0], ret[1]
            else:
                print("first chunck of data for"+ theme['topic'] + ":")
                print(data.header)

        return True
    def shutdown(self):
        #if self.node == None: 
        #    self.node.destroy_node()
        #rclpy.shutdown()
        pass