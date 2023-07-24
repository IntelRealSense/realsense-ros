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
import launch.actions

import launch_ros.actions
import launch_pytest
import rclpy
from rclpy import qos
from rclpy.node import Node
from rclpy.utilities import ok

import ctypes
import struct
import requests


from sensor_msgs.msg import Image as msg_Image
from sensor_msgs.msg import Imu as msg_Imu
from sensor_msgs.msg import PointCloud2 as msg_PointCloud2
from sensor_msgs.msg import CameraInfo as msg_CameraInfo
from realsense2_camera_msgs.msg import Extrinsics as msg_Extrinsics
from realsense2_camera_msgs.msg import Metadata as msg_Metadata
from sensor_msgs_py import point_cloud2 as pc2
import tf2_ros

import json
import rs_launch

'''
Copied from the old code in scripts folder
'''
from importRosbag.importRosbag import importRosbag

import tempfile
import os
import requests
class RosbagManager(object):
    def __new__(cls):
        if not hasattr(cls, 'instance'):
            cls.instance = super(RosbagManager, cls).__new__(cls)
            cls.init(cls.instance)
        return cls.instance
    def init(self):
        self.rosbag_files = {
                "outdoors_1color.bag":"https://librealsense.intel.com/rs-tests/TestData/outdoors_1color.bag",
                "D435i_Depth_and_IMU_Stands_still.bag":"https://librealsense.intel.com/rs-tests/D435i_Depth_and_IMU_Stands_still.bag"
                }
        self.rosbag_location = os.getenv("HOME") + "/realsense_records/" 
        print(self.rosbag_location)
        if not os.path.exists(self.rosbag_location):
            os.mkdir(self.rosbag_location)
        for key in self.rosbag_files:
            file_path = self.rosbag_location + key
            if not os.path.isfile(file_path): 
                print(" downloading from " + self.rosbag_files[key])
                r = requests.get(self.rosbag_files[key], allow_redirects=True)
                open(file_path, 'wb').write(r.content)
                print(file_path + " downloaded")
            else:
                print(file_path + " exists")

    def get_rosbag_path(self, filename):
        if filename in self.rosbag_files:
            return self.rosbag_location + "/" + filename
rosbagMgr = RosbagManager()
def get_rosbag_file_path(filename):
    path = rosbagMgr.get_rosbag_path(filename)
    assert path, "No rosbag file found :"+filename 
    return path


def CameraInfoGetData(rec_filename, topic):
    data = importRosbag(rec_filename, importTopics=[topic], log='ERROR', disable_bar=True)[topic]
    data =  {k.lower(): v for k, v in data.items()}
    data['distortionmodel'] = "plumb_bob"
    data['k'] = data['k'].reshape(-1)
    data['r'] = data['r'].reshape(-1)
    data['p'] = data['p'].reshape(-1)
    return data

def CameraInfoColorGetData(rec_filename):
    return CameraInfoGetData(rec_filename, '/device_0/sensor_1/Color_0/info/camera_info')


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
        channels = pyimg.shape[2] if len(pyimg.shape) > 2 else 1
        ok_percent.append(float(ok_number) / (pyimg.shape[0] * pyimg.shape[1] * channels))
        all_avg.append(pyimg.sum() / ok_number)
    
    all_avg = np.array(all_avg)

    res['num_channels'] = channels
    res['shape'] = pyimg.shape
    res['avg'] = all_avg.mean()
    res['ok_percent'] = np.array(ok_percent).mean()
    res['epsilon'] = max(all_avg.max() - res['avg'], res['avg'] - all_avg.min())
    res['reported_size'] = [pyimg.shape[1], pyimg.shape[0], pyimg.shape[1]*pyimg.dtype.itemsize*channels]

    return res

def ImageColorGetData(rec_filename):
    return ImageGetData(rec_filename, '/device_0/sensor_1/Color_0/image/data')


def ImageDepthGetData(rec_filename):
    return ImageGetData(rec_filename, '/device_0/sensor_0/Depth_0/image/data')

def ImageInfra1GetData(rec_filename):
    return ImageGetData(rec_filename, '/device_0/sensor_0/Infrared_1/image/data')

def ImageDepthInColorShapeGetData(rec_filename):
    gt_data = ImageDepthGetData(rec_filename)
    color_data = ImageColorGetData(rec_filename)
    gt_data['shape'] = color_data['shape'][:2]
    gt_data['reported_size'] = color_data['reported_size']
    gt_data['reported_size'][2] = gt_data['reported_size'][0]*2
    gt_data['epsilon'] *= 4 #4 instead of 3 due to size difference between Depth and color?
    return gt_data

def ImageDepthInInfra1ShapeGetData(rec_filename):
    gt_data = ImageDepthGetData(rec_filename)
    infra1_data = ImageInfra1GetData(rec_filename)
    gt_data['shape'] = infra1_data['shape'][:2]
    gt_data['reported_size'] = infra1_data['reported_size']
    gt_data['reported_size'][2] = gt_data['reported_size'][0]*2
    gt_data['epsilon'] *= 3
    return gt_data

def ImageDepthGetData_decimation(rec_filename):
    gt_data = ImageDepthGetData(rec_filename)
    gt_data['shape'] = [x/2 for x in gt_data['shape']]
    gt_data['reported_size'] = [x/2 for x in gt_data['reported_size']]
    gt_data['epsilon'] *= 3
    return gt_data

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

        msg = 'Expect no holes percent > %.3f. Got %.3f.' % (gt_data['ok_percent']-gt_data['epsilon'], np.array(data['ok_percent']).mean())
        print (msg)
        if np.array(data['ok_percent']).mean() < gt_data['ok_percent']-gt_data['epsilon']:
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

def PointCloudTest(data, gt_data):
    width = np.array(data['width']).mean()
    height = np.array(data['height']).mean()
    msg = 'Expect image size %d(+-%d), %d. Got %d, %d.' % (gt_data['width'][0], gt_data['width'][1], gt_data['height'][0], width, height)
    print (msg)
    if abs(width - gt_data['width'][0]) > gt_data['width'][1] or height != gt_data['height'][0]:
        return False, msg
    mean_pos = np.array([xx[:3] for xx in data['avg']]).mean(0)
    msg = 'Expect average position of %s (+-%.3f). Got average of %s.' % (gt_data['avg'][0][:3], gt_data['epsilon'][0], mean_pos)
    print (msg)
    if abs(mean_pos - gt_data['avg'][0][:3]).max() > gt_data['epsilon'][0]:
        return False, msg
    mean_col = np.array([xx[3:] for xx in data['avg']]).mean(0)
    msg = 'Expect average color of %s (+-%.3f). Got average of %s.' % (gt_data['avg'][0][3:], gt_data['epsilon'][1], mean_col)
    print (msg)
    if abs(mean_col - gt_data['avg'][0][3:]).max() > gt_data['epsilon'][1]:
        return False, msg

    return True, ''

def staticTFTest(data, gt_data):
    for couple in gt_data.keys():
        if data[couple] is None:
            msg = 'Tf is None for couple %s' % '->'.join(couple)
            return False, msg
        temp = data[couple].translation
        np_trans = np.array([temp.x, temp.y, temp.z])
        temp = data[couple].rotation
        np_rot = np.array([temp.x, temp.y, temp.z, temp.w])
        if any(abs(np_trans - gt_data[couple][0]) > 1e-5) or \
           any(abs(np_rot - gt_data[couple][1]) > 1e-5):
           msg = 'Tf is changed for couple %s' % '->'.join(couple)
           return False, msg
    return True, ''

def cameraInfoTest(data, gt_data):
    msg = ''
    if data.height != gt_data.height:
        msg = 'CameraInfo height is not matching'
        return False, msg
    if data.width != gt_data.width:
        msg = 'CameraInfo width is not matching'
        return False, msg
    if data.distortion_model != gt_data.distortion_model:
        msg = 'CameraInfo distortion_model is not matching'
        return False, msg
    if not np.all(np.equal(data.d , gt_data.d)):
        msg = 'CameraInfo d is not matching'
        return False, msg
    if len(data.k) != len(gt_data.k):
        msg = 'k sizes are not matching in extrinsics'
        return False, msg
    for count in range(len(data.k)):
        if abs(data.k[count] - gt_data.k[count]) > 1e-5:
            msg = 'k at %s are not matching values are %s and %s', (count, data.k[count] , gt_data.k[count])
            return False, msg

    if len(data.r) != len(gt_data.r):
        msg = 'r sizes are not matching in extrinsics'
        return False, msg
    for count in range(len(data.r)):
        if abs(data.r[count] - gt_data.r[count]) > 1e-5:
            msg = 'r at %s are not matching values are %s and %s', (count, data.r[count] , gt_data.r[count])
            return False, msg
    if len(data.p) != len(gt_data.p):
        msg = 'p sizes are not matching in extrinsics'
        return False, msg
    for count in range(len(data.p)):
        if abs(data.p[count] - gt_data.p[count]) > 1e-5:
            msg = 'p at %s are not matching values are %s and %s', (count, data.p[count] , gt_data.p[count])
            return False, msg

    if data.binning_x != gt_data.binning_x:
        msg = 'CameraInfo binning_x is not matching'
        return False, msg
    if data.binning_y != gt_data.binning_y:
        msg = 'CameraInfo binning_y is not matching'
        return False, msg
    if data.roi != gt_data.roi:
        msg = 'CameraInfo roi is not matching'
        return False, msg
    return True, ""

def extrinsicsTest(data, gt_data):
    msg = ''
    if len(data.translation) != len(gt_data.translation):
        msg = 'translation sizes are not matching in extrinsics'
        return False, msg
    if len(data.rotation) != len(gt_data.rotation):
        msg = 'rotation sizes are not matching in extrinsics'
        return False, msg
    for count in range(len(data.translation)):
        if abs(data.translation[count] - gt_data.translation[count]) > 1e-5:
            msg = 'translation at %s are not matching values are %s and %s', (count, data.translation[count] , gt_data.translation[count])
            return False, msg
    for count in range(len(data.rotation)):
        if abs(data.rotation[count] - gt_data.rotation[count]) > 1e-5:
            msg = 'rotation at %s are not matching values are %s and %s', (count, data.rotation[count] , gt_data.rotation[count])
            return False, msg
    return True, ""

def metadatTest(data, gt_data):
    jdata = json.loads(data.json_data)
    gt_jdata = json.loads(gt_data.json_data)
    if jdata['frame_number'] != gt_jdata['frame_number']:
        msg = 'Frame no not matching: ' + str(jdata['frame_number']) + " and " + str(gt_jdata['frame_number'])
        return False, msg
    if jdata['clock_domain'] != gt_jdata['clock_domain']:
        msg = 'clock_domain not matching: ' + str(jdata['clock_domain']) + " and " + str(gt_jdata['clock_domain'])
        return False, msg
    if jdata['frame_timestamp'] != gt_jdata['frame_timestamp']:
        msg = 'frame_timestamp not matching: ' + str(jdata['frame_timestamp']) + " and " + str(gt_jdata['frame_timestamp'])
        return False, msg
    '''
    frame counter is not populated by rsobag reader in libRealsense it seems
    '''
    '''
    if jdata['frame_counter'] != gt_jdata['frame_counter']:
        msg = 'frame_counter not matching: ' + str(jdata['frame_counter']) + " and " + str(gt_jdata['frame_counter'])
        return False, msg
    '''
    if jdata['time_of_arrival'] != gt_jdata['time_of_arrival']:
        msg = 'time_of_arrival not matching: ' + str(jdata['time_of_arrival']) + " and " + str(gt_jdata['time_of_arrival'])
        return False, msg
    return True, ""
    

def pc2_to_xyzrgb(point):
    # Thanks to Panos for his code used in this function.
    point = list(point)
    x, y, z = point[:3]
    rgb = point[3]

    # cast float32 to int so that bitwise operations are possible
    s = struct.pack('>f', rgb)
    i = struct.unpack('>l', s)[0]
    # you can get back the float value by the inverse operations
    pack = ctypes.c_uint32(i).value
    r = (pack & 0x00FF0000) >> 16
    g = (pack & 0x0000FF00) >> 8
    b = (pack & 0x000000FF)
    return x, y, z, r, g, b


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
This function returns a launch description with a single rs node instance built based on the parameter
passed, this is similar to launch_descr_with_parameters. However this delays the launch of the rs node
so as to give preparation time for the test node. This useful when the preprocessing of the test data takes
a lot of time due to the data size itself 
'''
@launch_pytest.fixture
def delayed_launch_descr_with_parameters(request):
    changed_params = request.param
    params = get_default_params()
    for key, value in changed_params.items():
        if key in params.keys():
            params[key] = value 
    if  'camera_name' not in changed_params:
        params['camera_name'] = 'camera_with_params'
    period = 2.0
    if 'delay_ms' in changed_params.keys():
        period = changed_params['delay_ms']/1000
    first_node = get_rs_node_description(params['camera_name'], params)
    return LaunchDescription([launch.actions.TimerAction(
            actions = [
        first_node,], period=period)
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
        self.frame_counter = {}

    def wait_for_node(self, node_name, timeout=8.0):
        start = time.time()
        flag = False
        print('Waiting for node... ' + node_name)
        while time.time() - start < timeout:
            print(node_name + ": waiting for the node to come up")
            flag = node_name in self.get_node_names()
            if flag:
                return True, ""
            time.sleep(timeout/5)
        return False, "Timed out waiting for "+ str(timeout)+  "seconds"
    def create_subscription(self, msg_type, topic , data_type, store_raw_data):
        super().create_subscription(msg_type, topic , self.rsCallback(topic,msg_type, store_raw_data), data_type)
        self.data[topic] = deque()
        self.frame_counter[topic] = 0
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
            #print(data.header)
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
                #print("pyimg from callback:")
                #print(pyimg)
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
                        import quaternion
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
            elif msg_type == msg_PointCloud2:
                func_data = dict()
                func_data.setdefault('frame_counter', 0)
                func_data.setdefault('avg', [])
                func_data.setdefault('size', [])
                func_data.setdefault('width', [])
                func_data.setdefault('height', [])
                # until parsing pointcloud is done in real time, I'll use only the first frame.
                func_data['frame_counter'] = self.frame_counter[topic]
                self.frame_counter[topic] += 1
                #print("frame_counter "+str(func_data['frame_counter']))
                if func_data['frame_counter'] == 1:
                    # Known issue - 1st pointcloud published has invalid texture. Skip 1st frame.
                    return
                    #pass
                try:
                    points = np.array([pc2_to_xyzrgb(pp) for pp in pc2.read_points(data, skip_nans=True, field_names=("x", "y", "z", "rgb")) if pp[0] > 0])
                except Exception as e:
                    print(e)
                    return
                func_data['avg'].append(points.mean(0))
                func_data['size'].append(len(points))
                func_data['width'].append(data.width)
                func_data['height'].append(data.height)
                self.data[topic].append(func_data)
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
        #cmd = "pip list | grep -i quat && pip show quaternion"
        #os.system(cmd)
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
        timeout = 25.0
        print('Waiting for topic... ' )
        flag = False
        while (time.time() - start) < timeout:
            rclpy.spin_once(self.node, timeout_sec=1)
            print('Spun once... ' )
            all_found = True 
            for theme in themes:
                if theme['expected_data_chunks'] > int(self.node.get_num_chunks(theme['topic'])):
                    all_found = False
                    break
            if all_found == True:
                flag =True
                break
        else:
            print("Timed out waiting for", timeout, "seconds" )
            return False, "run_test timedout"
        return flag,""

    def spin_for_time(self,wait_time):
        start = time.time()
        print('Waiting for topic... ' )
        flag = False
        while time.time() - start < wait_time:
            rclpy.spin_once(self.node)
            print('Spun once... ' )
 
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
                self.flag =False,e.message
            else:
                print(e)
                self.flag =False,e
            
        return self.flag 
    '''
    Please override and use your own process_data if the default check is not suitable.
    Please also store_raw_data parameter in the themes as True, if you want the
    callback to store the data as is.  
    '''
    def process_data(self, themes):
        for theme in themes:
            data = self.node.pop_first_chunk(theme['topic'])
            if 'data' not in theme:
                print('No data to compare for ' + theme['topic'])
                #print(data)
            elif theme['msg_type'] == msg_Image:
                ret = ImageColorTest(data, theme['data'])
                assert ret[0], ret[1]
            elif theme['msg_type'] == msg_Imu:
                ret = ImuTest(data, theme['data'])
                assert ret[0], ret[1]
            elif theme['msg_type'] == msg_PointCloud2:
                ret = PointCloudTest(data, theme['data'])
                assert ret[0], ret[1]
            elif theme['msg_type'] == msg_CameraInfo:
                ret = cameraInfoTest(data, theme['data'])
                assert ret[0], ret[1]
            elif theme['msg_type'] == msg_Extrinsics:
                ret = extrinsicsTest(data, theme['data'])
                assert ret[0], ret[1]
            elif theme['msg_type'] == msg_Metadata:
                ret = metadatTest(data, theme['data'])
                assert ret[0], ret[1]
            else:
                print("first chunck of data for"+ theme['topic'] + ":")
                print(data.header)

        return True
    def shutdown(self):
        #if self.node == None: 
        #    self.node.destroy_node()
        rclpy.shutdown()
        pass
