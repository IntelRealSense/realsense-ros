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
import rclpy
import logging
import threading
from enum import Enum
from rclpy.node import Node
from rclpy import qos
from rcl_interfaces.msg import ParameterType, Parameter, ParameterValue
from rcl_interfaces.srv import SetParameters
from realsense2_camera_msgs.msg import Metadata
import json
from rclpy.executors import MultiThreadedExecutor


class StreamType(Enum):
    ACCEL = 'accel'
    COLOR = 'color'
    DEPTH = 'depth'
    GYRO = 'gyro'
    IR1 = 'infra1'
    IR2 = 'infra2'


class KPIStatus(Enum):
    PASS = 'pass'
    FAIL = 'fail'


frame_collect = False
raw_data_frames = \
    {
        StreamType.COLOR: [],
        StreamType.DEPTH: [],
        StreamType.IR1: [],
        StreamType.IR2: [],
        StreamType.ACCEL: [],
        StreamType.GYRO: []
    }

test_time = int(time.time())
raw_data_path = os.path.join(os.getcwd(), f"raw_data_{int(test_time)}.csv")
test_results_data_path = os.path.join(os.getcwd(), f"test_results_{int(test_time)}.csv")
test_results_path = os.path.join(os.getcwd(), f"test_results_{int(test_time)}.csv")
test_log_path = os.path.join(os.getcwd(), f"test_log_{int(test_time)}.txt")
logging.basicConfig(filename=test_log_path, level=logging.DEBUG,
                    format='%(asctime)s:%(levelname)s:%(message)s')
logging.getLogger().addHandler(logging.StreamHandler())


def save_raw_data(current_iteration, f_list):
    f = open(raw_data_path, "a")
    if current_iteration == 0:
        f.write(
            "Iteration,Stream type,Frame Index,HW Timestamp\n")
    stream_types = f_list.keys()
    for st in stream_types:
        for frame in f_list[st]:
            f.write(f"{current_iteration},{st.value},{frame['index']},{frame['hw_timestamp']}\n")


def save_test_results(current_iteration, test_results_data):
    f = open(test_results_data_path, "a")
    if current_iteration == 0:
        f.write(
            "Iteration,Stream type,arrived_frames,frame_drops_count,seq_frame_drops_events,max_seq_frame_drop,"
            "first_seq_drop_index,max_seq_drop_index,frame_drops_percentage,frame_arrived_status,"
            "frame_drops_percentage_status,seq_frame_drops_status\n")
    stream_types = test_results_data.keys()
    for st in stream_types:
        stream_result_data = test_results_data[st]
        f.write(
            f"{current_iteration},{st.value},"
            f"{stream_result_data['arrived_frames']},"
            f"{stream_result_data['frame_drops_count']},{stream_result_data['seq_frame_drops_events']},"
            f"{stream_result_data['max_seq_frame_drop']},"
            f"{stream_result_data['first_seq_drop_index']},{stream_result_data['max_seq_drop_index']},"
            f"{stream_result_data['frame_drops_percentage']},{stream_result_data['frame_arrived_status'].value},"
            f"{stream_result_data['frame_drops_percentage_status'].value},"
            f"{stream_result_data['seq_frame_drops_status'].value}\n")


def calc_frame_drops(f_list, color_depth_fps, accel_fps, gyro_fps, frame_drops_percentage_spec):
    stream_fps_map = {
        StreamType.COLOR: color_depth_fps,
        StreamType.DEPTH: color_depth_fps,
        StreamType.IR1: color_depth_fps,
        StreamType.IR2: color_depth_fps,
        StreamType.GYRO: gyro_fps,
        StreamType.ACCEL: accel_fps,
    }
    frame_drops_result = dict()
    stream_types = f_list.keys()
    for st in stream_types:
        is_first_seq = True

        arrived_frames = len(f_list[st])
        frame_drops_count = 0
        seq_frame_drops_events = 0
        max_seq_frame_drop = 0
        first_seq_drop_index = -1
        max_seq_drop_index = -1
        frame_drops_percentage = 0
        frame_arrived_status = KPIStatus.PASS
        frame_drops_percentage_status = KPIStatus.PASS
        seq_frame_drops_status = KPIStatus.PASS

        for index, frame in enumerate(f_list[st]):
            frame_index = frame['index']
            frame_hw_ts = frame['hw_timestamp']
            if index != 0:
                prev_frame_hw_ts = f_list[st][index - 1]['hw_timestamp']
                expected_delta = 1000 / float(stream_fps_map[st])
                ts_delta = frame_hw_ts - prev_frame_hw_ts
                # logging.debug(f'{st.value} - index: {frame_index} -'
                #               f' hw delta: {float(ts_delta) / 1000:.2f}[ms] -'
                #               f' expected hw delta: {expected_delta:.2f}[ms]')
                ts_delta = float(ts_delta) / 1000
                if 1.5 * expected_delta < ts_delta < 2.5 * expected_delta:
                    frame_drops_count += 1
                elif ts_delta >= 2.5 * expected_delta:
                    seq_drops_num = int(round((ts_delta / expected_delta) - 1))
                    frame_drops_count += seq_drops_num
                    seq_frame_drops_events += 1
                    if is_first_seq:
                        first_seq_drop_index = frame_index
                        is_first_seq = False
                    if seq_drops_num > max_seq_frame_drop:
                        max_seq_frame_drop = seq_drops_num
                        max_seq_drop_index = frame_index
                    logging.warning(f'{seq_drops_num} sequential {st.value} frame drops at index: {frame_index} - '
                                    f'Actual hw delta: {ts_delta:.2f}[ms], Expected hw delta: {expected_delta:.2f}[ms]')

        if arrived_frames > 0:
            total_frames = arrived_frames + frame_drops_count
            frame_drops_percentage = 100 * (frame_drops_count / total_frames)

        if arrived_frames == 0:
            frame_arrived_status = KPIStatus.FAIL
        if seq_frame_drops_events > 0:
            seq_frame_drops_status = KPIStatus.FAIL
        if frame_drops_percentage > frame_drops_percentage_spec:
            frame_drops_percentage_status = KPIStatus.FAIL

        stream_frame_drops_result = \
            {
                'arrived_frames': arrived_frames,
                'frame_drops_count': frame_drops_count,
                'seq_frame_drops_events': seq_frame_drops_events,
                'max_seq_frame_drop': max_seq_frame_drop,
                'first_seq_drop_index': first_seq_drop_index,
                'max_seq_drop_index': max_seq_drop_index,
                'frame_drops_percentage': frame_drops_percentage,
                'frame_arrived_status': frame_arrived_status,
                'frame_drops_percentage_status': frame_drops_percentage_status,
                'seq_frame_drops_status': seq_frame_drops_status,
            }
        frame_drops_result[st] = stream_frame_drops_result

    return frame_drops_result


def metadata_cb(msg, stream_type):
    global raw_data_frames, frame_collect
    if frame_collect:
        aa = json.loads(msg.json_data)
        # os.system('clear')
        md_dict = dict(aa.items())
        hw_timestamp = md_dict['hw_timestamp']
        frame_index = md_dict['frame_number']
        raw_data_frames[stream_type].append({'index': frame_index, 'hw_timestamp': hw_timestamp})
        # logging.debug(f'Frame type: {stream_type.value}, Frame index: {frame_index}, Frame HW TS: {hw_timestamp}')

def metadata_cb1(msg, stream_type):
    global raw_data_frames, frame_collect
    if frame_collect:
        aa = json.loads(msg.json_data)
        # os.system('clear')
        md_dict = dict(aa.items())
        print(md_dict)
        hw_timestamp = md_dict['hw_timestamp']
        frame_index = md_dict['frame_number']
        raw_data_frames[stream_type].append({'index': frame_index, 'hw_timestamp': hw_timestamp})
        # logging.debug(f'Frame type: {stream_type.value}, Frame index: {frame_index}, Frame HW TS: {hw_timestamp}')

def accel_metadata_cb(msg):
    metadata_cb(msg, StreamType.ACCEL)


def color_metadata_cb(msg):
    metadata_cb(msg, StreamType.COLOR)


def depth_metadata_cb(msg):
    metadata_cb(msg, StreamType.DEPTH)


def gyro_metadata_cb(msg):
    metadata_cb1(msg, StreamType.GYRO)


def infra1_metadata_cb(msg):
    metadata_cb(msg, StreamType.IR1)


def infra2_metadata_cb(msg):
    metadata_cb(msg, StreamType.IR2)


def wait_for_future(future: rclpy.task.Future, timeout):
    st = time.time()
    while timeout > time.time() - st:
        if future.done():
            return True
        time.sleep(0.3)
    future.cancel()
    return False


def service_call(node, service, service_msg, request):
    if not node:
        logging.error("Controller node is not running")
        raise RuntimeError("Controller node is not running")
    logging.debug(f"Calling service: {service} [{service_msg}] with request: {request}")
    client = node.create_client(service_msg, service)
    if not client.wait_for_service(timeout_sec=5):
        logging.error("Service is not not available")
        raise TimeoutError("Service is not not available")
    future = client.call_async(request)
    if not wait_for_future(future, 100):
        logging.error("Request for option timed out")
        raise TimeoutError("Request for option timed out")
    result = future.result()
    logging.debug(f"Service call result: {result}")
    return result


def set_parameters(node, request: SetParameters.Request):
    result = service_call(node, '/camera/set_parameters', SetParameters, request)
    value = result.results[0]
    if not value.successful:
        logging.error(f"Failed to enable stream: {value.reason}")
        raise RuntimeError(f"Failed to enable stream: {value.reason}")


def disable_stream(node, stream_type):
    req = SetParameters.Request()
    query, val = f'enable_{stream_type}', False
    req.parameters.append(Parameter(name=query,
                                    value=ParameterValue(type=ParameterType.PARAMETER_BOOL,
                                                         bool_value=val)))
    set_parameters(node, req)


def enable_stream(node, stream_type):
    req = SetParameters.Request()
    query, val = f'enable_{stream_type}', True
    req.parameters.append(Parameter(name=query,
                                    value=ParameterValue(type=ParameterType.PARAMETER_BOOL,
                                                         bool_value=val)))
    set_parameters(node, req)

def enable_all_streams(node):
    global iter, raw_data_frames, frame_collect
    frame_collect = True
    logging.info(f'===================== iteration {iter} =====================')
    logging.info(f'Starting {StreamType.COLOR.value} stream')
    enable_stream(node, StreamType.COLOR.value)
    logging.info(f'Starting {StreamType.DEPTH.value} stream')
    enable_stream(node, StreamType.DEPTH.value)
    logging.info(f'Starting {StreamType.IR1.value} stream')
    enable_stream(node, StreamType.IR1.value)
    logging.info(f'Starting {StreamType.ACCEL.value} stream')
    enable_stream(node, StreamType.ACCEL.value)
    logging.info(f'Starting {StreamType.GYRO.value} stream')
    enable_stream(node, StreamType.GYRO.value)


def disable_all_streams(node):
    global raw_data_frames, frame_collect
    frame_collect = False
    logging.info(f'Stopping {StreamType.COLOR.value} stream')
    disable_stream(node, StreamType.COLOR.value)
    logging.info(f'Stopping {StreamType.DEPTH.value} stream')
    disable_stream(node, StreamType.DEPTH.value)
    logging.info(f'Stopping {StreamType.IR1.value} stream')
    disable_stream(node, StreamType.IR1.value)
    logging.info(f'Stopping {StreamType.ACCEL.value} stream')
    disable_stream(node, StreamType.ACCEL.value)
    logging.info(f'Stopping {StreamType.GYRO.value} stream')
    disable_stream(node, StreamType.GYRO.value)

def timer_callback():
    global iter, raw_data_frames, frame_collect, test_node1
    iter = iter + 1
    disable_all_streams(test_node1)

    # Save frames raw data
    logging.info('Saving frames raw data')
    save_raw_data(iter, raw_data_frames)

    # Save test results
    logging.info('Saving test results')
    test_result = calc_frame_drops(raw_data_frames, color_depth_fps=30, accel_fps=200, gyro_fps=200,
                                    frame_drops_percentage_spec=5)
    save_test_results(iter, test_result)

    # init test data
    raw_data_frames = {StreamType.COLOR: [], StreamType.DEPTH: [], StreamType.IR1: [], StreamType.IR2: [],
                        StreamType.ACCEL: [],
                        StreamType.GYRO: []}
    
    enable_all_streams(test_node1)

def main():
    global iter, raw_data_frames, frame_collect, test_node1
    iterations = 2000
    stream_duration = 10
    iter = 0
    rclpy.init()
    test_node = Node('test_node')
    test_node1 = Node('test_node1')
    custom_qos= qos.qos_profile_sensor_data
    #custom_qos.depth = 10

    accel_sub = test_node.create_subscription(Metadata, '/accel/metadata', accel_metadata_cb,
                                         custom_qos)
    color_sub = test_node.create_subscription(Metadata, '/color/metadata', color_metadata_cb,
                                         custom_qos)
    depth_sub = test_node.create_subscription(Metadata, '/depth/metadata', depth_metadata_cb,
                                         custom_qos)
    gyro_sub = test_node.create_subscription(Metadata, '/gyro/metadata', gyro_metadata_cb, custom_qos)
    ir1_sub = test_node.create_subscription(Metadata, '/infra1/metadata', infra1_metadata_cb,
                                       custom_qos)
    ir2_sub = test_node.create_subscription(Metadata, '/infra2/metadata', infra2_metadata_cb,
                                       custom_qos)

    #enable_all_streams(test_node)
    timer = test_node1.create_timer(10.0, timer_callback)

    executor = MultiThreadedExecutor()
    executor.add_node(test_node)
    executor.add_node(test_node1)
    executor.spin()
    executor.shutdown()
    
    logging.info('Test Done')
    test_node.destroy_node()
    test_node1.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
