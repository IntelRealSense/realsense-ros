import os
import sys
from rs2_listener import CWaitForMessage

import rosbag
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

import subprocess


def ImageGetData(rec_filename, topic):
    bag = rosbag.Bag(rec_filename)
    bridge = CvBridge()
    all_avg = []
    ok_percent = []
    res = dict()

    for topic, msg, t in bag.read_messages(topics=topic):
        try:
            cv_image = bridge.imgmsg_to_cv2(msg, msg.encoding)
        except CvBridgeError as e:
            print(e)
            continue
        pyimg = np.asarray(cv_image)
        ok_number = (pyimg != 0).sum()
        ok_percent.append(float(ok_number) / (pyimg.shape[0] * pyimg.shape[1]))
        all_avg.append(pyimg.sum() / ok_number)

    all_avg = np.array(all_avg)
    channels = cv_image.shape[2] if len(cv_image.shape) > 2 else 1
    res['num_channels'] = channels
    res['shape'] = cv_image.shape
    res['avg'] = all_avg.mean()
    res['ok_percent'] = {'value': (np.array(ok_percent).mean()) / channels, 'epsilon': 0.01}
    res['epsilon'] = max(all_avg.max() - res['avg'], res['avg'] - all_avg.min())
    res['reported_size'] = [msg.width, msg.height, msg.step]

    return res


def ImageColorGetData(rec_filename):
    return ImageGetData(rec_filename, '/device_0/sensor_1/Color_0/image/data')


def ImageDepthGetData(rec_filename):
    return ImageGetData(rec_filename, '/device_0/sensor_0/Depth_0/image/data')


def ImageDepthInColorShapeGetData(rec_filename):
    gt_data = ImageDepthGetData(rec_filename)
    color_data = ImageColorGetData(rec_filename)
    gt_data['shape'] = color_data['shape'][:2]
    gt_data['reported_size'] = color_data['reported_size']
    gt_data['reported_size'][2] = gt_data['reported_size'][0]*2
    gt_data['ok_percent']['epsilon'] *= 3
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
        channels = list(set(data['num_channels']))
        print 'Expect %d channels. Got %d channels.' % (channels[0], gt_data['num_channels'])
        if len(channels) > 1 or channels[0] != gt_data['num_channels']:
            return False
        print 'Expected all received images to be the same shape. Got %s' % str(set(data['shape']))
        if len(set(data['shape'])) > 1:
            return False
        print 'Expected shape to be %s. Got %s' % (gt_data['shape'], list(set(data['shape']))[0])
        if (np.array(list(set(data['shape']))[0]) != np.array(gt_data['shape'])).any():
            return False
        print 'Expected header [width, height, step] to be %s. Got %s' % (gt_data['reported_size'], list(set(data['reported_size']))[0])
        if (np.array(list(set(data['reported_size']))[0]) != np.array(gt_data['reported_size'])).any():
            return False
        print 'Expect average of %.3f (+-%.3f). Got average of %.3f.' % (gt_data['avg'].mean(), gt_data['epsilon'], np.array(data['avg']).mean())
        if abs(np.array(data['avg']).mean() - gt_data['avg'].mean()) > gt_data['epsilon']:
            return False

        print 'Expect no holes percent > %.3f. Got %.3f.' % (gt_data['ok_percent']['value']-gt_data['ok_percent']['epsilon'], np.array(data['ok_percent']).mean())
        if np.array(data['ok_percent']).mean() < gt_data['ok_percent']['value']-gt_data['ok_percent']['epsilon']:
            return False

    except Exception as e:
        print 'Test Failed: %s' % e
        return False
    return True


def ImageColorTest_3epsilon(data, gt_data):
    gt_data['epsilon'] *= 3
    return ImageColorTest(data, gt_data)


def PointCloudTest(data, gt_data):
    print 'Expect image size %d(+-%d), %d. Got %d, %d.' % (gt_data['width'][0], gt_data['width'][1], gt_data['height'][0], data['width'][0], data['height'][0])
    if abs(data['width'][0] - gt_data['width'][0]) > gt_data['width'][1] or data['height'][0] != gt_data['height'][0]:
        return False
    print 'Expect average position of %s (+-%.3f). Got average of %s.' % (gt_data['avg'][0][:3], gt_data['epsilon'][0], data['avg'][0][:3])
    if abs(data['avg'][0][:3] - gt_data['avg'][0][:3]).max() > gt_data['epsilon'][0]:
        return False
    print 'Expect average color of %s (+-%.3f). Got average of %s.' % (gt_data['avg'][0][3:], gt_data['epsilon'][1], data['avg'][0][3:])
    if abs(data['avg'][0][3:] - gt_data['avg'][0][3:]).max() > gt_data['epsilon'][1]:
        return False

    return True


test_types = {'vis_avg': {'listener_theme': 'colorStream',
                          'data_func': ImageColorGetData,
                          'test_func': ImageColorTest},
              'depth_avg': {'listener_theme': 'depthStream',
                            'data_func': ImageDepthGetData,
                            'test_func': ImageColorTest},
              'no_file': {'listener_theme': 'colorStream',
                          'data_func': lambda x: None,
                          'test_func': lambda x, y: not ImageColorTest(x, y)},
              'pointscloud_avg': {'listener_theme': 'pointscloud',
                          'data_func': lambda x: {'width': [776534, 2300], 'height': [1], 'avg': [np.array([ 1.28251814, -0.15839984, 4.82235184, 65, 88, 95])], 'epsilon': [0.02, 2]},
                          'test_func': PointCloudTest},
              'align_depth_ir1': {'listener_theme': 'alignedDepthInfra1',
                                  'data_func': ImageDepthGetData,
                                  'test_func': ImageColorTest},
              'align_depth_color': {'listener_theme': 'alignedDepthColor',
                                   'data_func': ImageDepthInColorShapeGetData,
                                   'test_func': ImageColorTest_3epsilon},
              'depth_avg_decimation': {'listener_theme': 'depthStream',
                                   'data_func': ImageDepthGetData_decimation,
                                   'test_func': ImageColorTest},
              'align_depth_ir1_decimation': {'listener_theme': 'alignedDepthInfra1',
                                  'data_func': ImageDepthGetData,
                                  'test_func': ImageColorTest},
              }


def run_test(test, listener_res):
    # gather ground truth with test_types[test['type']]['data_func'] and recording from test['rosbag_filename']
    # return results from test_types[test['type']]['test_func']
    test_type = test_types[test['type']]
    gt_data = test_type['data_func'](test['params']['rosbag_filename'])
    return test_type['test_func'](listener_res[test_type['listener_theme']], gt_data)


def print_results(results):
    title = 'TEST RESULTS'
    headers = ['test name', 'score']
    col_0_width = max([len(headers[0])] + [len(test[0]) for test in results]) + 1
    col_1_width = max([len(headers[1]), len('OK'), len('FAILED')]) + 1
    total_width = col_0_width + col_1_width
    print ('{:^%ds}'%total_width).format(title)
    print '-'*total_width
    print ('{:<%ds}{:>%ds}' % (col_0_width, col_1_width)).format('test name', 'score')
    print '-'*(col_0_width-1) + ' '*2 + '-'*(col_1_width-1)
    print '\n'.join([('{:<%ds}{:>%ds}' % (col_0_width, col_1_width)).format(test[0], 'OK' if test[1] else 'FAILED') for test in results])
    print


def run_tests(tests):
    msg_params = {'timeout_secs': 5}
    results = []
    params_strs = set([test['params_str'] for test in tests])
    for params_str in params_strs:
        rec_tests = [test for test in tests if test['params_str'] == params_str]
        themes = [test_types[test['type']]['listener_theme'] for test in rec_tests]
        msg_retriever = CWaitForMessage(msg_params)
        print 'Starting ROS'
        p_wrapper = subprocess.Popen(['roslaunch', 'realsense2_camera', 'rs_from_file.launch'] + params_str.split(' '), stdout=None, stderr=None)
        listener_res = msg_retriever.wait_for_messages(themes)
        print 'Killing ROS'
        p_wrapper.terminate()
        p_wrapper.wait()
        print 'DONE'

        for test in rec_tests:
            try:
                res = run_test(test, listener_res)
            except Exception as e:
                print 'Test %s Failed: %s' % (test['name'], e)
                res = False
            results.append([test['name'], res])

    return results

def main():
    all_tests = [{'name': 'vis_avg_1', 'type': 'no_file', 'params': {'rosbag_filename': '/home/non_existent_file.txt'}},
                 # {'name': 'vis_avg_2', 'type': 'vis_avg', 'params': {'rosbag_filename': '/home/doronhi/Downloads/checkerboard_30cm.bag'}},
                 {'name': 'vis_avg_2', 'type': 'vis_avg', 'params': {'rosbag_filename': './records/outdoors.bag'}},
                 {'name': 'depth_avg_1', 'type': 'depth_avg', 'params': {'rosbag_filename': './records/outdoors.bag'}},
                 {'name': 'depth_w_cloud_1', 'type': 'depth_avg', 'params': {'rosbag_filename': './records/outdoors.bag', 'enable_pointcloud': 'true'}},
                 {'name': 'points_cloud_1', 'type': 'pointscloud_avg', 'params': {'rosbag_filename': './records/outdoors.bag', 'enable_pointcloud': 'true'}},
                 {'name': 'align_depth_color_1', 'type': 'align_depth_color', 'params': {'rosbag_filename': './records/outdoors.bag', 'align_depth': 'true'}},
                 {'name': 'align_depth_ir1_1', 'type': 'align_depth_ir1', 'params': {'rosbag_filename': './records/outdoors.bag', 'align_depth': 'true'}},
                 {'name': 'depth_decimation_1', 'type': 'align_depth_ir1_decimation', 'params': {'rosbag_filename': './records/outdoors.bag', 'filters': 'decimation', 'align_depth': 'true'}},
                 {'name': 'depth_avg_decimation_1', 'type': 'depth_avg_decimation', 'params': {'rosbag_filename': './records/outdoors.bag', 'filters': 'decimation'}},
                 {'name': 'align_depth_ir1_decimation_1', 'type': 'align_depth_ir1_decimation', 'params': {'rosbag_filename': './records/outdoors.bag', 'filters': 'decimation', 'align_depth': 'true'}}

                 ]

    # Normalize parameters:
    for test in all_tests:
        test['params']['rosbag_filename'] = os.path.abspath(test['params']['rosbag_filename'])
        test['params_str'] = ' '.join([key + ':=' + test['params'][key] for key in sorted(test['params'].keys())])

    if len(sys.argv) < 2 or '--help' in sys.argv or '/?' in sys.argv:
        print 'USAGE:'
        print '------'
        print 'rs2_test.py --all | <test_name> [<test_name> [...]]'
        print
        print 'Available tests are:'
        print '\n'.join([test['name'] for test in all_tests])
        exit(-1)

    if '--all' in sys.argv[1:]:
        tests_to_run = all_tests
    else:
        tests_to_run = [test for test in all_tests if test['name'] in sys.argv[1:]]

    results = run_tests(tests_to_run)
    print_results(results)

    res = int(all([result[1] for result in results])) - 1
    print 'exit (%d)' % res
    exit(res)

if __name__ == '__main__':
    main()