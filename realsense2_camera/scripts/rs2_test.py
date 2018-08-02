import os
import sys
from rs2_listener import CWaitForMessage

import rosbag
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

import subprocess


def ImageColorGetData(rec_filename):
    bag = rosbag.Bag(rec_filename)
    bridge = CvBridge()
    all_avg = []
    res = dict()

    for topic, msg, t in bag.read_messages(topics='/device_0/sensor_1/Color_0/image/data'):
        try:
            cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)
            continue
        pyimg = np.asarray(cv_image)
        all_avg.append(pyimg.mean())

    all_avg = np.array(all_avg)
    (rows, cols, channels) = cv_image.shape
    res['num_channels'] = channels
    res['avg'] = all_avg.mean()
    res['epsilon'] =  max(all_avg.max() - res['avg'], res['avg'] - all_avg.min())

    return res


def ImageColorTest(data, gt_data):
    # check that all data['num_channels'] are the same as gt_data['num_channels'] and that avg value of all
    # images are within epsilon of gt_data['avg']
    try:
        channels = list(set(data['num_channels']))
        print 'Expect %d channels. Got %d channels.' % (channels[0], gt_data['num_channels'])
        if len(channels) > 1 or channels[0] != gt_data['num_channels']:
            return False
        print 'Expect avarage of %.3f. Got avarage of %.3f.' % (np.array(data['avg']).mean(), gt_data['avg'].mean())
        if abs(np.array(data['avg']) - gt_data['avg']).max() > gt_data['epsilon']:
            return False
    except Exception as e:
        print 'Test Failed: %s' % e
        return False
    return True


test_types = {'vis_avg': {'listener_theme': 'colorStream',
                          'data_func': ImageColorGetData,
                          'test_func': ImageColorTest},
              'no_file': {'listener_theme': 'colorStream',
                          'data_func': lambda x: None,
                          'test_func': lambda x, y: not ImageColorTest(x, y)},
              }


def run_test(test, listener_res):
    # gather ground truth with test_types[test['type']]['data_func'] and recording from test['rec_filename']
    # return results from test_types[test['type']]['test_func']
    test_type = test_types[test['type']]
    gt_data = test_type['data_func'](test['rec_filename'])
    return test_type['test_func'](listener_res[test_type['listener_theme']], gt_data)


def print_results(results):
    print '{:^20s}'.format('TEST RESULTS')
    print '-'*20
    print '{:<10s}{:>10s}'.format('test name', 'score')
    print '-'*9 + ' '*2 + '-'*9
    print '\n'.join(['{:<10s}{:>10s}'.format(test[0], 'OK' if test[1] else 'FAILED') for test in results])
    print


def run_tests(tests):
    msg_params = {'timeout_secs': 5}
    results = []
    rec_filenames = set([os.path.abspath(test['rec_filename']) for test in tests])
    for rec in rec_filenames:
        rec_tests = [test for test in tests if os.path.abspath(test['rec_filename']) == rec]
        themes = [test_types[test['type']]['listener_theme'] for test in rec_tests]
        msg_retriever = CWaitForMessage(msg_params)
        print 'Starting ROS'
        p_wrapper = subprocess.Popen(['roslaunch', 'realsense2_camera', 'rs_from_file.launch', 'rosbag_filename:=%s' % rec], stdout=None, stderr=None)
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
    all_tests = [{'name': 'vis_avg_1', 'type': 'no_file', 'rec_filename': '/home/non_existent_file.txt'},
                 # {'name': 'vis_avg_2', 'type': 'vis_avg', 'rec_filename': '/home/doronhi/Downloads/checkerboard_30cm.bag'},
                 {'name': 'vis_avg_2', 'type': 'vis_avg', 'rec_filename': './records/outdoors.bag'}]

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