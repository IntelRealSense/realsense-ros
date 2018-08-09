import sys
import time
import rospy
from sensor_msgs.msg import Image as msg_Image
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
import inspect



class CWaitForMessage:
    def __init__(self, params={}):
        self.result = None

        self.break_timeout = False
        self.timeout = params.get('timeout_secs', -1)
        self.seq = params.get('seq', -1)
        self.time = params.get('time', None)
        self.node_name = params.get('node_name', 'rs2_listener')
        self.bridge = CvBridge()

        self.themes = {'depthStream': {'topic': '', 'callback': self.imageDepthCallback, 'msg_type': msg_Image},
                       'colorStream': {'topic': '/camera/color/image_raw', 'callback': self.imageColorCallback, 'msg_type': msg_Image}}

        self.func_data = dict()

    def imageColorCallback(self, data):
        self.prev_time = time.time()
        func_name = inspect.stack()[0][3]
        theme_name = [key for key, value in self.themes.items() if func_name in value['callback'].__name__][0]
        self.func_data[theme_name].setdefault('avg', [])
        self.func_data[theme_name].setdefault('num_channels', [])

        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
            return
        (rows, cols, channels) = cv_image.shape
        pyimg = np.asarray(cv_image)
        self.func_data[theme_name]['avg'].append(pyimg.mean())
        self.func_data[theme_name]['num_channels'].append(channels)

    def imageDepthCallback(self, data):
        pass

    def wait_for_message(self, params):
        topic = params['topic']
        print 'connect to ROS with name: %s' % self.node_name
        rospy.init_node(self.node_name, anonymous=True)

        rospy.loginfo('Subscribing on topic: %s' % topic)
        self.sub = rospy.Subscriber(topic, msg_Image, self.callback)

        self.prev_time = time.time()
        break_timeout = False
        while not any([rospy.core.is_shutdown(), break_timeout, self.result]):
            rospy.rostime.wallsleep(0.5)
            if self.timeout > 0 and time.time() - self.prev_time > self.timeout:
                break_timeout = True
                self.sub.unregister()

        return self.result

    @staticmethod
    def unregister_all(registers):
        for test_name in registers:
            rospy.loginfo('Un-Subscribing test %s' % test_name)
            registers[test_name]['sub'].unregister()

    def wait_for_messages(self, themes):
        # tests_params = {<name>: {'callback', 'topic', 'msg_type', 'internal_params'}}
        self.func_data = dict([[theme_name, {}] for theme_name in themes])

        print 'connect to ROS with name: %s' % self.node_name
        rospy.init_node(self.node_name, anonymous=True)
        for theme_name in themes:
            theme = self.themes[theme_name]
            rospy.loginfo('Subscribing %s on topic: %s' % (theme_name, theme['topic']))
            self.func_data[theme_name]['sub'] = rospy.Subscriber(theme['topic'], theme['msg_type'], theme['callback'])

        self.prev_time = time.time()
        break_timeout = False
        while not any([rospy.core.is_shutdown(), break_timeout]):
            rospy.rostime.wallsleep(0.5)
            if self.timeout > 0 and time.time() - self.prev_time > self.timeout:
                break_timeout = True
                self.unregister_all(self.func_data)

        return self.func_data

    def callback(self, data):
        rospy.loginfo('Got message. Seq %d, secs: %d, nsecs: %d' % (data.header.seq, data.header.stamp.secs, data.header.stamp.nsecs))

        self.prev_time = time.time()
        if any([self.seq > 0 and data.header.seq >= self.seq,
                self.time and data.header.stamp.secs == self.time['secs'] and data.header.stamp.nsecs == self.time['nsecs']]):
            self.result = data
            self.sub.unregister()



def main():
    if len(sys.argv) < 2 or '--help' in sys.argv or '/?' in sys.argv:
        print 'USAGE:'
        print '------'
        print 'rs2_listener.py <topic> [Options]'
        print 'example: rs2_listener.py /camera/color/image_raw --time 1532423022.044515610 --timeout 3'
        print ''
        print 'Application subscribes on <topic>, wait for the first message matching [Options].'
        print 'When found, prints the timestamp.'
        print
        print '[Options:]'
        print '-s <sequential number>'
        print '--time <secs.nsecs>'
        print '--timeout <secs>'
        exit(-1)

    # wanted_topic = '/device_0/sensor_0/Depth_0/image/data'
    # wanted_seq = 58250

    wanted_topic = sys.argv[1]
    msg_params = {'topic': wanted_topic}

    for idx in range(2, len(sys.argv)):
        if sys.argv[idx] == '-s':
            msg_params['seq'] = int(sys.argv[idx + 1])
        if sys.argv[idx] == '--time':
            msg_params['time'] = dict(zip(['secs', 'nsecs'], [int(part) for part in sys.argv[idx + 1].split('.')]))
        if sys.argv[idx] == '--timeout':
            msg_params['timeout_secs'] = int(sys.argv[idx + 1])

    msg_retriever = CWaitForMessage(msg_params)
    res = msg_retriever.wait_for_message()

    rospy.loginfo('Got message: %s' % res.header)


if __name__ == '__main__':
    main()

