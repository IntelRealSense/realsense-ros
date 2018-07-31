import os
import sys
import rosbag


def get_message(filename, wanted_topic, wanted_seq):
    bag = rosbag.Bag(filename)

    for topic, msg, t in bag.read_messages(topics=wanted_topic):
        if msg.header.seq < wanted_seq:
            continue
        break

    if msg.header.seq > wanted_seq:
        return None

    return topic, msg, t


def main():
    if len(sys.argv) < 2 or '--help' in sys.argv or '/?' in sys.argv:
        print 'USAGE:'
        print '------'
        print 'parse_bag_file.py bag_filename [Options]'
        print
        print 'Application checks function get_message() - retrieving specific message from bagfile.'
        print 'example: parse_bag_file.py ~/home/user/Downloads/checkerboard_30cm.bag -t /device_0/sensor_0/Depth_0/image/data -s 58250'
        print ''

        print 'bag_filename: path to valid bag file.'
        print
        print '[Options:]'
        print '-t <topic>'
        print '-s <sequential number>'
        exit(-1)

    wanted_topic = None
    wanted_seq = None

    filename = sys.argv[1]
    for idx in range(2, len(sys.argv)):
        if sys.argv[idx] == '-t':
            wanted_topic = sys.argv[idx+1]
        if sys.argv[idx] == '-s':
            wanted_seq = sys.argv[idx + 1]

    if not all([wanted_topic, wanted_seq]):
        raise Exception('Not enough parameters to describe a message')

    if not os.access(filename, os.R_OK):
        raise Exception('%s is not a readable file.' % filename)

    message = get_message(filename, wanted_topic, wanted_seq)
    if message:
        print 'Found message with timestamp: %s' % message[2]
    else:
        print 'Found no matching message'


if __name__ == '__main__':
    main()
