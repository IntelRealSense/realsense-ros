#!/usr/bin/env python
"""
@file check_camera_transforms_publish_tf.py
"""
import os
import sys
import unittest
import time
import rospy
import rostest
from rs_general.rs_general import get_camera_params_and_values, \
     is_log_contains_keyword, LOGFILE, shell_cmd_timeout

PKG = "realsense_camera"
NAME = "test_camera_transforms_publish_tf"


class CheckCameraTransformsPublishTf(unittest.TestCase):
    """
    @class CheckCameraTransformsPublishTf: check whether message exist
           or not based on disable/enable publish tf
    """
    def test_camera_transforms_publish_tf(self):
        """
        @fn test_camera_transforms_publish_tf
        @param self
        @return
        """
        rospy.init_node(NAME, anonymous=True, log_level=rospy.DEBUG)
        time.sleep(10)
        messages = expected_message.split(',')
        indicator = 0
        cmd = "rm ~/.ros/test_rostopic.log >/dev/null 2>&1"
        os.system(cmd)
        cmd = "rostopic echo /tf_static > ~/.ros/test_rostopic.log"
        shell_cmd_timeout(cmd, 5)
        for message in messages:
            if is_log_contains_keyword("~/.ros/test_rostopic.log", message):
                indicator = indicator + 1
        rospy.loginfo("***transforms publish info***: " + str(indicator))
        if indicator == len(messages) and is_log_contains_keyword(LOGFILE, publish_info):
            self.assertEqual('true', expected)
        elif indicator == 0 and not is_log_contains_keyword(LOGFILE, publish_info):
            self.assertEqual('false', expected)
        else:
            self.assertTrue(False)

if __name__ == '__main__':
    param_dict = get_camera_params_and_values(sys.argv)
    expected_message = param_dict['expected_messages']
    expected = param_dict['expected']
    publish_info = param_dict['info'].replace('*', ' ')
    rostest.rosrun(PKG, NAME, CheckCameraTransformsPublishTf, sys.argv)
