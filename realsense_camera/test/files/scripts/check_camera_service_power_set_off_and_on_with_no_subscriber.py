#!/usr/bin/env python
"""
@file check_camera_service_power_set_off_and_on_with_no_subscriber.py
"""
import os
import sys
import unittest
import time
import subprocess
import commands
import rospy
import rostest
from rs_general.rs_general import get_camera_params_and_values, \
     is_log_contains_keyword, LOGFILE

PKG = "realsense_camera"
NAME = "check_camera_service_power_set_off_and_on_with_no_subscriber"


class CheckCameraServicePowerSetOffAndOnWithNoSubscriber(unittest.TestCase):
    """
    @class CheckCameraServicePowerSetOffAndOnWithNoSubscriber: check reponse for power operation like
    set on/off when camera has no subscriber
    """
    def setUp(self):
        '''
        @fn setUp: make sure camera is powered on
        @param self
        '''
        rospy.init_node(NAME, anonymous=True, log_level=rospy.DEBUG)
        time.sleep(10)
        output = os.popen("rosservice call /camera/driver/is_powered")
        if output.read().find('True') == -1:
            self.assertTrue(False, 'Camera is not powered on.')

    def test_set_camera_off_and_on_with_no_subscribers(self):
        """
        @fn test_set_camera_off_and_on_with_no_subscribers
         check camera power can be set off and on when no subscribers exist
        @param self
        @return
        """
        os.system("rosservice call /camera/driver/set_power false")
        time.sleep(2)
        output = os.popen("rosservice call /camera/driver/is_powered")
        self.assertNotEqual(-1, output.read().find('False'))
        os.system("rosservice call /camera/driver/set_power true")
        time.sleep(2)
        output = os.popen("rosservice call /camera/driver/is_powered")
        self.assertNotEqual(-1, output.read().find('True'))
        self.assertTrue(is_log_contains_keyword(LOGFILE, stop_camera_info))
        self.assertTrue(is_log_contains_keyword(LOGFILE, start_camera_info))

if __name__ == '__main__':
    param_dict = get_camera_params_and_values(sys.argv)
    start_camera_info = param_dict['start_camera_info'].replace('*', ' ')
    stop_camera_info = param_dict['stop_camera_info'].replace('*', ' ')
    rostest.rosrun(PKG, NAME, CheckCameraServicePowerSetOffAndOnWithNoSubscriber, sys.argv)
