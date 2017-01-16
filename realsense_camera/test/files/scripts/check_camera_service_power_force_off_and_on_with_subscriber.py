#!/usr/bin/env python
"""
@file check_camera_service_power_force_off_and_on_with_subscriber.py
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
BASEDIR = os.path.abspath(os.path.join(os.path.dirname(__file__)))

PKG = "realsense_camera"
NAME = "check_camera_service_power_force_off_and_on_with_subscriber"


class CheckCameraServicePowerForceOffAndOnWithSubscriber(unittest.TestCase):
    """
    @class CheckCameraServicePowerForceOffAndOnWithSubscriber: check reponse for power operation like
    force off and on when camera has subscriber
    """
    def setUp(self):
        '''
        @fn setUp: make sure camera is powered on
        @param self
        '''
        rospy.init_node(NAME, anonymous=True, log_level=rospy.DEBUG)
        time.sleep(6)
        output = os.popen("rosservice call /camera/driver/is_powered")
        if output.read().find('True') == -1:
            self.assertTrue(False, 'Camera is not powered on.')

    def test_force_camera_off_and_on_with_subscriber(self):
        """
        @fn test_force_camera_off_and_on_with_subscriber
         check camera power can be forced off and on when subscribers exist
        @param self
        @return
        """
        # add a subscriber to camera
        (ret, output) = commands.getstatusoutput(
            BASEDIR + '/libs/roscd_path.bash realsense_camera')
        cmd = "cd " + output + "; rosrun rviz rviz -d rviz/realsenseRvizConfiguration1.rviz"
        subprocess.Popen(cmd, stderr=subprocess.STDOUT,
                         stdout=subprocess.PIPE, shell=True,
                         preexec_fn=os.setsid)
        time.sleep(6)
        cmd = "rosservice call /camera/driver/force_power false"
        cmd_proc = subprocess.Popen(cmd, stderr=subprocess.STDOUT,
                                    stdout=subprocess.PIPE, shell=True,
                                    preexec_fn=os.setsid)
        time.sleep(2)
        output = os.popen("rosservice call /camera/driver/is_powered")
        self.assertNotEqual(-1, output.read().find('False'))
        cmd = "rosservice call /camera/driver/force_power true"
        subprocess.Popen(cmd, stderr=subprocess.STDOUT,
                         stdout=subprocess.PIPE, shell=True,
                         preexec_fn=os.setsid)
        time.sleep(2)
        output = os.popen("rosservice call /camera/driver/is_powered")
        self.assertNotEqual(-1, output.read().find('True'))
        self.assertTrue(is_log_contains_keyword(
                            LOGFILE, stop_camera_info))
        self.assertTrue(is_log_contains_keyword(
                            LOGFILE, start_camera_info))
        os.system('killall rviz')

if __name__ == '__main__':
    param_dict = get_camera_params_and_values(sys.argv)
    start_camera_info = param_dict['start_camera_info'].replace('*', ' ')
    stop_camera_info = param_dict['stop_camera_info'].replace('*', ' ')
    rostest.rosrun(PKG, NAME, CheckCameraServicePowerForceOffAndOnWithSubscriber, sys.argv)
