#!/usr/bin/env python
"""
@file check_librealsense_installed.py
"""
import os
import sys
import commands
import unittest
import rospy
import rostest

PKG = 'realsense_camera'
NAME = 'check_librealsense_installed'
LIB = 'librealsense.so'


class CheckLibrealsenseInstalled(unittest.TestCase):
    """
    @class CheckLibrealsenseInstalled
    """

    def setUp(self):
        """
        @fn setUp
        @param self
        @return
        """
        self.success = False

    def test_basic_librealsense_installaton(self):
        """verify that librealsense library has been installed
        @fn test_basic_librealsense_installaton
        @param self
        @return
        """
        rospy.init_node(NAME, anonymous=True, log_level=rospy.INFO)

        ros_version = commands.getoutput("rosversion -d")
        if os.path.exists('/opt/ros/' + ros_version + '/lib/' + LIB) == True \
                or os.path.exists('/opt/ros/' + ros_version +
                                  '/lib/x86_64-linux-gnu/' + LIB) == True:
            self.success = True

        self.assert_(self.success, str(self.success))

if __name__ == '__main__':
    rostest.rosrun(PKG, NAME, CheckLibrealsenseInstalled, sys.argv)
