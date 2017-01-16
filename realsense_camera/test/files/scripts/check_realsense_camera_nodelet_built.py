#!/usr/bin/env python
"""
@file check_realsense_camera_nodelet_built.py
"""
import os
import sys
import commands
import unittest
import rospy
import rostest

PKG = 'realsense_camera'
NAME = 'check_realsense_camera_nodelet_built'
LIB = 'librealsense_camera_nodelet.so'


class CheckRealsenseCameraNodeletBuilt(unittest.TestCase):
    """
    @class CheckRealsenseCameraNodeletBuilt
    """

    def setUp(self):
        """
        @fn setUp
        @param
        @return
        """
        self.success = False

    def test_basic_realsense_camera_wrapper_installation(self):
        """verify that realsense_camera has been built and installed
        @fn test_basic_realsense_camera_wrapper_installation
        @param
        return
        """
        rospy.init_node(NAME, anonymous=True, log_level=rospy.INFO)

        lib_paths = os.environ["LD_LIBRARY_PATH"].split(":")
        for path in lib_paths:
            if os.path.exists(path + '/' + LIB) == True:
                self.success = True

        self.assert_(self.success, str(self.success))

if __name__ == '__main__':
    rostest.rosrun(PKG, NAME, CheckRealsenseCameraNodeletBuilt, sys.argv)
