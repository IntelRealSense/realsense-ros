#!/usr/bin/env python
"""
@file check_camera_info_matrix.py
"""
import os
import sys
import subprocess
import signal
import unittest
import re
import time
import rospy
import rostest
from rs_general.rs_general import parse_camera_type

PKG = 'realsense_camera'
NAME = 'matrix_check'
PROJECTION = 'projection_matrix'
PROJECTION_LENGHT = 12
DISTORTION = 'distortion_coefficients'
DISTORTION_LENGHT = 5
ROTATION = 'rotation_identity'
ROTATION_LENGHT = 9
DEPTH = 'depth'
COLOR = 'color'
IR = 'ir'
IR2 = 'ir2'
FISHEYE = 'fisheye'
CAMERAS = ['R200', 'F200', 'SR300', 'ZR300']


class CheckCameraInfoMatrix(unittest.TestCase):
    """
    @class CheckCameraInfoMatrix
    """

    def setUp(self):
        """
        @fn setUp
        @param self
        @return
        """
        self.assertIn(camera_type, CAMERAS)

    def get_matrix(self, string, matrix_type):
        """search values of matrix type from string,
           and organize into matrix array
        @fn get_matrix
        @param self
        @param string
        @param matrix_type
        @return matrix_array: matrix values
        """
        if (matrix_type == PROJECTION and not string.find('P: ') == -1) \
                or (matrix_type == DISTORTION and
                    not string.find('D: ') == -1) \
                or (matrix_type == ROTATION and
                    not string.find('R: ') == -1):
            values = re.findall(r'([0-9]+\.[0-9]+)', string)
            matrix_array = map(float, values)
            return matrix_array
        else:
            return ''

    def is_zero(self, value):
        """determine whether the value equal to 0
        @fn is_zero
        @param self
        @param value
        @return True: equal to 0; False: not equal to 0
        """
        if (value > -0.00000001) and (value < 0.00000001):
            return True
        else:
            return False

    def get_camera_info(self, stream_type, matrix_type):
        """run rostopic to print camera info, get matrix values from stdout
        @fn get_camera_info
        @param self
        @param stream_type
        @param matrix_type
        @return matrix_array
        """
        cmd = 'rostopic echo /camera/' + stream_type + '/camera_info'
        print "cmd = ", cmd
        process = subprocess.Popen(cmd, stdout=subprocess.PIPE,
                                   stderr=subprocess.PIPE, shell=True)
        while True:
            buff = process.stdout.readline()
            if buff == '' and process.poll is not None:
                break
            array = self.get_matrix(buff, matrix_type)
            if array != '':
                os.kill(process.pid, signal.SIGKILL)
                return array
        return ''

    def verify_projection_matrix(self, matrix, stream_type):
        """verify projection matrix values, adapt each stream type
        @fn verify_projection_matrix
        @param self
        @param matrix
        @param stream_type
        return
        """
        if matrix == '' or len(matrix) != PROJECTION_LENGHT \
                or stream_type == '':
            self.assertTrue(False,
                            'Not get correct projection matrix or stream type.')

        if stream_type == DEPTH:
            if (self.is_zero(matrix[3]) or
                    self.is_zero(matrix[7]) or
                    self.is_zero(matrix[11])):
                self.assertTrue(False, 'Projection matrix value is incorrect.')
        elif (stream_type == COLOR or
                stream_type == IR or
                stream_type == IR2 or
                stream_type == FISHEYE):
            if (not self.is_zero(matrix[3]) or
                    not self.is_zero(matrix[7]) or
                    not self.is_zero(matrix[11])):
                self.assertTrue(False, 'Projection matrix value is incorrect.')
        else:
            self.assertTrue(False,
                            'Stream type "' + stream_type + '" is invalid.')

    def verify_rotation_matrix(self, matrix):
        """verify rotation matrix values, adapt each stream type
        @fn verify_rotation_matrix
        @param self
        @param matrix
        return
        """
        if matrix == '' or len(matrix) != ROTATION_LENGHT:
            self.assertTrue(False, 'Not get correct rotation matrix.')

        for i in range(0, len(matrix)):
            if i == 0 or i == 4 or i == 8:
                if matrix[i] < 0.99999999 or matrix[i] > 1.00000001:
                    self.assertTrue(False, 'Rotation matrix value is incorrect.')
            else:
                if not self.is_zero(matrix[i]):
                    self.assertTrue(False, 'Rotation matrix value is incorrect.')

    def verify_distortion_matrix(self, matrix, stream_type):
        """verify distortion matrix values, adapt each stream type
        @fn verify_distortion_matrix
        @param self
        @param matrix
        @param stream_type
        return
        """
        if camera_type == '' or stream_type == '' or matrix == '' \
                or len(matrix) != DISTORTION_LENGHT:
            self.assertTrue(False,
                            'Not get correct distortion matrix or stream type.')

        if camera_type == 'R200' or camera_type == 'ZR300':
            if stream_type == COLOR:
                for i in range(0, len(matrix)):
                    if i == 4:
                        self.assertTrue(self.is_zero(matrix[i]), 'Distortion \
                                        matrix value of color is incorrect.')
                    else:
                        self.assertFalse(self.is_zero(matrix[i]), 'Distortion \
                                         matrix value of color is incorrect.')
            elif (stream_type == DEPTH or
                  stream_type == IR or
                  stream_type == IR2):
                for i in range(0, len(matrix)):
                    self.assertTrue(self.is_zero(matrix[i]), 'Distortion \
                                    matrix value of (depth/IR/IR2) is incorrect.')
            elif stream_type == FISHEYE:
                for i in range(0, len(matrix)):
                    if i == 0:
                        self.assertFalse(self.is_zero(matrix[i]), 'Distortion \
                                         matrix value of fisheye is incorrect.')
                    else:
                        self.assertTrue(self.is_zero(matrix[i]), 'Distortion \
                                        matrix value of fisheye is incorrect.')
            else:
                self.assertTrue(False,
                                'Stream type "' + stream_type + '" is invalid.')
        elif camera_type == 'F200' or camera_type == 'SR300':
            if stream_type == COLOR:
                for i in range(0, len(matrix)):
                    self.assertTrue(self.is_zero(matrix[i]), 'Distortion \
                                    matrix value of color is incorrect.')
            elif stream_type == DEPTH or stream_type == IR:
                for i in range(0, len(matrix)):
                    self.assertFalse(self.is_zero(matrix[i]), 'Distortion \
                                     matrix value of (depth/IR) is incorrect.')
            else:
                self.assertTrue(False,
                                'Stream type "' + stream_type + '" is invalid.')

    def test_camera_info_check_projection_matrix(self):
        """check projection matrix values from camera info
        @fn test_check_projection_matrix
        @param self
        @return
        """
        rospy.init_node(NAME, anonymous=True, log_level=rospy.INFO)

        # wait for nodelet to be started
        time.sleep(10)

        self.verify_projection_matrix(
            self.get_camera_info(DEPTH, PROJECTION), DEPTH)
        time.sleep(0.5)
        self.verify_projection_matrix(
            self.get_camera_info(COLOR, PROJECTION), COLOR)
        time.sleep(0.5)
        self.verify_projection_matrix(
            self.get_camera_info(IR, PROJECTION), IR)
        time.sleep(0.5)
        if camera_type == 'R200' or camera_type == 'ZR300':
            self.verify_projection_matrix(
                self.get_camera_info(IR2, PROJECTION), IR2)
            time.sleep(0.5)
        if camera_type == 'ZR300':
            self.verify_projection_matrix(
                self.get_camera_info(FISHEYE, PROJECTION), FISHEYE)
            time.sleep(0.5)

    def test_camera_info_check_rotation_identity_matrix(self):
        """check rotation identity matrix values from camera info
        @fn test_check_rotation_identity_matrix
        @param self
        @return
        """
        rospy.init_node(NAME, anonymous=True, log_level=rospy.INFO)

        # wait for nodelet to be started
        time.sleep(10)

        self.verify_rotation_matrix(self.get_camera_info(DEPTH, ROTATION))
        time.sleep(0.5)
        self.verify_rotation_matrix(self.get_camera_info(COLOR, ROTATION))
        time.sleep(0.5)
        self.verify_rotation_matrix(self.get_camera_info(IR, ROTATION))
        time.sleep(0.5)
        if camera_type == 'R200' or camera_type == 'ZR300':
            self.verify_rotation_matrix(self.get_camera_info(IR2, ROTATION))
            time.sleep(0.5)
        if camera_type == 'ZR300':
            self.verify_rotation_matrix(self.get_camera_info(FISHEYE, ROTATION))
            time.sleep(0.5)

    def test_camera_info_check_distortion_coefficients(self):
        """check distortion coefficients matrix values from camera info
        @fn test_check_distortion_coefficients
        @param self
        @return
        """
        rospy.init_node(NAME, anonymous=True, log_level=rospy.INFO)

        # wait for nodelet to be started
        time.sleep(10)

        self.verify_distortion_matrix(
            self.get_camera_info(DEPTH, DISTORTION), DEPTH)
        time.sleep(0.5)
        self.verify_distortion_matrix(
            self.get_camera_info(COLOR, DISTORTION), COLOR)
        time.sleep(0.5)
        self.verify_distortion_matrix(
            self.get_camera_info(IR, DISTORTION), IR)
        time.sleep(0.5)
        if camera_type == 'R200' or camera_type == 'ZR300':
            self.verify_distortion_matrix(
                self.get_camera_info(IR2, DISTORTION), IR2)
            time.sleep(0.5)
        if camera_type == 'ZR300':
            self.verify_distortion_matrix(
                self.get_camera_info(FISHEYE, DISTORTION), FISHEYE)
            time.sleep(0.5)

if __name__ == '__main__':
    camera_type = parse_camera_type(sys.argv)
    rostest.rosrun(PKG, NAME, CheckCameraInfoMatrix, sys.argv)
