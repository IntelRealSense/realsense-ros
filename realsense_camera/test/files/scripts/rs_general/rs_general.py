#!/usr/bin/env python
"""
@file rs_general.py
"""

import os
import subprocess
import time
import string
import signal
import commands
import fileinput
from PIL import Image

# Find the latest rosout log file
BASE_LOG_DIR = (os.path.expanduser('~/.ros/log/'))
LOG_FOLDER = os.listdir(BASE_LOG_DIR)
LOG_FOLDER.sort(key=lambda fn: os.path.getmtime(BASE_LOG_DIR+fn)
                if os.path.isdir(BASE_LOG_DIR+fn) else 0)
LOGFILE = BASE_LOG_DIR+LOG_FOLDER[-1] + '/rosout.log'


def parse_camera_type(args):
    """parse args to get camera type
    @fn parse_camera_type
    @param args: all the arguments
    @return camera_type
    """
    argc = len(args)
    if argc == 0:
        return None

    for i in range(0, argc):
        if args[i] == 'camera_type' and i+1 < argc:
            return args[i+1]


def get_camera_params_and_values(args):
    """parse args to get all the camera parameters and paired values
       the args transferred from .test file, should remove 4 elements
       in args: start 1 - script name;
                last 3 - '--gtest_output', '--name', '_log';
    @fn parse_camera_type
    @param args: all the arguments
    @return param_dict: dictionary of camera params
    """
    param_dict = {}
    argc = len(args) - 4
    if argc == 0:
        return None

    params = args[1:-3]
    for i in range(0, argc, 2):
        if i+1 < argc:
            param_dict.setdefault(params[i], params[i+1])

    return param_dict


def is_log_contains_keyword(log_file, keyword):
    """check if the keyword contains in log"""
    if (not os.path.exists(os.path.expanduser(log_file)) or
            keyword is None or
            keyword == ''):
        return False
    file = open(os.path.expanduser(log_file))
    for line in file:
        if line.find(keyword) != -1:
            file.close()
            return True
    file.close()
    return False


def shell_cmd_timeout(cmd, timeout=0):
    """Execute shell command till timeout"""
    cmd_proc = subprocess.Popen(cmd, stderr=subprocess.STDOUT,
                                stdout=subprocess.PIPE, shell=True,
                                preexec_fn=os.setsid)
    pid = cmd_proc.pid
    pgid = os.getpgid(pid)
    if not cmd_proc:
        return -1, ''
    t_timeout, tick = timeout, 2
    while True:
        time.sleep(tick)
        ret = cmd_proc.poll()
        if ret is not None:
            break
        if t_timeout > 0:
            t_timeout -= tick
        if t_timeout <= 0:
            os.killpg(pgid, signal.SIGTERM)
            ret = -99999
            break
    return
