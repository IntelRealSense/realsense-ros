#!/usr/bin/env python

import rospkg
import subprocess
import os

r = rospkg.RosPack()
path = r.get_path('realsense2_description')


def run_xacro_in_file(filename):
    assert(filename != "")
    assert(subprocess.check_output(["xacro", "--inorder", "tests/{}".format(filename)],
                                   cwd=path))


def test_files():
    for _, _, filenames in os.walk(os.path.join(path, "tests")):
        for file in filenames:
            if file.endswith(".xacro"):
                yield run_xacro_in_file, file
