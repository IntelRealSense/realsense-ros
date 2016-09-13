import subprocess
import os
import sys


def compile_pack(list1):
    for com in list1:
        proc = subprocess.Popen(com,
        stdout = subprocess.PIPE,
        stderr = subprocess.PIPE, shell=True)  
        (out, err) = proc.communicate()
        if (proc.returncode !=0):
            return 1
    return 0


os.chdir(r"../../../..")
make_msgs = 'catkin_make -DCATKIN_WHITELIST_PACKAGES="realsense_msgs"'
make_srvs = 'catkin_make -DCATKIN_WHITELIST_PACKAGES="realsense_srvs"'
make_person = 'catkin_make -DCATKIN_WHITELIST_PACKAGES="realsense_person_tracking"'
make_test = 'catkin_make -DCATKIN_WHITELIST_PACKAGES="realsense_person_tracking_test"'

list_to_make = [make_msgs, make_srvs, make_person, make_test]

        


sys.exit(compile_pack(list_to_make))


