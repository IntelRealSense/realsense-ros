#!/bin/bash

# Copyright (c) 2018, PlusOne Robotics Inc. All rights reserved.

# This is a convenience script (hopefully temporary) to install dependency of https://github.com/intel-ros/realsense.
# Codes taken from https://github.com/intel-ros/realsense/blob/c5ea27245967e0938f7d10384f4b7279e01000b4/.travis.yml
# When used on CI sudo is not usually needed but it is most likely needed when used on your local computer.

echo "deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo $(lsb_release -sc) main" | tee /etc/apt/sources.list.d/realsense-public.list
add-apt-repository "deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo $(lsb_release -sc) main"
apt-get update -qq
apt-get install librealsense2-dkms --allow-unauthenticated -y 
apt-get install librealsense2-dev --allow-unauthenticated -y
