# Copyright 2026 RealSense, Inc. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Verify vanilla `ros2 bag play` (not librealsense) delivers every image message
# byte-identical to the one stored in the .db3.

import hashlib, os, subprocess, sys, time
import pytest, rclpy
from rclpy.qos import HistoryPolicy, QoSProfile
from sensor_msgs.msg import CompressedImage

sys.path.append(os.path.abspath(os.path.dirname(__file__) + "/../utils"))
from pytest_rs_utils import get_rosbag_file_path, db3_topic_messages

DB3 = "outdoors_1color.db3"

TOPICS = [
    "/device_0/sensor_1/Color_0/image/data/compressed",
    "/device_0/sensor_0/Depth_0/image/data/compressedDepth",
    "/device_0/sensor_0/Infrared_1/image/data/compressed",
]


@pytest.mark.no_librealsense
@pytest.mark.rosbag
def test_ros2_bag_play_db3():
    assert subprocess.run(["ros2", "bag", "play", "--help"],
                          capture_output=True).returncode == 0, \
        "ros2 bag play not available (install ros-${ROS_DISTRO}-ros2bag)"
    db3 = get_rosbag_file_path(DB3)

    if not rclpy.ok():
        rclpy.init()
    node = rclpy.create_node("db3_play_subscriber")
    # KEEP_ALL avoids subscriber-side drops; hash in the callback so frames don't pile up.
    qos = QoSProfile(history=HistoryPolicy.KEEP_ALL, depth=1000)
    received = {t: [] for t in TOPICS}
    for t in TOPICS:
        node.create_subscription(
            CompressedImage, t,
            lambda m, t=t: received[t].append(hashlib.sha256(bytes(m.data)).hexdigest()),
            qos)

    proc = subprocess.Popen(["ros2", "bag", "play", db3],
                            stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    try:
        deadline = time.time() + 60
        while proc.poll() is None and time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
        drain = time.time() + 2
        while time.time() < drain:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        if proc.poll() is None:
            proc.terminate()
            try:
                proc.wait(timeout=10)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait()
        out = proc.stdout.read().decode(errors="replace") if proc.stdout else ""
        node.destroy_node()
        rclpy.shutdown()

    assert proc.returncode == 0, \
        f"ros2 bag play failed (rc={proc.returncode}):\n{out}"

    for t in TOPICS:
        src_h = [hashlib.sha256(bytes(m.data)).hexdigest()
                 for m in db3_topic_messages(db3, t, CompressedImage)]
        rx_h = received[t]
        assert rx_h and (rx_h == src_h[:len(rx_h)] or rx_h == src_h[-len(rx_h):]), \
            f"{t}: content mismatch (got {len(rx_h)} / {len(src_h)} source frames)"
