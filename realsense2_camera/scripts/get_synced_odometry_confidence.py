import rospy
import message_filters
from nav_msgs.msg import Odometry
from realsense2_camera.msg import Confidence

def callback(odometry, confidence):
  print("sync")
  print(odometry)
  print("")
  print(confidence)
  print("\n")

rospy.init_node("message_filter")

odom_sub = message_filters.Subscriber('/camera/odom/sample', Odometry)
confidence_sub = message_filters.Subscriber('/camera/odom/confidence', Confidence)

ts = message_filters.TimeSynchronizer([odom_sub, confidence_sub], 1)
ts.registerCallback(callback)
rospy.spin()
