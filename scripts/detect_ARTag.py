import roslib
import rospy
from std_msgs.msg import Empty
from ar_track_alvar_msgs.msg import AlvarMarkers


import time


def alvar_callback(msg):
    for marker in msg.markers:
        print(marker.id)


rospy.init_node('test', anonymous=True)
time.sleep(1)
sub_alvar = rospy.Subscriber(
    "/ar_pose_marker", AlvarMarkers, alvar_callback, queue_size=1)
rospy.spin()
