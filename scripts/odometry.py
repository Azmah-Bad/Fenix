import rospy
from nav_msgs.msg import Odometry
import time


def callback(Odometry):
    print("*****************")
    print("x: ", Odometry.pose.pose.position.x)
    print("y: ", Odometry.pose.pose.position.y)
    print("z: ", Odometry.pose.pose.position.z)


def listener():
    rospy.init_node("listener", anonymous=True)
    rospy.Subscriber("/bebop/odom", Odometry, callback)
    rospy.spin()


if __name__ == '__main__':
    listener()
