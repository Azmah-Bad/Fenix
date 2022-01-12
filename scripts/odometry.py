import rospy
from nav_msgs.msg import Odometry


def callback(Odometry):
    print(Odometry.z)


def listener():
    rospy.init_node("listener",anonymous=True)
    rospy.Subscriber("odom",Odometry,callback)
    rospy.spin()


if __name__ == '__main__':  
    listener()
