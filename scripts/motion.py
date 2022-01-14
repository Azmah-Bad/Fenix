import roslib
import rospy
import time
from std_msgs.msg import Empty, UInt8
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

#from bebop_msg.msg import CommonCommonStateWifiSignalChanged

currentPos = {'x': 0, 'y': 0, 'z': 0}
targetPos = {'x': 0.2, 'y': 0.2, 'z': 0.2}

STRENGTH = 0.2
WAYPOINT_SPHERE = 0.1

"""
def PID_controller(currentPos, targetPos):
    err_func = {x: targetPos.x - currentPos.x, y: targetPos.y
                - currentPos.y, z: targetPos.z - currentPos.z}
"""


def get_direction(currentPos, targetPos):
    twist = Twist()
    twist.linear.x = 0
    twist.linear.y = 0
    twist.linear.z = 0

    if currentPos != targetPos:
        if currentPos.x > targetPos.x:
            twist.linear.x = STRENGTH
        else:
            twist.linear.x = - STRENGTH

        if currentPos.y > targetPos.y:
            twist.linear.y = STRENGTH
        else:
            twist.linear.y = - STRENGTH

        if currentPos.z > targetPos.z:
            twist.linear.z = STRENGTH
        else:
            twist.linear.z = - STRENGTH

    return(twist)

def odom_callback(Odometry):
    currentPos.x = Odometry.x
    currentPos.y = Odometry.y
    currentPos.z = Odometry.z



def on_target(currentPos, targetPos):
    if abs(currentPos.x-targetPos.x) >= WAYPOINT_SPHERE:
        return False

    elif abs(currentPos.y-targetPos.y) >= WAYPOINT_SPHERE:
        return False

    elif abs(currentPos.z-targetPos.z) >= WAYPOINT_SPHERE:
        return False
    return True


def move():
    pubTakeoff = rospy.Publisher('/bebop/takeoff', Empty, queue_size=1)
    pubLand = rospy.Publisher('/bebop/land', Empty, queue_size=1)
    pubPilot = rospy.Publisher('/bebop/cmd_vel', Twist, queue_size=1)

    rospy.Subscriber("odom", Odometry, odom_callback)

    rospy.init_node('motion', anonymous=True)

    if not rospy.is_shutdown():
        time.sleep(1)
        print("Going to takeoff")
        pubTakeoff.publish()

        while not on_target:
            pubPilot.publish(get_direction(currentPos, targetPos))

        print("Going to land")
        pubLand.publish()


if __name__ == '__main__':
    try:
        move()
    except rospy.ROSInterruptException:
        raise
