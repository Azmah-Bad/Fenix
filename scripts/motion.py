import roslib
import rospy
import time
from std_msgs.msg import Empty, UInt8
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

#from bebop_msg.msg import CommonCommonStateWifiSignalChanged

currentPos = {'x': 0, 'y': 0, 'z': 0}
targetPos = {'x': 0, 'y': 0, 'z': 1}

STRENGTH = 0.2
WAYPOINT_SPHERE = 0.1

"""
def PID_controller(currentPos, targetPos):
    err_func = {x: targetPos['x'] - currentPos['x'], y: targetPos['x']
                - currentPos['y'], z: targetPos['z'] - currentPos['z']}
"""


def get_direction(currentPos, targetPos):
    print('GET DIRECTION')
    twist = Twist()
    twist.linear.x = 0
    twist.linear.y = 0
    twist.linear.z = 0

    if currentPos != targetPos:
        if currentPos['x'] < targetPos['x']:
            twist.linear.x = STRENGTH
        else:
            twist.linear.x = - STRENGTH

        if currentPos['y'] < targetPos['x']:
            twist.linear.y = STRENGTH
        else:
            twist.linear.y = - STRENGTH

        if currentPos['z'] < targetPos['z']:
            twist.linear.z = STRENGTH
        else:
            twist.linear.z = - STRENGTH

    return(twist)


def odom_callback(Odometry):
    currentPos['x'] = Odometry.pose.pose.position.x
    currentPos['y'] = Odometry.pose.pose.position.y
    currentPos['z'] = Odometry.pose.pose.position.z


def on_target(currentPos, targetPos):
    if abs(currentPos['x']-targetPos['x']) >= WAYPOINT_SPHERE:
        return False

    elif abs(currentPos['y']-targetPos['x']) >= WAYPOINT_SPHERE:
        return False

    elif abs(currentPos['z']-targetPos['z']) >= WAYPOINT_SPHERE:
        return False
    return True


def move():
    pubTakeoff = rospy.Publisher('/bebop/takeoff', Empty, queue_size=1)
    pubLand = rospy.Publisher('/bebop/land', Empty, queue_size=1)
    pubPilot = rospy.Publisher('/bebop/cmd_vel', Twist, queue_size=1)

    rospy.init_node('motion', anonymous=True)

    rospy.Subscriber("/bebop/odom", Odometry, odom_callback)

    if not rospy.is_shutdown():
        time.sleep(2)
        print("Going to takeoff")
        pubTakeoff.publish()

        time.sleep(5)
        print("Going to mooove")
        while not on_target(currentPos, targetPos):
            print('not on target')
            pubPilot.publish(get_direction(currentPos, targetPos))

        print('on target')    
        print("Going to land")
        pubLand.publish()



if __name__ == '__main__':
    try:
        move()
    except rospy.ROSInterruptException:
        raise
