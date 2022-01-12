import roslib
import rospy
import time
from std_msgs.msg import Empty, UInt8

#from bebop_msg.msg import CommonCommonStateWifiSignalChanged


def test():
    pubTakeoff = rospy.Publisher('/bebop/takeoff', Empty, queue_size = 1)
    pubLand = rospy.Publisher('/bebop/land', Empty, queue_size = 1)
    pubFlip = rospy.Publisher('/bebop/flip', UInt8, queue_size = 1)
    rospy.init_node('test', anonymous=True)
    m = UInt8()
    m.data = 0
    if not rospy.is_shutdown():
        time.sleep(1)
        print("Going to takeoff")
        pubTakeoff.publish()
        #time.sleep(5)
        #print("Going to flip")
        #pubFlip.publish(m)
        time.sleep(5)
        print("Going to land")
        pubLand.publish()

if __name__ == '__main__':
    try:
        test()
    except rospy.ROSInterruptException:
        raise
