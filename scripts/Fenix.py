from mimetypes import init
import roslib
import rospy

from std_msgs.msg import Empty
from ar_track_alvar_msgs.msg import AlvarMarkers
from std_msgs.msg import Empty, String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

WAYPOINT_SPHERE = 0.1

class Fenix():
    def __init__(self):
        #Faire tous les configurations
        self.speed = 0.1
        self.land_initiated == False
        self.currentPos = {'x': 0, 'y': 0, 'z': 0}
        self.targetPos = {'x': 0, 'y': 0, 'z': 0.5}

        rospy.on_shutdown(self.shutdown)
        rospy.loginfo("Fenix Running")

        #subscribers
        rospy.Subscriber('/ar_pose_marker', AlvarMarkers, self.marker_callback)
        rospy.Subscriber("/bebop/odom", Odometry, self.odom_callback)

        #publishers
        self.pub_takeoff = rospy.Publisher('bebop/takeoff', Empty, queue_size=1)
        self.pub_land = rospy.Publisher('bebop/land', Empty, queue_size=1)
        self.pub_cmd_vel = rospy.Publisher('bebop/cmd_vel', Twist, queue_size=1)


    def shutdown(self):
        rospy.loginfo("Shutting down Fenix node.")
        land = Empty()
        self.pub_land.publish(land)
        rospy.sleep(1)

    # takeoff
    def takeoff(self):
        takeoff = Empty()
        rospy.loginfo("Going to takeoff.")
        rospy.sleep(1)
        self.pub_takeoff.publish(takeoff)
        rospy.sleep(2)

    # landing intiation
    def land(self):
        if self.land_initiated == False:
            self.land_initiated = True
            rospy.loginfo("Initiating Landing Sequence.")
            self.pub_land.publish(land)

    def move(self):
        # testing only.  Simply moves forward turns around
        vel_msg = Twist()
        vel_msg.linear.x = self.speed
        
        # move forward for 1 seconds
        seconds = 0.0
        while(seconds < 1):
            self.pub_cmd_vel.publish(vel_msg)
            rospy.sleep(0.1)
            seconds += 0.1

        # turn around

        vel_msg.linear.x = 0
        vel_msg.angular.z = 0.1

        seconds = 0.0
        while(seconds < 1):
            self.pub_cmd_vel.publish(vel_msg)
            rospy.sleep(0.1)
            seconds += 0.1

        vel_msg.linear.x = 0
        vel_msg.angular.z = 0

        self.pub_cmd_vel.publish(vel_msg)

    def marker_callback(self, data):
        if self.land_initiated:
            if not data.markers.isempty():
                for marker in data.markers:
                    vel_msg = Twist()

                    p = marker.pose.pose.position
                    print("tag: ", marker.id, "\nposition: ", p)
                    #!!!just for testing
                    if marker.id == 4:
                        vel_msg.linear.x = (marker.pose.pose.position.x - self.targetPos['x']) * self.speed
                        vel_msg.linear.y = - (marker.pose.pose.position.y - self.targetPos['y']) * self.speed
                        vel_msg.linear.z = - (marker.pose.pose.position.z - self.targetPos['z']) * self.speed

                        self.pub_cmd_vel.publish(vel_msg)


    def odom_callback(self, data):
        pass



if __name__ == '__main__':
    try:
        rospy.init_node('Fenix', anonymous=False)
        # callback to handle commands/requests
        Fenix()
        
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.logerr("Fenix node terminated.")