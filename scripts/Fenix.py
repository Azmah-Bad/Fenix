import roslib
import rospy

from std_msgs.msg import Empty
from ar_track_alvar_msgs.msg import AlvarMarkers
from std_msgs.msg import Empty, String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

WAYPOINT_SPHERE = 0.1
AR_TAGS = {  # id: position of the ar tag
    0: (2.52, 1.4, 3.1),
    1: (1.68, 1.47, 3.1),
    2: (2.1, 1.66, 3.1),
    3: (3.12, 1.17, 3.1),
    4: (3.6, 0.8, 3.1),
    5: (2.0, 0.87, 3.1),
    6: (1.23, 1.75, 3.1),
    7: (0.8, 1.25, 3.1),
    8: (0.25, 1.85, 3.1),
    9: (4.0, 1.82, 3.1),
    10: (4.44, 1.35, 3.1),
    11: (4.83, 1.96, 3.1),
    12: (5.12, 1.05, 3.1),
    }


class Fenix:
    def __init__(self):
        # Faire tous les configurations
        self.speed = 0.1
        self.land_initiated = False
        self.currentPos = {'x': 0, 'y': 0, 'z': 0}
        self.targetPos = {'x': 0, 'y': 0, 'z': 2.5}

        rospy.on_shutdown(self.shutdown)
        rospy.loginfo("Fenix Running")

        # subscribers
        rospy.Subscriber('/bebop/ar_pose_marker', AlvarMarkers, self.marker_callback)
        rospy.Subscriber("/bebop/odom", Odometry, self.odom_callback)

        # publishers
        self.pub_takeoff = rospy.Publisher('bebop/takeoff', Empty, queue_size=1)
        self.pub_land = rospy.Publisher('bebop/land', Empty, queue_size=1)
        self.pub_cmd_vel = rospy.Publisher('bebop/cmd_vel', Twist, queue_size=1)

        self.land()
        #self.move()
        self.odom_data = []

    def shutdown(self):
        rospy.loginfo("Shutting down Fenix node...")
        land = Empty()
        self.pub_land.publish(land)
        rospy.sleep(1)
        rospy.loginfo("Landing completed.")

    # takeoff
    def takeoff(self):
        takeoff = Empty()
        rospy.loginfo("Going to takeoff...")
        rospy.sleep(1)
        self.pub_takeoff.publish(takeoff)
        rospy.sleep(2)
        rospy.loginfo("Takeoff completed !")

    # landing initiation
    def land(self):
        if not self.land_initiated:
            self.land_initiated = True
            rospy.loginfo("Initiating landing sequence...")
            self.pub_land.publish(land)
            rospy.loginfo("Landing completed !")

    def move(self):
        # testing only.  Simply moves forward turns around
        vel_msg = Twist()
        vel_msg.linear.x = self.speed

        # move forward for 1 seconds
        seconds = 0.0
        while seconds < 1:
            self.pub_cmd_vel.publish(vel_msg)
            rospy.sleep(0.1)
            seconds += 0.1

        # turn around

        vel_msg.linear.x = 0
        vel_msg.angular.z = 0.1

        seconds = 0.0
        while seconds < 1:
            self.pub_cmd_vel.publish(vel_msg)
            rospy.sleep(0.1)
            seconds += 0.1

        vel_msg.linear.x = 0
        vel_msg.angular.z = 0

        self.pub_cmd_vel.publish(vel_msg)

    def marker_callback(self, data):
        self.currentPos['x'] = self.currentPos['y'] = self.currentPos['z'] = 0
        if not self.land_initiated:
            if len(data.markers) > 0:
                vel_msg = Twist()

                for marker in data.markers:

                    p = marker.pose.pose.position
                    print "\ntag:", marker.id
                    print "position:\n", p

                    artag_id = marker.id
                    self.currentPos['x'] += AR_TAGS[artag_id][0] - marker.pose.pose.position.x
                    self.currentPos['y'] += AR_TAGS[artag_id][1] - marker.pose.pose.position.y
                    self.currentPos['z'] += AR_TAGS[artag_id][2] - marker.pose.pose.position.z

                for key in self.currentPos:
                    self.currentPos[key] = self.currentPos[key]/len(data.markers)

                print "Current position: ", self.currentPos

                #!!!just for testing
                #vel_msg.linear.x = (self.targetPos['x'] - self.currentPos['x']) * self.speed
                #vel_msg.linear.y = (self.targetPos['y'] - self.currentPos['y']) * self.speed
                vel_msg.linear.z = (self.targetPos['z'] - self.currentPos['z']) * self.speed

                self.pub_cmd_vel.publish(vel_msg)

    def odom_callback(self, data):
        pass


if __name__ == '__main__':
    try:
        rospy.init_node('Fenix', anonymous=False)
        fenix = Fenix()
        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.logerr("Fenix node terminated.")
