#!/usr/bin/env python3

import rospy
​
# msg needed for /scan.
from sensor_msgs.msg import LaserScan
​
# msgs needed for /cmd_vel.
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3

# minimum distance from wall
followDist = 0.4

error = 0.05

class FollowWall():

    def __init__(self):
        # Start rospy node.
        rospy.init_node("follow_wall")

        # Declare our node as a subscriber to the scan topic and
        #   set self.process_scan as the function to be used for callback.
        rospy.Subscriber("/scan", LaserScan, self.process_scan)

        # Get a publisher to the cmd_vel topic.
        self.twist_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        # Create a default twist msg (all values 0).
        lin = Vector3()
        ang = Vector3()
        self.twist = Twist(linear=lin,angular=ang)

    def process_scan(self, data):
        # Determine closeness to person by looking at scan data from in front of
        #   the robot, set linear velocity based on that information, and
        #   publish to cmd_vel.

        # The ranges field is a list of 360 number where each number
        #   corresponds to the distance to the closest obstacle from the
        #   LiDAR at various angles. Each measurement is 1 degree apart.

        # find closest object's angle and distance
        closestInd = 0
        closestDist = 100
        for i in len(data.ranges):
            if (data.ranges[i] > 0) and (data.ranges[i] < closestDist):
                closestInd = i
                closestDist = data.ranges[i]

        # go forward to find a wall if nothing close
        if closestDist > followDist:
            self.twist.angular.z = 0
            self.twist.linear.x = 0.2
        elif closestDist == followDist:
            if data.ranges[90] == followDist:
                if data.ranges[45] > (followDist / 0.70710678118): # followDist/cos(45)
                    # start turning left
                    self.twist.angular.z = 0.1
                    self.twist.linear.x = 0.2

            if data.ranges[0] < followDist: # turn right if wall right ahead
                self.twist.angular.z = 0.8
                self.twist.linear.x = 0
                self.twist_pub.publish(self.twist)
                rospy.sleep(2)
                return

        # Publish msg to cmd_vel.
        self.twist_pub.publish(self.twist)

    def run(self):
        # Keep the program alive.
        rospy.spin()

if __name__ == '__main__':
    # Declare a node and run it.
    node = FollowWall()
    node.run()
