#!/usr/bin/env python3

import rospy

# msg needed for /scan.
from sensor_msgs.msg import LaserScan

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

        # find closest object's angle and distance
        closestInd = 0
        closestDist = 100
        for i in range(len(data.ranges)):
            if (data.ranges[i] > 0) and (data.ranges[i] < closestDist):
                closestInd = i
                closestDist = data.ranges[i]

        # go forward to find a wall if nothing close
        if closestDist > followDist:
            self.twist.angular.z = 0
            self.twist.linear.x = 0.2
        elif closestDist > followDist-error:
            if closestInd == 90:
                self.twist.angular.z = 0
                self.twist.linear.x = 0
            elif closestInd < 90:
                self.twist.angular.z = ((90-closestInd) * 0.1) / 90
                self.twist.linear.x = -0.01
            elif closestInd < 270:
                self.twist.angular.z = ((closestInd-90) * -0.1) / 90
                self.twist.linear.x = -0.01
            else:
                self.twist.angular.z = ((360-closestInd) * 0.1) / 90 + 0.05
                self.twist.linear.x = -0.01

        # Publish msg to cmd_vel.
        self.twist_pub.publish(self.twist)

    def run(self):
        # Keep the program alive.
        rospy.spin()

if __name__ == '__main__':
    # Declare a node and run it.
    node = FollowWall()
    node.run()
