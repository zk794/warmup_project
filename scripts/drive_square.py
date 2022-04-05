#!/usr/bin/env python3

import rospy

# msgs needed for /cmd_vel
from geometry_msgs.msg import Twist, Vector3

class DriveSquare(object):

    def __init__(self):
        # initialize the ROS node
        rospy.init_node('drive_square')
        # setup publisher to the cmd_vel ROS topic
        self.robot_movement_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    def run(self):

        # set twist object for going forward
        go_straight = Twist(
            linear=Vector3(0.3, 0, 0),
            angular=Vector3(0, 0, 0)
        )

        # set twist object for turning
        turn = Twist(
           linear=Vector3(0, 0, 0),
           angular=Vector3(0, 0, 0.78)
        )

        # allow the publisher enough time to set up before publishing the first msg
        rospy.sleep(1)

        for i in range(12):
            # straight for 2 seconds
            self.robot_movement_pub.publish(go_straight)
            rospy.sleep(2)
            # turn for 2 seconds
            self.robot_movement_pub.publish(turn)
            rospy.sleep(2)

        stop = Twist(
           linear=Vector3(0, 0, 0),
           angular=Vector3(0, 0, 0)
        )

        self.robot_movement_pub.publish(stop)

if __name__ == '__main__':
    # instantiate the ROS node and run it
    node = DriveSquare()
    node.run()
