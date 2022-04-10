import rospy
​
# msg needed for /scan.
from sensor_msgs.msg import LaserScan
​
# msgs needed for /cmd_vel.
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3

# minimum distance from person
followDist = 0.3

class FollowPerson():

    def __init__(self):
        # Start rospy node.
        rospy.init_node("follow_person")

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
        closestDist = 10
        for i in len(data.ranges):
            if (data.ranges[i] > 0) and (data.ranges[i] < closestDist):
                closestInd = i
                closestDist = data.ranges[i]

        # set velocity (turn towards closest object and go towards/away from it)
        if (closestInd > 0 ) and (closestInd < 180):
            self.twist.angular.z = closestInd * 0.1
            if closestDist > followDist:
                self.twist.linear.x = closestDist * 0.1
            elif closestDist < followDist:
                self.twist.linear.x = closestDist * -0.1
            else:
                self.twist.linear.x = 0
        elif (closestInd > 0 ):
            self.twist.angular.z = (360 - closestInd) * -0.1
            if closestDist > followDist:
                self.twist.linear.x = closestDist * 0.1
            elif closestDist < followDist:
                self.twist.linear.x = closestDist * -0.1
            else:
                self.twist.linear.x = 0
        else:
            self.twist.angular.z = 0
            if closestDist > followDist:
                self.twist.linear.x = closestDist * 0.1
            elif closestDist < followDist:
                self.twist.linear.x = closestDist * -0.1
            else:
                self.twist.linear.x = 0

        # Publish msg to cmd_vel.
        self.twist_pub.publish(self.twist)

    def run(self):
        # Keep the program alive.
        rospy.spin()

if __name__ == '__main__':
    # Declare a node and run it.
    node = FollowPerson()
    node.run()
