# warmup_project

## drive in square

- description: I wanted to make a square by looping the process of going straight then turning 90 degrees. I made it square by using consistent timing.
- structure: In the class DriveSquare, I have an init method to initialize the node and publisher, and a run method. In the run method, I specify three Twist objects to control the robot's movement: one for going straight, one for turning right, and one for stopping. To make the robot drive in a square three times, I use a for loop to alternate between publishing the going straight and turning Twist objects, then I publish the stop Twist object. The main command of the program just initialilzes a DriveSquare object and calls the run method on it.
- video: see square.mov

## person follower

- description: First the robot finds the angle and distance of the closest object to it. Then it turns towards the object if the angle is not 0, and moves towards it if too far away or away if it's too close. The robot accounts for updates in the person's location by never having a final destination in mind, rather it constantly checks where the person is and tries to turn/move towards it.
- structure: There is one class, FollowPerson. When this class is initialized, it subscribes to the robot's lidar scan readings and sets a publisher to cmd_vel. In the subscriber, we call process_scan(), which analyzes the scan and tells the robot how it should move. In process_scan(), there is a for loop to record the closest distance of an object and the angle where that object/person is. Then there is a series of if statements telling the robot how much to turn and go forward based on the angle and distance of the object. In run(), which is called when the program begins, there is just a call to rospy.spin(), which allows the program to continuously run the process_scan() method.
- video: see person-follow.mov
- note: this code was working when I took the video a couple days ago, but when I tried it again last night it didn't work, either because I forgot to save some change or some unknown reason, so I wanted to note this so you don't think I took video of someone else's robot.

## wall follower

- description: The idea was to have the robot identify the closest object/wall, if it is not on the robot's left then maneuver such that it is, and if the wall is on the left then go straight. If there is a wall in front of it then it will turn right. If there was something on its left but now there isn't, turn left. If it doesn't see anything, it goes forward until it sees something.
- structure: Therer is one class, FollowWall. The init and process_scan methods are set up like in the person follower. The contents of process_scan are similar to the person follower, except there are more if statements to account for different states of walls surrounding the robot.
- video: see wall-follow.mov

## challenges

The thing I found most challenging about this project was debugging. Since the robot is taking scans so frequently, printing what it's seeing at all times would be impossible to sort through. Additionally, it is difficult to watch the robot and the printout at the same time, so it is difficult to associate the readings with the actual state that the robot is in, which is necessary to check if the robot is actually reading what I expect it to.

## future work

For driving in a square, I think it could be improved if I could find some external way to measure how far the robot is turning, then I could make it turn exactly 90 degrees.
I think that both my person and wall followers could be improved by better organization. Instead of a ton of if-else  statements, maybe I could break the problem down into more helper functions to make the code easier to handle.

## takeaways

Even more so than with pure software, coding a robot requires a lot of organization because programming for hardware has more things that can go wrong. It is important to have a very good idea of the robot's strategy to accomplish the task, and to understand how to break that strategy down into manageable chunks so debugging isn't a nightmare and you can keep track of which part of your code does what.
