## Teleoperate The Turtlebot with a Node
<!-- <p align="center">
  <img src="images/move_stretch.gif"/>
</p> -->

The objective of this guide is to send [Twist](https://docs.ros2.org/galactic/api/geometry_msgs/msg/Twist.html)


Open a terminal from your PC with `Ctrl` + `Alt` + `T` and connect to the Raspberry Pi with its IP address.
```bash
# Terminal 1 
ssh ubuntu@{IP_ADDRESS_OF_RASPBERRY_PI}
```

Within that same terminal, launch the turtlebot3 robot bringup. Make sure to use correct `Turtlebot3_model` parameter for your system. It'll be either `burger` or `waffle_pi`. 
```bash
# Terminal 1
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_bringup robot.launch.py
```

Open a new terminal on your local machine and run the following command to execute the move node.
```bash
# Terminal 2
ros2 run tb3_ros2_tutorials move
```

To stop the node from sending twist messages, type **`Ctrl`** + **`c`**. However, if that doesn't work, an alternative is to copy and paste the following in the terminal:


```bash
# Terminal 2
ros2 topic pub --once /cmd_vel geometry_msgs/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

```
The goal of this example is to control the mobile base by sending `Twist` messages.

### The Code
Below is the code which will send *Twist* messages to drive the robot forward.

```python
#!/usr/bin/env python3

import rclpy
import time
# import sys
# import argparse
import signal
from rclpy.node import Node
from rclpy.duration import Duration

from geometry_msgs.msg import Twist

class Move(Node):
	'''
	A class that sends Twist messages to move the Turtlebot Burger forward.
	'''
	def __init__(self):
		'''
		Constructor method for initializing the Move class.
		Parameters:
		- self: The self reference.
		'''
		super().__init__('twist_publisher')

		self.pub = self.create_publisher(Twist,'/cmd_vel', 1) 

		self.get_logger().info('The {0} class is up and running. Sending Twist commands to the Turtlebot.'.format(self.__class__.__name__))

		self.command = Twist()
		self.command.linear.x = 0.0
		self.command.linear.y = 0.0
		self.command.linear.z = 0.0
		self.command.angular.x = 0.0
		self.command.angular.y = 0.0
		self.command.angular.z = 0.0 

	def move_base(self, duration=5):
		'''
		Function that publishes Twist messages
		Parameters:
		- self: The self reference.

		Publisher:
		- command (Twist): base velocity commands for the Turtlebot.
		'''
		self.command.angular.z = 0.5

		start_time = self.get_clock().now()
		duration = Duration(seconds=duration)

		while (self.get_clock().now() - start_time) < duration:
			self.pub.publish(self.command)

		self.stop()
		

	def stop(self):
		'''
		Function to stop the robot by sending zero velocities.

		Parameters:
		- self: The self reference.
		'''
		self.command.angular.z = 0.0
		self.pub.publish(self.command)
		self.get_logger().info('Stop command sent.')


def main(args=None):
	'''
	A function that initializes all the methods.
	Parameters:
	- args: Command line arguments (default is None).
	'''
	def signal_handler(sig, frame):
		base_motion.stop()
		base_motion.destroy_node()
		rclpy.shutdown()
		sys.exit(0)

	## Register the signal handler
	signal.signal(signal.SIGINT, signal_handler)

	rclpy.init(args=args)

	base_motion = Move()
	base_motion.move_base(duration=5)
	base_motion.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()

```

### The Code Explained

Now let's break the code down.

```python
#!/usr/bin/env python
```
Every Python ROS [Node](http://wiki.ros.org/Nodes) will have this declaration at the top. The first line makes sure your script is executed as a Python script.


<!-- ```python
import rospy
from geometry_msgs.msg import Twist
```
You need to import rospy if you are writing a ROS Node. The geometry_msgs.msg import is so that we can send velocity commands to the robot.


```python
class Move:
	def __init__(self):
		self.pub = rospy.Publisher('/stretch/cmd_vel', Twist, queue_size=1)#/stretch_diff_drive_controller/cmd_vel for gazebo
```
This section of code defines the talker's interface to the rest of ROS. pub = rospy.Publisher("/stretch/cmd_vel", Twist, queue_size=1) declares that your node is publishing to the /stretch/cmd_vel topic using the message type Twist. The queue_size argument limits the amount of queued messages if any subscriber is not receiving them fast enough.


```Python
command = Twist()
```
Make a Twist message.  We're going to set all of the elements, since we
can't depend on them defaulting to safe values.

```python
command.linear.x = 0.1
command.linear.y = 0.0
command.linear.z = 0.0
```
A Twist has three linear velocities (in meters per second), along each of the axes. For Stretch, it will only pay attention to the x velocity, since it can't directly move in the y direction or the z direction.


```python
command.angular.x = 0.0
command.angular.y = 0.0
command.angular.z = 0.0
```
A *Twist* also has three rotational velocities (in radians per second).
The Stretch will only respond to rotations around the z (vertical) axis.


```python
self.pub.publish(command)
```
Publish the Twist commands in the previously defined topic name */stretch/cmd_vel*.

```Python
rospy.init_node('move')
base_motion = Move()
rate = rospy.Rate(10)
```
The next line, rospy.init_node(NAME, ...), is very important as it tells rospy the name of your node -- until rospy has this information, it cannot start communicating with the ROS Master. In this case, your node will take on the name talker. NOTE: the name must be a base name, i.e. it cannot contain any slashes "/".

The `rospy.Rate()` function creates a Rate object rate. With the help of its method sleep(), it offers a convenient way for looping at the desired rate. With its argument of 10, we should expect to go through the loop 10 times per second (as long as our processing time does not exceed 1/10th of a second!)

```python
while not rospy.is_shutdown():
	base_motion.move_forward()
	rate.sleep()
```
This loop is a fairly standard rospy construct: checking the rospy.is_shutdown() flag and then doing work. You have to check is_shutdown() to check if your program should exit (e.g. if there is a Ctrl-C or otherwise). The loop calls rate.sleep(), which sleeps just long enough to maintain the desired rate through the loop.


## Move Stretch in Simulation
<p align="center">
  <img src="images/move.gif"/>
</p>

Using your preferred text editor, modify the topic name of the published *Twist* messages. Please review the edit in the **move.py** script below.

```python
self.pub = rospy.Publisher('/stretch_diff_drive_controller/cmd_vel', Twist, queue_size=1)
```

After saving the edited node, bringup [Stretch in the empty world simulation](gazebo_basics.md). To drive the robot with the node, type the follwing in a new terminal

```bash
cd catkin_ws/src/stretch_ros_tutorials/src/
python move.py
```
To stop the node from sending twist messages, type **Ctrl** + **c**.


**Next Example:** [Example 2](example_2.md) -->
