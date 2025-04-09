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


```python
import rospy
from geometry_msgs.msg import Twist
```
You need to import rospy if you are writing a ROS Node. The geometry_msgs.msg import is so that we can send velocity commands to the robot.




After saving the edited node, bringup [Stretch in the empty world simulation](gazebo_basics.md). To drive the robot with the node, type the follwing in a new terminal

```bash
cd catkin_ws/src/stretch_ros_tutorials/src/
python move.py
```
To stop the node from sending twist messages, type **Ctrl** + **c**.


**Next Example:** [Example 2](example_2.md)
