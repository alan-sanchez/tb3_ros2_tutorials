## Teleoperate The Turtlebot with a Node
<!-- <p align="center">
  <img src="images/move_stretch.gif"/>
</p> -->

The objective of this guide is to send [Twist](https://docs.ros2.org/galactic/api/geometry_msgs/msg/Twist.html) message type to the turtlebot and move it's base in a rotaitonal motion. 


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
import sys
import signal

from typing import Optional, List
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import Twist

class Move(Node):
	"""
	A class that sends Twist messages to move the Turtlebot Burger forward.
	"""
	def __init__(self) -> None:
		"""
		Constructor method for initializing the Move class.
		"""
		super().__init__('twist_publisher')

		self.pub = self.create_publisher(Twist,'/cmd_vel', 1) 

		self.get_logger().info(
			f'The {self.__class__.__name__} class is up and running. Sending Twist commands to the Turtlebot.'
		)

		self.command = Twist()
		self.command.linear.x = 0.0
		self.command.linear.y = 0.0
		self.command.linear.z = 0.0
		self.command.angular.x = 0.0
		self.command.angular.y = 0.0
		self.command.angular.z = 0.0 

	def move_base(self, duration: float = 5) -> None:
		"""
		Function that publishes Twist messages
	
		Args:
			duration (float): The duration (in seconds) for which the robot should move.
		"""
		self.command.angular.z = 0.5

		start_time = self.get_clock().now()
		duration = Duration(seconds=duration)

		while (self.get_clock().now() - start_time) < duration:
			self.pub.publish(self.command)

	def stop(self) -> None:
		"""
		Function to stop the robot by sending a zero angular velocity.
		"""
		self.command.angular.z = 0.0

		self.pub.publish(self.command)
		self.get_logger().info('Stop command sent.')


def main(args: Optional[List[str]] = None) -> None:
	"""
	A function that initializes all the methods.
	Args:
		args: Command line arguments (default is None).
	"""
	rclpy.init(args=args) 
	base_motion = Move()

	def signal_handler(sig, frame) -> None:
		base_motion.stop()
		base_motion.destroy_node()
		rclpy.shutdown()
		sys.exit(0)

	signal.signal(signal.SIGINT, signal_handler)

	base_motion.move_base(duration=5.0) 
	base_motion.stop()

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
You need to import rospy if you are writing a ROS Node. The `geometry_msgs.msg` import is so that we can send velocity commands to the robot.


**Next Example:** [Example 2](example_2.md)
