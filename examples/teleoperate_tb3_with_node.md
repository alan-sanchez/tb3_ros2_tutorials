# Teleoperate the TurtleBot3 with a Node

<!-- ![TurtleBot3 moving in a rotational motion](https://raw.githubusercontent.com/alan-sanchez/tb3_ros2_tutorials/main/images/move_tb3.gif) -->

The goal of this example is to give you an enhanced understanding of how to control the mobile base by sending `Twist` messages to a TurtleBot3 robot.

Begin by opening a terminal on your PC with **Ctrl** + **Alt** + **T** and connecting to the Raspberry Pi with its IP address.

```
# Terminal 1
ssh ubuntu@{IP_ADDRESS_OF_RASPBERRY_PI}
```

Within that same terminal, launch the TurtleBot3 robot bringup. Make sure to use the correct `TURTLEBOT3_MODEL` parameter for your system — either `burger` or `waffle_pi`.

```
# Terminal 1
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_bringup robot.launch.py
```

Open a new terminal on your local machine and run the following command to execute the move node.

```
# Terminal 2
ros2 run tb3_ros2_tutorials move
```

To stop the node from sending Twist messages, type **Ctrl** + **c**. However, if that doesn't work, an alternative is to copy and paste the following into the terminal:

```
# Terminal 2
ros2 topic pub --once /cmd_vel geometry_msgs/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

## The Code

Below is the code which will send *Twist* messages to drive the robot in a rotational motion.

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
	A function that initializes all the methods
	
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

## The Code Explained

Now let's break the code down.

```python
#!/usr/bin/env python3
```

Every Python ROS [Node](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Nodes/Understanding-ROS2-Nodes.html) will have this declaration at the top. The first line makes sure your script is executed as a Python3 script.

```python
import rclpy
import sys
import signal
from typing import Optional, List
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import Twist
```

You need to import `rclpy` if you are writing a ROS 2 Node. The `geometry_msgs.msg` import provides the built-in `Twist` message type used to send velocity commands to the robot. `signal` and `sys` are used to handle graceful shutdowns on **Ctrl** + **c**.

```python
class Move(Node):
    def __init__(self) -> None:
        super().__init__('twist_publisher')
```

The idiom in ROS 2 is to define nodes as classes that inherit from `Node`. Using `super()` calls the `Node` class's constructor and gives the node its name.

```python
        self.pub = self.create_publisher(Twist, '/cmd_vel', 1)
```

This creates a publisher and assigns it to a member variable. The call takes a message type, a topic name, and a queue size.

```python
        self.get_logger().info(
            f'The {self.__class__.__name__} class is up and running...'
        )
```

In ROS 2, loggers are associated with nodes. The idiom is to use the `get_logger()` call to get the node's logger, which has methods for each logging level (`info`, `warn`, `error`, etc.).

```python
        self.command = Twist()
        self.command.linear.x = 0.0
        self.command.linear.y = 0.0
        self.command.linear.z = 0.0
```

Make a `Twist` message. We're going to set all of the elements, since we can't depend on them defaulting to safe values. A `Twist` has three linear velocities (in meters per second) along each of the axes. For the TurtleBot3, it will only pay attention to the x velocity, since it can't directly move in the y or z direction.

```python
        self.command.angular.x = 0.0
        self.command.angular.y = 0.0
        self.command.angular.z = 0.0
```

A `Twist` also has three rotational velocities (in radians per second). The TurtleBot3 will only respond to rotations around the z (vertical) axis.

```python
    def move_base(self, duration: float = 5) -> None:
        self.command.angular.z = 0.5
```

Positive values make the robot spin counter-clockwise.

```python
        start_time = self.get_clock().now()
        duration = Duration(seconds=duration)

        while (self.get_clock().now() - start_time) < duration:
            self.pub.publish(self.command)
```

Get the current time and set the duration for the movement, then continuously publish the `Twist` command to move the TurtleBot3 for the specified duration.

```python
    def stop(self) -> None:
        self.command.angular.z = 0.0
        self.pub.publish(self.command)
        self.get_logger().info('Stop command sent.')
```

Set the angular velocity to zero, then publish the command and notify the user that the stop command has been sent.

```python
def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    base_motion = Move()
```

The idiom in ROS 2 is to use a `main()` function to do all of the setup and work. This function is referenced in the `setup.py` file as the entry point of the node when running with `ros2 run`. It takes one argument for passing command line arguments, defaulting to `None`. `rclpy.init()` initializes the ROS 2 Python client library, and a `Move` node instance is then created.

```python
    def signal_handler(sig, frame) -> None:
        base_motion.stop()
        base_motion.destroy_node()
        rclpy.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)
```

Define and register a signal handler to stop the robot cleanly on **Ctrl** + **c** (SIGINT). It sends a stop command, destroys the node, and shuts down `rclpy` before exiting.

```python
    base_motion.move_base(duration=5.0)
    base_motion.stop()

    base_motion.destroy_node()
    rclpy.shutdown()
```

Move the TurtleBot3 for the specified duration, then send a stop command. Finally, destroy the node and shut down `rclpy` to cleanly release all resources.

```python
if __name__ == '__main__':
    main()
```

The idiom in ROS 2 is to set up a `main()` function and to call it from the entry point of the script.

**Next Example:** [Example 2](https://github.com/alan-sanchez/tb3_ros2_tutorials/blob/main/examples/example_2.md)