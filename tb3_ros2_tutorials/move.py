#!/usr/bin/env python3

## Import needed libraries
import rclpy
import time
import sys
import argparse
import signal
from typing import Optional, List
from rclpy.node import Node
from rclpy.duration import Duration

## Import the built-in Twist message type that is used to send velocity commands to the robot
from geometry_msgs.msg import Twist

class Move(Node):
	'''
	A class that sends Twist messages to move the Turtlebot Burger forward.
	'''
	def __init__(self) -> None:
		'''
		Constructor method for initializing the Move class.
		Parameters:
		- self: The self reference.
		'''
		## Initialize parent class, giving it a name. The idiom is to use the `super()` class and it calls the
		## `Node` class's constructor
		super().__init__('twist_publisher')

		## Create a publisher, and assign it to a member variable. The call takes a type, topic name, and queue size
		self.pub = self.create_publisher(Twist,'/cmd_vel', 1) 

		## Log that we published something.  In ROS2, loggers are associated with nodes, and the idiom is to use the 
		## get_logger() call to get the logger. This has functions  for each of the logging levels
		self.get_logger().info(
			f'The {self.__class__.__name__} class is up and running. Sending Twist commands to the Turtlebot.'
		)

		## Make a Twist message. We're going to set all of the elements
		self.command = Twist()

		## A Twist has three linear velocities (in meters per second), along each of the axes.
		## For the turtlebot, it will only pay attention to the x velocity, since it can't
		## directly move in the y or z direction
		self.command.linear.x = 0.0
		self.command.linear.y = 0.0
		self.command.linear.z = 0.0

		## A Twist also has three rotational velocities (in radians per second).
		## The turtlebot will only respond to rotations around the z (vertical) axis
		self.command.angular.x = 0.0
		self.command.angular.y = 0.0
		self.command.angular.z = 0.0 

	def move_base(self, duration: float = 5) -> None:
		'''
		Function that publishes Twist messages
		Parameters:
		- self: The self reference.

		Publisher:
		- command (Twist): base velocity commands for the Turtlebot.
		'''
		## Positive values makes the robot spin counter-clockwise
		self.command.angular.z = 0.5

		## Get the current time and set the duration for the movement
		start_time = self.get_clock().now()
		duration = Duration(seconds=duration)

		## Move the turtlebot for the specified duration
		while (self.get_clock().now() - start_time) < duration:
			## Publish the Twist commands
			self.pub.publish(self.command)

		## Send a stopping command
		self.stop()
		
	def stop(self) -> None:
		'''
		Function to stop the robot by sending zero velocities.

		Parameters:
		- self: The self reference.
		'''
		## Set the angular velocity to zero
		self.command.angular.z = 0.0

		## Publish the command and notify user that the command has been sent
		self.pub.publish(self.command)
		self.get_logger().info('Stop command sent.')

## The idiom in ROS2 is to use a function to do all of the setup and work.  This
## function is referenced in the setup.py file as the entry point of the node when
## we're running the node with ros2 run.  The function should have one argument, for
## passing command line arguments, and it should default to None. 
def main(args: Optional[List[str]] = None) -> None:
	'''
	A function that initializes all the methods.
	Parameters:
	- args: Command line arguments (default is None).
	'''
	# ## Set up argument parsing
	# ## Create an ArgumentParser object, which helps parse command-line arguments.
	# ## The 'description' is just a message that shows up when you run the script with --help.
	# parser = argparse.ArgumentParser(description='Move Turtlebot')

	# ## Adds a command-line argument called --duration.
	# ## It's expected to be an integer (type=int).
	# ## If the user doesn’t provide this argument, it defaults to 5.
	# ## The help string describes what the argument does.
	# parser.add_argument('--duration', type=int, default=5, help='Duration to move the Turtlebot')

	# ## Parses the command-line arguments and stores them in the 'parsed_args' object.
	# ## You can then access the duration with: parsed_args.duration
	# parsed_args = parser.parse_args()

	# ## Convert the parsed arguments to a list format suitable for rclpy.init()
	# rclpy_args = sys.argv[1:]

	## Initialize rclpy
	rclpy.init(args=args) #rclpy_args)

	## Create an intsance of the `Move` class 
	base_motion = Move()

	## Define a signal handler to stop the robot on exit
	def signal_handler(sig, frame) -> None:
		base_motion.stop()
		base_motion.destroy_node()
		rclpy.shutdown()
		sys.exit(0)

	## Register the signal handler
	signal.signal(signal.SIGINT, signal_handler)

	## Move the turtlebot for the specified duration
	base_motion.move_base(duration=5.0) #parsed_args.duration)

	## Destroy the node and shutdown rclpy
	base_motion.destroy_node()
	rclpy.shutdown()

## If we run the node as a script, then we're going to start here.
if __name__ == '__main__':
	## The idiom in ROS2 is to set up a main() function and to call it from the entry
	## point of the script.
	main()

