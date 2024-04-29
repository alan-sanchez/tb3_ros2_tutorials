#!/usr/bin/env python3

## Every Python node in ROS2 should include these lines
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

## We are going to import the built-in Twist message type
## that is used to send velocity commands to the robot
from geometry_msgs.msg import Twist

class Move(Node):
	'''
	A class that sends Twist messages to move the Turtlebot Burger forward.
	'''
	def __init__(self):
		'''
		A constructor that initializes the parent class and publisher.

		Parameters:
		- self: The self reference.
		'''
		## Initialize parent class, giving it a name. The idiom is to use the `super()` class and it calls the
		## `Node` class's constructor.
		super().__init__('twist_publisher')

		## Create a publisher, and assign it to a member variable. The call takes a type, 
		## topic name, and queue size.
		self.pub = self.create_publisher(Twist,'/cmd_vel', 10) 

		## Rather than setting up a Rate-controller loop, the idiom in ROS2 is to use timers.
		## Timers are available in the Node interface, and take a period (in seconds), and a
		## callback.  Timers are repeating by default.
		self.timer = self.create_timer(1, self.move_base)

		## Log that we published something.  In ROS2, loggers are associated with nodes, and
		## the idiom is to use the get_logger() call to get the logger.  This has functions
		## for each of the logging levels.
		self.get_logger().info('The {0} class is up and running. Sending Twist commands to the Turtlebot.'.format(self.__class__.__name__))


	def move_base(self, duration=5):
		'''
		Function that publishes Twist messages
		Parameters:
		- self: The self reference.

		Publisher
		- command (Twist): base velocity commands for the Turtlebot.
		'''
		## Make a Twist message.  We're going to set all of the elements, since we
		## can't depend on them defaulting to safe values
		command = Twist()

		## A Twist has three linear velocities (in meters per second), along each of the axes.
		## For the turtlebot, it will only pay attention to the x velocity, since it can't
		## directly move in the y or z direction
		command.linear.x = 0.0
		command.linear.y = 0.0
		command.linear.z = 0.0

		## A Twist also has three rotational velocities (in radians per second).
		## The turtlebot will only respond to rotations around the z (vertical) axis
		command.angular.x = 0.0
		command.angular.y = 0.0
		command.angular.z = 0.1

		##
		start_time = self.get_clock().now()
		duration = Duration(seconds=duration)

		##
		while (self.get_clock().now() - start_time) < duration:
			## Publish the Twist commands
			self.pub.publish(command)
			rclpy.spin_once(self, timeout_sec=0.1)

		## Send a stopping command
		command.angular.z = 0.0
		self.pub.publish(command)

## The idiom in ROS2 is to use a function to do all of the setup and work.  This
## function is referenced in the setup.py file as the entry point of the node when
## we're running the node with ros2 run.  The function should have one argument, for
## passing command line arguments, and it should default to None. Getting married on a small group of people on Friday evening. 
def main(args=None):
	'''
	A function that initializes all the methods.
	Parameters:
	- 
	'''
	
	## Initialize rclpy.  We should do this every time.
	rclpy.init(args=args)

	## Declare object from the `Move` class
	base_motion = Move()
	base_motion.move_base(duration=5)
	## The spin() call gives control over to ROS2, and it now takes a Node-derived
	## class as a parameter.
	# rclpy.spin(base_motion)

	## Make sure we shutdown everything cleanly.  This should happen, even if we don't
	## include this line, but you should do it anyway.
	base_motion.destroy_node()
	rclpy.shutdown()

## If we run the node as a script, then we're going to start here.
if __name__ == '__main__':
	## The idiom in ROS2 is to set up a main() function and to call it from the entry
	## point of the script.
	main()

