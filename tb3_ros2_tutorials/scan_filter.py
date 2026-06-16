#!/usr/bin/env python3

## Every Python node in ROS2 should include these lines
import rclpy
from rclpy.node import Node

## Import modules and the LaserScan message type
from rclpy.qos import QoSProfile
from typing import Optional, List
from numpy import linspace, inf
from math import sin
from sensor_msgs.msg import LaserScan

class ScanFilter(Node):
    '''
    A class that implements a LaserScan filter that removes all of the points.
	that are not directly in front of the robot.
    '''
    def __init__(self):
        ''' 
        A constructor that initializes the parent class, subscriber, publisher
        and other parameters.
        '''
        ## Initialize parent class, giving it a name. The idiom is to use the `super()` class and it calls the
		## `Node` class's constructor.
        super().__init__('scan_filter')

        ## Set QoS profile for subscriber becuase we get an mismatch in Quality of Service (QOS)
        ## policies between the publishers and subscribers on the `/scan` topic. 
        subscriber_qos = QoSProfile(
            depth=10,  # Adjust the depth as needed
            reliability= 2 # Reference values:https://docs.ros2.org/foxy/api/rclpy/api/qos.html
        )

        ## Create a publisher, and assign it to a member variable. The call takes a type, 
		## topic name, and queue size.
        self.pub = self.create_publisher(LaserScan, '/filtered_scan', 10)
        
        ## Create a subscriber, and assign it to a member variable. The call takes a type, 
        ## topic name, and the qos_profile argument.
        self.sub = self.create_subscription(LaserScan, '/scan', self.scan_filter_callback,  qos_profile=subscriber_qos)

        ## We're going to assume that the robot is pointing up the x-axis, so that
		## any points with y coordinates further than half of the defined width (1 meter) 
        ## from the axis are not considered
        self.width = 1
        self.extent = self.width / 2.0

        ## Log that we published something.  In ROS2, loggers are associated with nodes, and
		## the idiom is to use the get_logger() call to get the logger.  This has functions
		## for each of the logging levels.
        self.get_logger().info("Publishing the filtered_scan topic. Use RViz to visualize.")
    
    def scan_filter_callback(self,msg: LaserScan) -> None:
        '''
        Callback function to deal with incoming LaserScan messages.

        Args:
            msg (LaserScan): The Turtlebot Scan message.

        Publisher:
		    msg (LaserScan): Filtered scan.
        '''

        ## Figure out the angles of the scan.  We're going to do this each time,
		## in case we're subscribing to more than one laser, with different numbers of beams
        angles = linspace(msg.angle_min, msg.angle_max, len(msg.ranges))

        ## Work out the y coordinates of the ranges
        points = [r * sin(theta) if (theta < 0.525 or theta > 5.753) else inf for r,theta in zip(msg.ranges, angles)]
        
        ## If we're close to the x axis, keep the range, otherwise use inf, which means "no return"
        new_ranges = [r if abs(y) < self.extent else inf for r,y in zip(msg.ranges, points)]
        
        ## Substitute in the new ranges in the original message, and republish it
        msg.ranges = new_ranges
        self.pub.publish(msg)

## The idiom in ROS2 is to use a function to do all of the setup and work.  This
## function is referenced in the setup.py file as the entry point of the node when
## we're running the node with ros2 run.  The function should have one argument, for
## passing command line arguments, and it should default to None.
def main(args: Optional[List[str]] = None) -> None:
    ## Initialize rclpy.  We should do this every time.
    rclpy.init(args=args)

    ## Declare object from the `ScanFilter` class
    scan_filter = ScanFilter()

    ## The spin() call gives control over to ROS2, and it now takes a Node-derived
	## class as a parameter.
    rclpy.spin(scan_filter)

    ## Make sure we shutdown everything cleanly.  This should happen, even if we don't
	## include this line, but you should do it anyway.
    scan_filter.destroy_node()
    rclpy.shutdown()

## If we run the node as a script, then we're going to start here.
if __name__ == '__main__':
	## The idiom in ROS2 is to set up a main() function and to call it from the entry
	## point of the script.
	main()
