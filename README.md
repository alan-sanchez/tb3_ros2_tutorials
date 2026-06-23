# ROS2 Examples

This repo provides guides to get started on your ROS2 software development. Below are examples of nodes to have a Turtlebot3 perform simple tasks. Note: I used the Turtlebot Burger for this set of tutorials.

Before you get started, I recommend you complete the [Quick Start Guide](https://docs.robotis.com/docs/systems/turtlebot3/quick_start_guide/pc_setup) offered by [ROBOTIS](https://emanual.robotis.com/). I found it to be very helpful when setting everything up. For this project, I followed the instructions that installed humble on the turtlebot. 

1. [Teleoperate the Turtlebot with a Node](examples/teleoperate_tb3_with_node.md) - Use a python script that sends velocity commands.  
2. [Filter Laser Scans](examples/laser_scan_filter.md) - Publish new scan ranges that are directly in front of the Turtlebot Burger.
<!-- 3. [Mobile Base Collision Avoidance](example_3.md) - Stop the Turtlebot Burger from running into a wall.
4. [Create a Service to Avoidance Range) - 