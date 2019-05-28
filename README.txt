Author: Rene Jacques
Class: ENPM661 Planning for Autonomous Robotics
Project 3
April 25, 2019

Note: 
	This code was written in python and run using ROS Kinetic in Ubuntu 16.04 

Necessary Libraries:
	cv2, random, copy, numpy, math, rospy, std_msgs, geometry_msgs

Main File:
	project_3_alpha.py

Secondary Files:
	node.py, A_star_differential_charlie.py, map_builder_2.py, priority_queue.py, path_publisher.py

Run Instructions:

Place the submitted package (project3) in your catkin_ws/src folder. Then open a terminal window. Inside that window input the following commands. Note that the center of the world is the origin (0.0,0.0), the coordinate system behaves normally (+y is up and +x is to the right), and all units are in meters.

source /opt/ros/kinetic/setup.bash
cd (catkin_ws root path)
source devel/setup.bash
catkin_make
roslaunch project3 turtle_bot_world.launch start_x:="0.0" start_y:="0.0" goal_x:="0.0" goal_y:="0.0" 
	Replace the values inside "" with the desired coordinates in meters from the center of the map

Once the program is running you will need to hit spacebar three times:
	1) The first window to be displayed will be the A* map with the start and goal nodes colored in. You need to hit spacebar to start searching.
	2) Once the algorithm has found the path it will be displayed. At this point you need to hit spacebar again to send the path to the virtual turtlebot in gazebo.
	3) Once the gazebo simulation is finished you can hit spacebar again to close the A* map.

Main File Description:
	project_3_final.py
		-This is the main file for this project
		-This file:
			-receives parameters defined by user input when launching the world
			-converts input into usable variables
			-runs A* to find the path between the start node and the goal node
			-sends path in form of wheel velocities in radians/sec to gazebo publisher path_publisher

Secondary File Descriptions:
	map_builder_2.py
		-This file builds the map with obstacles using semi-algebraic models and half-planes. It also enlarges the obstacles using the Minkowski sum for the robot radius of 0.177m and with a clearance of 0.05m.  This file can be run on its own and it will display 1 image: the map with the Minkowski sum applied based on the input radius and clearance. 

	node.py
		-This file contains several classes. The Node class is the same class that was used for projects 1 and 2. The node class that is used for this project is called D_Node. D_Node is an extension of H_Node. D_Node is able to store (x,y) coordinate pairs and g, h, and f costs for heuristic storage as well as rpm pairs and robot facing angle.

	A_star_differential_charlie.py
		-This file contains the A_star class. This class calculates and visualizes both the nodes that have been visited and the path required to reach the goal using the A* algorithm. This class takes as input the grid squares, grid lines, image, resolution, and scaling factors. When performing a search the search method takes as input the starting coordinates in the grid space and the goal coordinates in the grid space. 

	path_publisher.py
		-This file contains the PathPublisher class. This class takes as input the radius of the robot and the distance between the robot wheels. The internal run command takes as input the path array (in wheel velocities in radians/sec) that is then converted into linear velocities and angular velocities and published to gazebo using the mobile_base/commands/velocity publisher.
