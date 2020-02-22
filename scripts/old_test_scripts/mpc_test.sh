#!/bin/bash

gnome-terminal -e "/opt/ros/kinetic/bin/roscore" --geometry=40x12+0+0

sleep 1s

gnome-terminal -e "/opt/ros/kinetic/bin/rosrun dynamic_obstacle_avoidance_planner mpc_path_tracker" --geometry=40x12+485+0

gnome-terminal -e "/opt/ros/kinetic/bin/roslaunch dynamic_obstacle_avoidance_planner sin_curve.launch" --geometry=40x12+900+0

gnome-terminal -e "/opt/ros/kinetic/bin/rosrun dynamic_obstacle_avoidance_planner sim_3dof.py" --geometry=40x12+1235+0
