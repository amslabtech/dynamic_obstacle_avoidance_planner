#!/bin/bash

gnome-terminal -e "/opt/ros/kinetic/bin/roscore" --geometry=40x12+0+0

sleep 1s

gnome-terminal -e "/opt/ros/kinetic/bin/roslaunch dynamic_obstacle_avoidance_planner gazebo.launch" --geometry=40x12+450+0

sleep 1s

gnome-terminal -e "/opt/ros/kinetic/bin/roslaunch dynamic_obstacle_avoidance_planner rviz.launch" --geometry=40x12+450+300

sleep 1s

gnome-terminal -e "/opt/ros/kinetic/bin/rosrun dynamic_obstacle_avoidance_planner fwdis_controller" --geometry=40x12+850+0

sleep 1s

gnome-terminal -e "/opt/ros/kinetic/bin/rosrun dynamic_obstacle_avoidance_planner fwdis_gazebo" --geometry=40x12+1250+300

sleep 1s

gnome-terminal -e "/opt/ros/kinetic/bin/roslaunch dynamic_obstacle_avoidance_planner control.launch" --geometry=40x12+0+300

sleep 1s


#gnome-terminal -e "/opt/ros/kinetic/bin/roslaunch --screen dynamic_obstacle_avoidance_planner holonomic_dwa.launch" --geometry=40x12+850+300

sleep 1s
