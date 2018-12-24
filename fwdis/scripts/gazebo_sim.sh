#!/bin/bash

gnome-terminal -e "/opt/ros/kinetic/bin/roscore" --geometry=40x12+0+0

sleep 1s

gnome-terminal -e "/opt/ros/kinetic/bin/roslaunch four_wheel_drive_independent_steering gazebo.launch" --geometry=40x12+450+0

sleep 1s

gnome-terminal -e "/opt/ros/kinetic/bin/roslaunch four_wheel_drive_independent_steering rviz.launch" --geometry=40x12+450+300

sleep 1s

gnome-terminal -e "/opt/ros/kinetic/bin/rosrun four_wheel_drive_independent_steering fwdis_controller" --geometry=40x12+850+0

sleep 1s

gnome-terminal -e "/opt/ros/kinetic/bin/rosrun four_wheel_drive_independent_steering fwdis_gazebo" --geometry=40x12+1250+300

sleep 1s

gnome-terminal -e "/opt/ros/kinetic/bin/roslaunch four_wheel_drive_independent_steering control.launch" --geometry=40x12+0+300

sleep 1s


#gnome-terminal -e "/opt/ros/kinetic/bin/roslaunch --screen four_wheel_drive_independent_steering holonomic_dwa.launch" --geometry=40x12+850+300

sleep 1s
