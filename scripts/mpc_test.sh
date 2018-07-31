#!/bin/bash

gnome-terminal -e "/opt/ros/kinetic/bin/roscore" --geometry=40x12+0+0

sleep 1s

gnome-terminal -e "/opt/ros/kinetic/bin/rosrun four_wheel_drive_independent_steering mpc_path_tracker" --geometry=40x12+485+0

gnome-terminal -e "/opt/ros/kinetic/bin/roslaunch four_wheel_drive_independent_steering sin_curve.launch" --geometry=40x12+900+0

gnome-terminal -e "/opt/ros/kinetic/bin/rosrun four_wheel_drive_independent_steering sim_3dof.py" --geometry=40x12+1235+0
