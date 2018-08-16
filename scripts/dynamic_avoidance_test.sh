#!/bin/bash

gnome-terminal -e "/opt/ros/kinetic/bin/roscore" --geometry=45x12+0+0 &
sleep 1s

# robot sim
gnome-terminal -e "/opt/ros/kinetic/bin/rosrun four_wheel_drive_independent_steering sim_3dof.py" --geometry=45x12+475+0 &
sleep 1s

# obstacle sim
gnome-terminal -e "/opt/ros/kinetic/bin/roslaunch four_wheel_drive_independent_steering diff_drive_sim.launch" --geometry=45x12+895+0 &
sleep 1s

gnome-terminal -e "/opt/ros/kinetic/bin/roslaunch four_wheel_drive_independent_steering obstacle_predictor.launch" --geometry=45x12+1315+0 &
sleep 1s

gnome-terminal -e "/opt/ros/kinetic/bin/roslaunch four_wheel_drive_independent_steering robot_predictor.launch" --geometry=45x12+0+250 &
sleep 1s

gnome-terminal -e "/opt/ros/kinetic/bin/rosrun tf static_transform_publisher 0 0 0 0 0 0 map odom 100" --geometry=45x12+475+250 &
sleep 1s

gnome-terminal -e "/opt/ros/kinetic/bin/roslaunch four_wheel_drive_independent_steering dynamic_local_costmap.launch" --geometry=45x12+895+250 &
sleep 1s

gnome-terminal -e "/opt/ros/kinetic/bin/rosrun rviz rviz -d ../config/dynamic_avoidance.rviz" --geometry=45x12+1315+250 &
sleep 1s
