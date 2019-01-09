#!/bin/bash

gnome-terminal -e "/opt/ros/kinetic/bin/roscore" --geometry=45x12+0+0 &
sleep 0.5s

gnome-terminal -e "/opt/ros/kinetic/bin/roslaunch four_wheel_drive_independent_steering load_param_diff_drive.launch" --geometry=45x12+0+0 &
sleep 0.1s

gnome-terminal -e "/opt/ros/kinetic/bin/roslaunch four_wheel_drive_independent_steering load_param_fwdis.launch" --geometry=45x12+0+0 &
sleep 0.1s

gnome-terminal -e "/opt/ros/kinetic/bin/rosrun rviz rviz -d ../config/dynamic_avoidance.rviz" --geometry=45x12+475+0 &
sleep 0.5s

# robot sim
gnome-terminal -e "/opt/ros/kinetic/bin/rosrun four_wheel_drive_independent_steering sim_3dof.py" --geometry=45x12+895+0 &
sleep 0.1s

# obstacle sim
gnome-terminal -e "/opt/ros/kinetic/bin/roslaunch four_wheel_drive_independent_steering diff_drive_sim.launch" --geometry=45x12+1315+0 &
sleep 0.1s

gnome-terminal -e "/opt/ros/kinetic/bin/rosrun four_wheel_drive_independent_steering obstacle_predictor_kf" --geometry=45x12+0+250 &
sleep 0.1s

gnome-terminal -e "/opt/ros/kinetic/bin/roslaunch four_wheel_drive_independent_steering robot_predictor.launch" --geometry=45x12+475+250 &
sleep 0.1s

# if no localization
WORLD_FRAME=$(/opt/ros/${ROS_DISTRO}/bin/rosparam get /dynamic_avoidance/WORLD_FRAME)
gnome-terminal -e "/opt/ros/kinetic/bin/rosrun tf static_transform_publisher -15 0 0 0 0 0 $WORLD_FRAME odom 100" --geometry=45x12+895+250 &
sleep 0.1s

gnome-terminal -e "/opt/ros/kinetic/bin/roslaunch four_wheel_drive_independent_steering dynamic_local_costmap.launch" --geometry=45x12+1315+250 &
sleep 0.1s

gnome-terminal -e "/opt/ros/kinetic/bin/roslaunch four_wheel_drive_independent_steering avoidance_path_planner.launch" --geometry=45x12+0+476 &
sleep 0.1s

# for test
gnome-terminal -e "/opt/ros/kinetic/bin/rosrun four_wheel_drive_independent_steering waypoints_test" --geometry=45x12+475+476 &
sleep 0.1s

#gnome-terminal -e "/opt/ros/kinetic/bin/rosrun four_wheel_drive_independent_steering fwdis_mpc" --geometry=45x12+895+476 &
sleep 0.1s

gnome-terminal -e "/opt/ros/kinetic/bin/rosrun four_wheel_drive_independent_steering base_costmap" --geometry=45x12+1315+476 &
sleep 0.1s

gnome-terminal -e "/opt/ros/kinetic/bin/roslaunch four_wheel_drive_independent_steering collision_detector.launch" --geometry=45x12+0+702 &
sleep 0.1s

#gnome-terminal -e "/opt/ros/kinetic/bin/rosrun four_wheel_drive_independent_steering trajectory_logger" --geometry=45x12+475+702 &
sleep 0.1s
