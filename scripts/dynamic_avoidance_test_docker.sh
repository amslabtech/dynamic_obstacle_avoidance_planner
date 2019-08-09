#!/bin/bash

gnome-terminal -e "/opt/ros/${ROS_DISTRO}/bin/roscore" --geometry=45x12+0+0 &
sleep 1.0s

gnome-terminal -e "docker exec -it ros_mpc /bin/bash -c 'ldconfig && source /root/catkin_ws/devel/setup.bash && /opt/ros/kinetic/bin/roslaunch dynamic_obstacle_avoidance_planner load_param_diff_drive.launch'" --geometry=45x12+0+0 &
sleep 1.0s

gnome-terminal -e "docker exec -it ros_mpc /bin/bash -c 'ldconfig && source /root/catkin_ws/devel/setup.bash && /opt/ros/kinetic/bin/roslaunch dynamic_obstacle_avoidance_planner load_param_dynamic_avoidance.launch'" --geometry=45x12+0+0 &
sleep 1.0s

gnome-terminal -e "/opt/ros/${ROS_DISTRO}/bin/rosrun rviz rviz -d ../config/dynamic_avoidance.rviz" --geometry=45x12+475+0 &
sleep 0.5s

# robot sim
gnome-terminal -e "docker exec -it ros_mpc /bin/bash -c 'ldconfig && source /root/catkin_ws/devel/setup.bash && /opt/ros/kinetic/bin/rosrun dynamic_obstacle_avoidance_planner sim_3dof.py'" --geometry=45x12+895+0 &
sleep 0.1s

# obstacle sim
gnome-terminal -e "docker exec ros_mpc /bin/bash -c 'ldconfig && source /root/catkin_ws/devel/setup.bash && /opt/ros/kinetic/bin/roslaunch dynamic_obstacle_avoidance_planner diff_drive_sim.launch'" --geometry=45x12+1315+0 &
sleep 0.1s

gnome-terminal -e "docker exec ros_mpc /bin/bash -c 'ldconfig && source /root/catkin_ws/devel/setup.bash && /opt/ros/kinetic/bin/rosrun dynamic_obstacle_avoidance_planner obstacle_predictor_kf'" --geometry=45x12+0+250 &
sleep 0.1s

gnome-terminal -e "docker exec ros_mpc /bin/bash -c 'ldconfig && source /root/catkin_ws/devel/setup.bash && /opt/ros/kinetic/bin/roslaunch dynamic_obstacle_avoidance_planner robot_predictor.launch'" --geometry=45x12+475+250 &
sleep 0.1s

WORLD_FRAME=$(/opt/ros/${ROS_DISTRO}/bin/rosparam get /dynamic_avoidance/WORLD_FRAME)
gnome-terminal -e "docker exec ros_mpc /bin/bash -c 'ldconfig && source /root/catkin_ws/devel/setup.bash && /opt/ros/kinetic/bin/rosrun tf static_transform_publisher 0 0 0 0 0 0 $WORLD_FRAME odom 100'" --geometry=45x12+895+250 &
sleep 0.1s

gnome-terminal -e "docker exec ros_mpc /bin/bash -c 'ldconfig && source /root/catkin_ws/devel/setup.bash && /opt/ros/kinetic/bin/roslaunch dynamic_obstacle_avoidance_planner dynamic_local_costmap.launch'" --geometry=45x12+1315+250 &
sleep 0.1s

gnome-terminal -e "docker exec ros_mpc /bin/bash -c 'ldconfig && source /root/catkin_ws/devel/setup.bash && /opt/ros/kinetic/bin/roslaunch dynamic_obstacle_avoidance_planner avoidance_path_planner.launch'" --geometry=45x12+0+476 &
sleep 0.1s

# for test
gnome-terminal -e "docker exec ros_mpc /bin/bash -c 'ldconfig && source /root/catkin_ws/devel/setup.bash && /opt/ros/kinetic/bin/rosrun dynamic_obstacle_avoidance_planner waypoints_test'" --geometry=45x12+475+476 &
sleep 0.1s

gnome-terminal -e "docker exec ros_mpc /bin/bash -c 'ldconfig && source /root/catkin_ws/devel/setup.bash && /opt/ros/kinetic/bin/roslaunch dynamic_obstacle_avoidance_planner diff_drive_mpc.launch'" --geometry=45x12+895+476 &
sleep 0.1s

gnome-terminal -e "docker exec ros_mpc /bin/bash -c 'ldconfig && source /root/catkin_ws/devel/setup.bash && /opt/ros/kinetic/bin/rosrun dynamic_obstacle_avoidance_planner base_costmap_tf_publisher'" --geometry=45x12+1315+476 &
sleep 0.1s

gnome-terminal -e "docker exec ros_mpc /bin/bash -c 'ldconfig && source /root/catkin_ws/devel/setup.bash && /opt/ros/kinetic/bin/roslaunch dynamic_obstacle_avoidance_planner collision_detector.launch'" --geometry=45x12+0+702 &
sleep 0.1s

#gnome-terminal -e "docker exec ros_mpc /bin/bash -c 'ldconfig && source /root/catkin_ws/devel/setup.bash && /opt/ros/kinetic/bin/rorun dynamic_obstacle_avoidance_planner trajectory_logger'" --geometry=45x12+475+702 &
sleep 0.1s
