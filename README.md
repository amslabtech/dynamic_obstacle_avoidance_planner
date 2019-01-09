# project_high_agility
[![Build Status](https://travis-ci.org/amslabtech/project_high_agility.svg?branch=master)](https://travis-ci.org/amslabtech/project_high_agility)


# for real robot
Push emergency stop switch.
### terminal0
roscore
### terminal1
cd {project_root_dir}

./run_docker.sh

cd catkin_ws

catkin_make

### terminal2
docker exec -it ros_mpc bash

roslaunch four_wheel_drive_independent_steering robot.launch

### terminal3
docker exec -it ros_mpc bash

roslaunch four_wheel_drive_independent_steering teleop.launch

### terminal4
rosrun joy joy_node /dev/ps4_usb

### terminal5
cd {project_root_dir}

roslaunch fwdis/launch/mbed.launch

after roslaunch, turn on the emergency stop switches.

### terminal6
roslaunch knm_tiny_power tkhsh_tiny.launch

### terminal7
rostopic pub /mbed/start std_msgs/Empty "{}"

ctrl+c

### When emergency stop

rostopic pub /mbed/reset std_msgs/Empty "{}"

Restart processes on terminal5-7
