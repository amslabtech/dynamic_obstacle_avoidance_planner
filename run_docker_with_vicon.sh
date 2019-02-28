#!/bin/bash

echo "=== run_docker ==="

xhost +local:docker

docker run -it --rm \
  --env=QT_X11_NO_MITSHM=1 \
  --env=DISPLAY=$DISPLAY \
  --env=ROS_MASTER_URI=http://192.168.10.102:11311 \
  --env=ROS_IP=192.168.0.245 \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  --volume="${PWD}:/root/catkin_ws/src/fwdis" \
  --net='host' \
  --name="ros_mpc" \
  kazukitakahashi/ros_mpc

