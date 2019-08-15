# dynamic_obstacle_avoidance_planner
[![Build Status](https://travis-ci.org/amslabtech/dynamic_obstacle_avoidance_planner.svg?branch=master)](https://travis-ci.org/amslabtech/dynamic_obstacle_avoidance_planner)

Collision prediction based dynamic obstacle avoidance planner

## Environment
- Ubuntu16.04 or Ubuntu18.04

## Requirement
- Docker

Using docker is recommended. If you don't want to use docker, see Dockerfile and Reference(below) to setup.

## Install and Build
```
$ cd your/workspace
$ git clone https://github.com/amslabtech/dynamic_obstacle_avoidance_planner.git
$ ./build.sh
```

## How to Use
```
$ run_docker.sh
$ cd catkin_ws && catkin_make

open another terminal

for collision prediction, local costmap generation, and avoidance path planning
$ docker exec -it ros_mpc bash -c "source /ros_entrypoint.sh && roslaunch dynamic_obstacle_avoidance_planner dynamic_obstacle_avoidance_core.launch"
```

## Reference
- https://github.com/coin-or/Ipopt
- https://coin-or.github.io/CppAD/doc/cppad.htm
