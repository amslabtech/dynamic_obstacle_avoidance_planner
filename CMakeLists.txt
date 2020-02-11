cmake_minimum_required(VERSION 2.8.3)
project(dynamic_obstacle_avoidance_planner)

SET(CMAKE_CXX_FLAGS "-std=c++11 -O2 -g -Wall ${CMAKE_CXX_FLAGS}")

string(ASCII 27 ESC)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
  cmake_modules
  pcl_ros
)

find_package(Eigen3 REQUIRED COMPONENTS system)
find_package(PkgConfig REQUIRED)
pkg_check_modules(IPOPT ipopt)

if(${IPOPT_FOUND})
  message("-- ${ESC}[32mIPOPT was found!!!${ESC}[0m")
else()
  message("-- ${ESC}[31mIPOPT was NOT found!!!${ESC}[0m")
  message("-- ${ESC}[031mMPC nodes will not be built${ESC}[0m")
endif()

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES obstacle_tracker_kf
# CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
  ${IPOPT_INCLUDE_DIRS}
)

add_library(obstacle_tracker_kf
  src/obstacle_tracker_kf.cpp
)
add_executable(path_to_local_goal src/path_to_local_goal.cpp)
add_executable(diff_drive_sim src/diff_drive_sim.cpp)
add_executable(robot_predictor src/robot_predictor.cpp)
add_executable(dynamic_local_costmap_generator src/dynamic_local_costmap_generator.cpp)
add_executable(avoidance_path_planner src/avoidance_path_planner.cpp)
add_executable(waypoints_test src/waypoints_test.cpp)
add_executable(base_costmap_tf_publisher src/base_costmap_tf_publisher.cpp)
add_executable(collision_detector src/collision_detector.cpp)
add_executable(trajectory_logger src/trajectory_logger.cpp)
add_executable(velocity_arrow_to_obstacle_converter src/velocity_arrow_to_obstacle_converter.cpp)
add_executable(local_costmap_generator src/local_costmap_generator.cpp)
if(${IPOPT_FOUND})
  add_executable(diff_drive_mpc src/diff_drive_mpc.cpp)
  add_executable(fwdis_mpc src/fwdis_mpc.cpp)
endif()
add_executable(tf_to_obstacles src/tf_to_obstacles.cpp)
add_executable(static_local_costmap_generator src/static_local_costmap_generator.cpp)
add_executable(obstacle_tracker_kf_demo src/demo/obstacle_tracker_kf_demo.cpp)
add_executable(sfm_obstacle_simulator src/sfm_obstacle_simulator.cpp)

target_link_libraries(path_to_local_goal
  ${catkin_LIBRARIES}
)
target_link_libraries(diff_drive_sim
  ${catkin_LIBRARIES}
)
target_link_libraries(robot_predictor
  ${catkin_LIBRARIES}
)
target_link_libraries(dynamic_local_costmap_generator
  ${catkin_LIBRARIES}
  obstacle_tracker_kf
)
target_link_libraries(avoidance_path_planner
  ${catkin_LIBRARIES}
)
target_link_libraries(waypoints_test
  ${catkin_LIBRARIES}
)
target_link_libraries(base_costmap_tf_publisher
  ${catkin_LIBRARIES}
)
target_link_libraries(collision_detector
  ${catkin_LIBRARIES}
)
target_link_libraries(trajectory_logger
  ${catkin_LIBRARIES}
)
target_link_libraries(velocity_arrow_to_obstacle_converter
  ${catkin_LIBRARIES}
)
target_link_libraries(local_costmap_generator
  ${catkin_LIBRARIES}
)
if(${IPOPT_FOUND})
  target_link_libraries(diff_drive_mpc
    ${catkin_LIBRARIES}
    ${IPOPT_LIBRARIES}
  )
  target_link_libraries(fwdis_mpc
    ${catkin_LIBRARIES}
    ${IPOPT_LIBRARIES}
  )
endif()
target_link_libraries(tf_to_obstacles
  ${catkin_LIBRARIES}
)
target_link_libraries(static_local_costmap_generator
  ${catkin_LIBRARIES}
)
target_link_libraries(obstacle_tracker_kf_demo
  ${catkin_LIBRARIES}
  obstacle_tracker_kf
)
target_link_libraries(sfm_obstacle_simulator
  ${catkin_LIBRARIES}
)
