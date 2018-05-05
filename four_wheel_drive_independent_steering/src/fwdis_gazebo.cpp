#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include "four_wheel_drive_independent_steering/FourWheelDriveIndependentSteering.h"

std_msgs::Float64 frw;
std_msgs::Float64 flw;
std_msgs::Float64 rrw;
std_msgs::Float64 rlw;
std_msgs::Float64 frs;
std_msgs::Float64 fls;
std_msgs::Float64 rrs;
std_msgs::Float64 rls;

four_wheel_drive_independent_steering::FourWheelDriveIndependentSteering command;

void command_callback(const four_wheel_drive_independent_steering::FourWheelDriveIndependentSteeringConstPtr &msg)
{
  command = *msg;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "fwdis_gazebo");
  ros::NodeHandle nh;

  ros::Publisher frw_pub = nh.advertise<std_msgs::Float64>("/fwdis/front_right_wheel_joint/command", 100);
  ros::Publisher flw_pub = nh.advertise<std_msgs::Float64>("/fwdis/front_left_wheel_joint/command", 100);
  ros::Publisher rrw_pub = nh.advertise<std_msgs::Float64>("/fwdis/rear_right_wheel_joint/command", 100);
  ros::Publisher rlw_pub = nh.advertise<std_msgs::Float64>("/fwdis/rear_left_wheel_joint/command", 100);
  ros::Publisher frs_pub = nh.advertise<std_msgs::Float64>("/fwdis/front_right_steering_joint/command", 100);
  ros::Publisher fls_pub = nh.advertise<std_msgs::Float64>("/fwdis/front_left_steering_joint/command", 100);
  ros::Publisher rrs_pub = nh.advertise<std_msgs::Float64>("/fwdis/rear_right_steering_joint/command", 100);
  ros::Publisher rls_pub = nh.advertise<std_msgs::Float64>("/fwdis/rear_left_steering_joint/command", 100);

  ros::Subscriber fwdis_sub = nh.subscribe("/fwdis/command", 100, command_callback);

  ros::Rate loop_rate(10);

  while(ros::ok()){
    frw.data = command.front_right_wheel_velocity;
    flw.data = command.front_left_wheel_velocity;
    rrw.data = command.rear_right_wheel_velocity;
    rlw.data = command.rear_left_wheel_velocity;
    frs.data = command.front_right_steering_angle;
    fls.data = command.front_left_steering_angle;
    rrs.data = command.rear_right_steering_angle;
    rls.data = command.rear_left_steering_angle;

    frw_pub.publish(frw);
    flw_pub.publish(flw);
    rrw_pub.publish(rrw);
    rlw_pub.publish(rlw);
    frs_pub.publish(frs);
    fls_pub.publish(fls);
    rrs_pub.publish(rrs);
    rls_pub.publish(rls);

    ros::spinOnce();
    loop_rate.sleep();

  }
}


