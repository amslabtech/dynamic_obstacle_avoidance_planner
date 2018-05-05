#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/JointState.h>
#include "four_wheel_drive_independent_steering/FourWheelDriveIndependentSteering.h"

std_msgs::Float64 frw;
std_msgs::Float64 flw;
std_msgs::Float64 rrw;
std_msgs::Float64 rlw;
std_msgs::Float64 frs;
std_msgs::Float64 fls;
std_msgs::Float64 rrs;
std_msgs::Float64 rls;

enum JOINTS
{
  FLS,
  FLW,
  FRS,
  FRW,
  RLS,
  RLW,
  RRS,
  RRW,
};

four_wheel_drive_independent_steering::FourWheelDriveIndependentSteering command;
nav_msgs::Odometry odometry;
sensor_msgs::JointState joint;
bool joint_subscribed = false;

const double WHEEL_RADIUS = 0.075;
const double WHEEL_BASE = 0.50;
const double TREAD = 0.50;
const double INTERVAL = 0.1;

void command_callback(const four_wheel_drive_independent_steering::FourWheelDriveIndependentSteeringConstPtr &msg)
{
  command = *msg;
}

void joint_callback(const sensor_msgs::JointStateConstPtr &msg)
{
  joint = *msg;
  joint_subscribed = true;
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
  ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("/fwdis/odometry", 100);

  ros::Subscriber fwdis_sub = nh.subscribe("/fwdis/command", 100, command_callback);
  ros::Subscriber joint_sub = nh.subscribe("/fwdis/joint_states", 100, joint_callback);

  tf::TransformBroadcaster odom_broadcaster;

  odometry.header.frame_id = "odom";
  odometry.child_frame_id = "base_link";
  odometry.pose.pose.orientation = tf::createQuaternionMsgFromYaw(0);

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

    if(joint_subscribed){
      double vfr = joint.velocity[FRW] * WHEEL_RADIUS;
      double vfl = joint.velocity[FLW] * WHEEL_RADIUS;
      double vrl = joint.velocity[RLW] * WHEEL_RADIUS;
      double vrr = joint.velocity[RRW] * WHEEL_RADIUS;
      double vfx = (vfr * cos(joint.position[FRS]) + vfl * cos(joint.position[FLS])) / 2.0;
      double vfy = (vfr * sin(joint.position[FRS]) + vfl * sin(joint.position[FLS])) / 2.0;
      double vrx = (vrr * cos(joint.position[RRS]) + vrl * cos(joint.position[RLS])) / 2.0;
      double vry = (vrr * sin(joint.position[RRS]) + vrl * sin(joint.position[RLS])) / 2.0;

      double omega = (vfy - vry) / WHEEL_BASE;
      double vx = (vfx + vrx) / 2.0;
      double vy = (vfy + vry) / 2.0;
      double yaw = tf::getYaw(odometry.pose.pose.orientation);
      odometry.pose.pose.position.x += (vx * cos(yaw) - vy * sin(yaw)) * INTERVAL;
      odometry.pose.pose.position.y += (vx * sin(yaw) + vy * cos(yaw)) * INTERVAL;
      odometry.pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw + omega * INTERVAL);
      odometry.header.stamp = ros::Time::now();
      odom_pub.publish(odometry);
      geometry_msgs::TransformStamped odom_tf;
      odom_tf.header = odometry.header;
      odom_tf.child_frame_id = odometry.child_frame_id;
      odom_tf.transform.translation.x = odometry.pose.pose.position.x;
      odom_tf.transform.translation.y = odometry.pose.pose.position.y;
      odom_tf.transform.rotation = odometry.pose.pose.orientation;
      odom_broadcaster.sendTransform(odom_tf);
    }

    ros::spinOnce();
    loop_rate.sleep();

  }
}


