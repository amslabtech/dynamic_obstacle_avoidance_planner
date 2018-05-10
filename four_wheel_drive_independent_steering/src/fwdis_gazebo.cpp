#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/JointState.h>
#include <Eigen/Dense>
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
const double RADIUS = sqrt(pow(WHEEL_BASE, 2) + pow(TREAD, 2)) / 2.0;
const double THETA = atan(TREAD / WHEEL_BASE);
const double INTERVAL = 0.01;

Eigen::MatrixXd forward_matrix;
Eigen::MatrixXd inversed_matrix;
Eigen::VectorXd wheel_velocity;//V1x, V1y, ...
Eigen::Vector3d robot_velocity;//Vx, Vy, w

void command_callback(const four_wheel_drive_independent_steering::FourWheelDriveIndependentSteeringConstPtr &msg)
{
  command = *msg;
}

void joint_callback(const sensor_msgs::JointStateConstPtr &msg)
{
  joint = *msg;
  joint_subscribed = true;
}

//https://robotics.naist.jp/edu/text/?Robotics%2FEigen#b3b26d13
template <typename t_matrix>
t_matrix PseudoInverse(const t_matrix& m, const double &tolerance=1.e-6)
{
  using namespace Eigen;
  typedef JacobiSVD<t_matrix> TSVD;
  unsigned int svd_opt(ComputeThinU | ComputeThinV);
  if(m.RowsAtCompileTime!=Dynamic || m.ColsAtCompileTime!=Dynamic)
  svd_opt= ComputeFullU | ComputeFullV;
  TSVD svd(m, svd_opt);
  const typename TSVD::SingularValuesType &sigma(svd.singularValues());
  typename TSVD::SingularValuesType sigma_inv(sigma.size());
  for(long i=0; i<sigma.size(); ++i)
  {
    if(sigma(i) > tolerance)
      sigma_inv(i)= 1.0/sigma(i);
    else
      sigma_inv(i)= 0.0;
  }
  return svd.matrixV()*sigma_inv.asDiagonal()*svd.matrixU().transpose();
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

  std::cout << "initialize matrix" << std::endl;

  forward_matrix.resize(8, 3);
  forward_matrix << 1.0, 0.0,  RADIUS * sin(THETA),
                    0.0, 1.0,  RADIUS * cos(THETA),
                    1.0, 0.0, -RADIUS * sin(THETA),
                    0.0, 1.0,  RADIUS * cos(THETA),
                    1.0, 0.0, -RADIUS * sin(THETA),
                    0.0, 1.0, -RADIUS * cos(THETA),
                    1.0, 0.0,  RADIUS * sin(THETA),
                    0.0, 1.0, -RADIUS * cos(THETA);


  std::cout << "forward:" << std::endl;
  std::cout << forward_matrix << std::endl;

  inversed_matrix.resize(3, 8);
  inversed_matrix = PseudoInverse(forward_matrix);

  wheel_velocity.resize(8, 1);

  std::cout << "inversed:" << std::endl;
  std::cout << inversed_matrix << std::endl;

  ros::Rate loop_rate(1.0 / INTERVAL);

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

      wheel_velocity << vfr * cos(joint.position[FRS]),
                        vfr * sin(joint.position[FRS]),
                        vfl * cos(joint.position[FLS]),
                        vfl * sin(joint.position[FLS]),
                        vrl * cos(joint.position[RLS]),
                        vrl * sin(joint.position[RLS]),
                        vrr * cos(joint.position[RRS]),
                        vrr * sin(joint.position[RRS]);

      robot_velocity = inversed_matrix * wheel_velocity;

      double omega = robot_velocity(2);
      double vx = robot_velocity(0);
      double vy = robot_velocity(1);
      std::cout << "(vx, vy, w) = " << vx << ", " << vy << ", " << omega << std::endl;

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


