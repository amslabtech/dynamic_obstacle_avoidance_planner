#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseArray.h>

double MAX_VELOCITY;
double MAX_ACCELERATION;
double MAX_ANGULAR_VELOCITY;
double MAX_ANGULAR_ACCELERATION;
double VELOCITY_RESOLUTION;
double ANGULAR_VELOCTIY_RESOLUTION;
const double INTERVAL = 0.100;
double SIMULATE_TIME;
double ROBOT_RADIUS;
double GOAL_XY_TOLERANCE;
double GOAL_YAW_TOLERANCE;

double ALPHA;
double BETA;
double GAMMA;

double window_vx_max = MAX_VELOCITY;
double window_vx_min = -MAX_VELOCITY;
double window_vy_max = MAX_VELOCITY;
double window_vy_min = -MAX_VELOCITY;
double window_omega_max = MAX_ANGULAR_VELOCITY;
double window_omega_min = -MAX_ANGULAR_VELOCITY;

geometry_msgs::PoseStamped goal;
bool goal_subscribed = false;


void goal_callback(const geometry_msgs::PoseStampedConstPtr &msg)
{
  goal = *msg;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "holonomic_dwa");

  ros::NodeHandle nh;

  ros::NodeHandle local_nh;

  local_nh.getParam("MAX_VELOCITY", MAX_VELOCITY);
  local_nh.getParam("MAX_ACCELERATION", MAX_ACCELERATION);
  local_nh.getParam("MAX_ANGULAR_VELOCITY", MAX_ANGULAR_VELOCITY);
  local_nh.getParam("MAX_ANGULAR_ACCELERATION", MAX_ANGULAR_ACCELERATION);
  local_nh.getParam("VELOCITY_RESOLUTION", VELOCITY_RESOLUTION);
  local_nh.getParam("ANGULAR_VELOCITY_RESOLUTION", ANGULAR_VELOCTIY_RESOLUTION);
  local_nh.getParam("SIMULATE_TIME", SIMULATE_TIME);
  local_nh.getParam("ROBOT_RADIUS", ROBOT_RADIUS);
  local_nh.getParam("GOAL_XY_TOLERANCE", GOAL_XY_TOLERANCE);
  local_nh.getParam("GOAL_YAW_TOLERANCE", GOAL_YAW_TOLERANCE);
  local_nh.getParam("ALPHA", ALPHA);
  local_nh.getParam("BETA", BETA);
  local_nh.getParam("GAMMA", GAMMA);


  ros::Publisher velocity_pub = nh.advertise<geometry_msgs::Twist>("/fwdis/velocity", 100);

  ros::Subscriber goal_sub = nh.subscribe("/fwdis/local_goal", 100, goal_callback);

  ros::Rate loop_rate(10);

  while(ros::ok()){

    loop_rate.sleep();
    ros::spinOnce();
  }
}
