#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Path.h>

double MAX_VELOCITY;
double MAX_ACCELERATION;
double MAX_ANGULAR_VELOCITY;
double MAX_ANGULAR_ACCELERATION;
double VELOCITY_RESOLUTION;
double ANGULAR_VELOCITY_RESOLUTION;
double INTERVAL;
double DT;
double SIMULATE_TIME;
double ROBOT_RADIUS;
double GOAL_XY_TOLERANCE;
double GOAL_YAW_TOLERANCE;

double ALPHA;
double BETA;
double GAMMA;

double GOAL_DISTANCE;

double window_vx_max = MAX_VELOCITY;
double window_vx_min = -MAX_VELOCITY;
double window_vy_max = MAX_VELOCITY;
double window_vy_min = -MAX_VELOCITY;
double window_omega_max = MAX_ANGULAR_VELOCITY;
double window_omega_min = -MAX_ANGULAR_VELOCITY;

geometry_msgs::PoseStamped _goal;
geometry_msgs::PoseStamped goal;
geometry_msgs::Twist velocity;
std::vector<std::vector<std::vector<double> > > cost;
geometry_msgs::PoseArray local_path;
nav_msgs::Path global_path;
geometry_msgs::PoseStamped odometry;

void path_callback(const nav_msgs::PathConstPtr& msg)
{
  global_path = *msg;
}

void calculate_dynamic_window(void);
void generate_paths(void);
double evaluate_distance_to_local_goal(nav_msgs::Path);
double evaluate_distance_to_global_path(nav_msgs::Path);
double calcurate_distance(geometry_msgs::Pose, geometry_msgs::Pose);

geometry_msgs::Twist get_velocity(void);

int main(int argc, char** argv)
{
  ros::init(argc, argv, "holonomic_dwa");

  ros::NodeHandle nh;

  ros::NodeHandle local_nh("~");

  local_nh.getParam("MAX_VELOCITY", MAX_VELOCITY);
  local_nh.getParam("MAX_ACCELERATION", MAX_ACCELERATION);
  local_nh.getParam("MAX_ANGULAR_VELOCITY", MAX_ANGULAR_VELOCITY);
  local_nh.getParam("MAX_ANGULAR_ACCELERATION", MAX_ANGULAR_ACCELERATION);
  local_nh.getParam("VELOCITY_RESOLUTION", VELOCITY_RESOLUTION);
  local_nh.getParam("ANGULAR_VELOCITY_RESOLUTION", ANGULAR_VELOCITY_RESOLUTION);
  local_nh.getParam("INTERVAL", INTERVAL);
  local_nh.getParam("DT", DT);
  local_nh.getParam("SIMULATE_TIME", SIMULATE_TIME);
  local_nh.getParam("ROBOT_RADIUS", ROBOT_RADIUS);
  local_nh.getParam("GOAL_XY_TOLERANCE", GOAL_XY_TOLERANCE);
  local_nh.getParam("GOAL_YAW_TOLERANCE", GOAL_YAW_TOLERANCE);
  local_nh.getParam("ALPHA", ALPHA);
  local_nh.getParam("BETA", BETA);
  local_nh.getParam("GAMMA", GAMMA);
  local_nh.getParam("GOAL_DISTANCE", GOAL_DISTANCE);

  std::cout << MAX_VELOCITY << std::endl;
  std::cout << MAX_ACCELERATION << std::endl;
  std::cout << MAX_ANGULAR_VELOCITY << std::endl;
  std::cout << MAX_ANGULAR_ACCELERATION << std::endl;
  std::cout << VELOCITY_RESOLUTION << std::endl;
  std::cout << ANGULAR_VELOCITY_RESOLUTION << std::endl;
  std::cout << INTERVAL << std::endl;
  std::cout << DT << std::endl;
  std::cout << SIMULATE_TIME << std::endl;
  std::cout << ROBOT_RADIUS << std::endl;
  std::cout << GOAL_XY_TOLERANCE << std::endl;
  std::cout << GOAL_YAW_TOLERANCE << std::endl;
  std::cout << ALPHA << std::endl;
  std::cout << BETA << std::endl;
  std::cout << GAMMA << std::endl;
  std::cout << GOAL_DISTANCE << std::endl;

  ros::Publisher velocity_pub = nh.advertise<geometry_msgs::Twist>("/fwdis/velocity", 100);

  ros::Publisher path_pub = nh.advertise<geometry_msgs::PoseArray>("/fwdis/local_path", 100);

  ros::Publisher goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/fwdis/local_goal", 100);

  ros::Subscriber path_sub = nh.subscribe("/path", 100, path_callback);

  tf::TransformListener listener;
  tf::StampedTransform _transform;
  geometry_msgs::TransformStamped transform;

  velocity.linear.x = 0;
  velocity.linear.y = 0;
  velocity.angular.z = 0;

  _goal.header.frame_id = "odom";
  _goal.pose.position.x = 1;
  _goal.pose.orientation.z = 1;

  local_path.header.frame_id = "base_link";

  ros::Rate loop_rate(10);

  while(ros::ok()){
    bool transformed = false;
    try{
      listener.lookupTransform("odom", "base_link", ros::Time(0), _transform);
      tf::transformStampedTFToMsg(_transform, transform);
      odometry.header = transform.header;
      odometry.pose.position.x = transform.transform.translation.x;
      odometry.pose.position.y = transform.transform.translation.y;
      odometry.pose.position.z = transform.transform.translation.z;
      odometry.pose.orientation = transform.transform.rotation;

      transformed = true;
    }catch(tf::TransformException ex){
      ROS_ERROR("%s", ex.what());
      ros::Duration(1.0).sleep();
    }
    if(!global_path.poses.empty() && transformed){
      // local goal を決める部分
      try{
        for(int i=global_path.poses.size()-1;i>=0;i--){
          if(calcurate_distance(global_path.poses[i].pose, odometry.pose) < GOAL_DISTANCE){
            _goal = global_path.poses[i];
            break;
          }
        }
      }catch(tf::TransformException ex){
        ROS_ERROR("%s", ex.what());
        ros::Duration(1.0).sleep();
      }
      try{
        listener.transformPose("base_link", ros::Time(0), _goal, "odom", goal);
      }catch(tf::TransformException ex){
        std::cout << ex.what() << std::endl;
        ros::Duration(1.0).sleep();
      }

      goal_pub.publish(goal);
      //std::cout << goal << std::endl;
      calculate_dynamic_window();
      generate_paths();
      velocity = get_velocity();
      double distance = sqrt(goal.pose.position.x * goal.pose.position.x + goal.pose.position.y * goal.pose.position.y);
      if(distance < GOAL_XY_TOLERANCE){
        std::cout << "stop" << std::endl;
        velocity.linear.x = 0;
        velocity.linear.y = 0;
      }
      velocity_pub.publish(velocity);

      local_path.poses.clear();
      geometry_msgs::Pose pose;
      pose.position.x = 0;
      pose.position.y = 0;
      pose.orientation = tf::createQuaternionMsgFromYaw(0);
      int step_time = SIMULATE_TIME / (10 * DT);
      for(int t=0;t<step_time;t++){
        double yaw = tf::getYaw(pose.orientation);
        pose.position.x += (velocity.linear.x * cos(yaw) - velocity.linear.y * sin(yaw)) * (10 * DT);
        pose.position.y += (velocity.linear.x * sin(yaw) + velocity.linear.y * cos(yaw)) * (10 * DT);
        pose.orientation = tf::createQuaternionMsgFromYaw(yaw + velocity.angular.z * (10 * DT));
        local_path.poses.push_back(pose);
      }
      path_pub.publish(local_path);

      std::cout << "v: " << velocity.linear.x << ", " << velocity.linear.y << ", " << velocity.angular.z << std::endl;
    }

    loop_rate.sleep();
    ros::spinOnce();
  }
}

void calculate_dynamic_window(void)
{
  std::cout << "calculate_dynamic_window" << std::endl;
  geometry_msgs::Twist _velocity;
  _velocity = velocity;
  window_vx_max = _velocity.linear.x + INTERVAL * MAX_ACCELERATION;
  if(window_vx_max > MAX_VELOCITY){
    window_vx_max = MAX_VELOCITY;
  }
  window_vx_min = _velocity.linear.x - INTERVAL * MAX_ACCELERATION;
  if(window_vx_min < -MAX_VELOCITY){
    window_vx_min = -MAX_VELOCITY;
  }
  window_vy_max = _velocity.linear.y + INTERVAL * MAX_ACCELERATION;
  if(window_vy_max > MAX_VELOCITY){
    window_vy_max = MAX_VELOCITY;
  }
  window_vy_min = _velocity.linear.y - INTERVAL * MAX_ACCELERATION;
  if(window_vy_min < -MAX_VELOCITY){
    window_vy_min = -MAX_VELOCITY;
  }
  window_omega_max = _velocity.angular.z + INTERVAL * MAX_ANGULAR_ACCELERATION;
  if(window_omega_max > MAX_ANGULAR_VELOCITY){
    window_omega_max = MAX_ANGULAR_VELOCITY;
  }
  window_omega_min = _velocity.angular.z - INTERVAL * MAX_ANGULAR_ACCELERATION;
  if(window_omega_min < -MAX_ANGULAR_VELOCITY){
    window_omega_min = -MAX_ANGULAR_VELOCITY;
  }
}

void generate_paths(void)
{
  std::cout << "generate paths" << std::endl;
  int step_vx = (window_vx_max - window_vx_min) / VELOCITY_RESOLUTION + 1;
  int step_vy = (window_vy_max - window_vy_min) / VELOCITY_RESOLUTION + 1;
  int step_omega = (window_omega_max - window_omega_min) / ANGULAR_VELOCITY_RESOLUTION + 1;
  int step_time = SIMULATE_TIME / DT;

  /*
  std::cout << window_vx_max << ", ";
  std::cout << window_vx_min << std::endl;
  std::cout << window_vy_max << ", ";
  std::cout << window_vy_min << std::endl;
  std::cout << window_omega_max << ", ";
  std::cout << window_omega_min << std::endl;
  std::cout << step_vx << ", ";
  std::cout << step_vy << ", ";
  std::cout << step_omega << std::endl;
  */

  cost.clear();
  cost.resize(step_vx);
  for(int i=0;i<cost.size();i++){
    cost[i].resize(step_vy);
    for(int j=0;j<cost[i].size();j++){
      cost[i][j].resize(step_omega);
    }
  }

  std::cout << "generation start" << std::endl;
  for(int i=0;i<step_vx;i++){
    for(int j=0;j<step_vy;j++){
      for(int k=0;k<step_omega;k++){
        double vx = window_vx_min + i * VELOCITY_RESOLUTION;
        double vy = window_vy_min + j * VELOCITY_RESOLUTION;
        double omega = window_omega_min + k * ANGULAR_VELOCITY_RESOLUTION;
        if(vx * vx + vy * vy > MAX_VELOCITY * MAX_VELOCITY){
          cost[i][j][k] = -1;
          continue;
        }
        nav_msgs::Path path;
        path.header.frame_id = "base_link";
        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = "base_link";
        pose.pose.position.x = 0;
        pose.pose.position.y = 0;
        pose.pose.orientation = tf::createQuaternionMsgFromYaw(0);
        for(int t=0;t<step_time;t++){
          double yaw = tf::getYaw(pose.pose.orientation);
          pose.pose.position.x += (vx * cos(yaw) - vy * sin(yaw)) * DT;
          pose.pose.position.y += (vx * sin(yaw) + vy * cos(yaw)) * DT;
          pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw + omega * DT);
          path.poses.push_back(pose);
        }
        cost[i][j][k] = ALPHA * evaluate_distance_to_local_goal(path);
        cost[i][j][k] += BETA * evaluate_distance_to_global_path(path);
        //std::cout << cost[i][j][k] << ", ";
      }
      //std::cout << ":" << std::endl;
    }
    //std::cout << std::endl;
  }
  std::cout << "generation end" << std::endl;
}

double evaluate_distance_to_local_goal(nav_msgs::Path path)
{
  int length = path.poses.size() - 1;
  //double distance = calcurate_distance(goal.pose, path.poses[length].pose);
  double dx = goal.pose.position.x - path.poses[length].pose.position.x;
  double dy = goal.pose.position.y - path.poses[length].pose.position.y;
  double distance = dx * dx + dy * dy;
  distance += fabs(tf::getYaw(goal.pose.orientation) - tf::getYaw(path.poses[length].pose.orientation));
  return distance;
}

geometry_msgs::Twist get_velocity(void)
{
  std::cout << "get_velocity" << std::endl;
  geometry_msgs::Twist _velocity;
  int min_i = 0;
  int min_j = 0;
  int min_k = 0;
  double min_cost = 10000;

  for(int i=0;i<cost.size();i++){
    for(int j=0;j<cost[i].size();j++){
      for(int k=0;k<cost[i][j].size();k++){
        if(cost[i][j][k] > 0){
          if(cost[i][j][k] < min_cost){
            min_i = i;
            min_j = j;
            min_k = k;
            min_cost = cost[i][j][k];
          }
        }
      }
    }
  }

  std::cout << min_i << ", " << min_j << ", " << min_k << std::endl;

  _velocity.linear.x = window_vx_min + min_i * VELOCITY_RESOLUTION;
  _velocity.linear.y = window_vy_min + min_j * VELOCITY_RESOLUTION;
  _velocity.angular.z = window_omega_min + min_k * ANGULAR_VELOCITY_RESOLUTION;
  return _velocity;
}

double calcurate_distance(geometry_msgs::Pose a, geometry_msgs::Pose b)
{
  return sqrt((a.position.x - b.position.x) * (a.position.x - b.position.x) + (a.position.y - b.position.y) * (a.position.y - b.position.y));
}

double evaluate_distance_to_global_path(nav_msgs::Path local_path)
{
  double min_distance = 100;
  int local_path_length = local_path.poses.size() - 1;
  for(int i=0;i<global_path.poses.size();i++){
    double distance = calcurate_distance(local_path.poses[local_path_length].pose, global_path.poses[i].pose);
    if(distance < min_distance){
      min_distance = distance;
    }
  }
  return min_distance;
}
