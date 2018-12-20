#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseArray.h>


double HZ = 100.0;

visualization_msgs::Marker lines;

void path_callback(const geometry_msgs::PoseArrayConstPtr& msg)
{
  lines.points.clear();
  geometry_msgs::PoseArray pose_array;
  pose_array = *msg;
  lines.header = pose_array.header;
  lines.color.r = 0;
  lines.color.g = 0;
  lines.color.b = 1;
  lines.color.a = 1;
  for(int i=0;i<36;i++){
    lines.points.push_back(pose_array.poses[i].position);
    lines.points.push_back(pose_array.poses[i+36].position);
    lines.points.push_back(pose_array.poses[i].position);
    lines.points.push_back(pose_array.poses[i+72].position);
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "trajectory_logger");
  ros::NodeHandle nh;

  std::cout << "=== trajectory_logger ===" << std::endl;

  ros::Rate loop_rate(HZ);

  tf::TransformListener listener;

  ros::Publisher robot_path_pub = nh.advertise<nav_msgs::Path>("/trajectory/robot", 1);
  ros::Publisher obs0_path_pub = nh.advertise<nav_msgs::Path>("/trajectory/obs0", 1);
  ros::Publisher robot_viz_pub = nh.advertise<visualization_msgs::Marker>("/marker/robot", 1);
  ros::Publisher obs0_viz_pub = nh.advertise<visualization_msgs::Marker>("/marker/obs0", 1);
  ros::Publisher robot_viz_array_pub = nh.advertise<visualization_msgs::MarkerArray>("/markers/robot", 1);
  ros::Publisher obs0_viz_array_pub = nh.advertise<visualization_msgs::MarkerArray>("/markers/obs0", 1);

  ros::Subscriber robot_path_sub = nh.subscribe("/robot_predicted_path", 1, path_callback);
  ros::Publisher robot_line_pub = nh.advertise<visualization_msgs::Marker>("/lines", 1);

  nav_msgs::Path robot_path;
  nav_msgs::Path obs0_path;
  visualization_msgs::MarkerArray robot_viz;
  visualization_msgs::MarkerArray obs0_viz;

  robot_path.header.frame_id = "world";
  obs0_path.header.frame_id = "world";

  geometry_msgs::PoseStamped pose;
  visualization_msgs::Marker viz;
  viz.scale.x = 0.6;
  viz.scale.y = 0.6;
  viz.scale.z = 0.6;
  viz.type = visualization_msgs::Marker::CYLINDER;
  viz.action = visualization_msgs::Marker::ADD;
  viz.header.frame_id = "world";
  viz.ns = "robot_viz";

  lines.type = visualization_msgs::Marker::LINE_LIST;
  lines.lifetime = ros::Duration(0.01);
  lines.action = visualization_msgs::Marker::ADD;
  lines.header.frame_id = "world";
  lines.ns = "lines";
  lines.scale.x = 0.01;

  int count = 0;

  while(ros::ok()){
    bool transformed = false;
    tf::StampedTransform robot_transform;
    tf::StampedTransform obs0_transform;
    try{
      listener.lookupTransform("world", "/vicon/base_link/base_link", ros::Time(0), robot_transform);
      listener.lookupTransform("world", "/vicon/obs/obs", ros::Time(0), obs0_transform);
      transformed = true;
    }catch(tf::TransformException ex){
      std::cout << ex.what() << std::endl;
    }
    if(transformed){
      robot_path.header.stamp = robot_transform.stamp_;
      pose.header.frame_id = "world";
      pose.header.stamp = robot_transform.stamp_;
      pose.pose.position.x = robot_transform.getOrigin().getX();
      pose.pose.position.y = robot_transform.getOrigin().getY();
      pose.pose.orientation.x = robot_transform.getRotation().getX();
      pose.pose.orientation.y = robot_transform.getRotation().getY();
      pose.pose.orientation.z = robot_transform.getRotation().getZ();
      pose.pose.orientation.w = robot_transform.getRotation().getW();
      robot_path.poses.push_back(pose);
      viz.header.stamp = ros::Time::now();
      viz.pose = pose.pose;
      viz.scale.z = 0.2;
      viz.color.r = 0;
      viz.color.g = 1;
      viz.color.b = 0;
      viz.color.a = 1;
      viz.lifetime = ros::Duration(0.01);
      robot_viz_pub.publish(viz);
      viz.lifetime = ros::Duration(0);
      if(count%10==0){
        viz.id = count;
        robot_viz.markers.push_back(viz);
      }

      obs0_path.header.stamp = obs0_transform.stamp_;
      pose.header.frame_id = "world";
      pose.header.stamp = obs0_transform.stamp_;
      pose.pose.position.x = obs0_transform.getOrigin().getX();
      pose.pose.position.y = obs0_transform.getOrigin().getY();
      pose.pose.orientation.x = obs0_transform.getRotation().getX();
      pose.pose.orientation.y = obs0_transform.getRotation().getY();
      pose.pose.orientation.z = obs0_transform.getRotation().getZ();
      pose.pose.orientation.w = obs0_transform.getRotation().getW();
      obs0_path.poses.push_back(pose);
      viz.header.stamp = ros::Time::now();
      viz.pose = pose.pose;
      viz.scale.z = 0.1;
      viz.color.r = 1;
      viz.color.g = 0;
      viz.color.b = 0;
      viz.color.a = 1;
      viz.lifetime = ros::Duration(0.01);
      obs0_viz_pub.publish(viz);
      viz.lifetime = ros::Duration(0);
      if(count%10==0){
        viz.id = count;
        obs0_viz.markers.push_back(viz);
      }
      count++;
      transformed = false;
    }
    robot_path_pub.publish(robot_path);
    obs0_path_pub.publish(obs0_path);
    robot_viz_array_pub.publish(robot_viz);
    obs0_viz_array_pub.publish(obs0_viz);
    robot_line_pub.publish(lines);

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
