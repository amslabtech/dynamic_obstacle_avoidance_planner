#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <geometry_msgs/TransformStamped.h>

double GOAL_DISTANCE;

nav_msgs::Path path;

void path_callback(const nav_msgs::PathConstPtr& msg)
{
  path = *msg;
}

double calcurate_distance(geometry_msgs::Pose, geometry_msgs::Pose);

int main(int argc, char** argv)
{
  ros::init(argc, argv, "path_to_local_goal");
  ros::NodeHandle nh;
  ros::NodeHandle local_nh("~");

  local_nh.getParam("GOAL_DISTANCE", GOAL_DISTANCE);

  ros::Publisher goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/goal", 100);
  ros::Subscriber path_sub = nh.subscribe("/path", 100, path_callback);

  tf::TransformListener listener;
  tf::StampedTransform _transform;
  geometry_msgs::TransformStamped transform;
  geometry_msgs::PoseStamped odometry;

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
    }

    if(!path.poses.empty() && transformed){
      for(int i=path.poses.size()-1;i>=0;i--){
        if(calcurate_distance(path.poses[i].pose, odometry.pose) < GOAL_DISTANCE){
          goal_pub.publish(path.poses[i]);
          break;
        }
      }
    }

    loop_rate.sleep();
    ros::spinOnce();
  }
  return 0;
}

double calcurate_distance(geometry_msgs::Pose a, geometry_msgs::Pose b)
{
  return sqrt((a.position.x - b.position.x) * (a.position.x - b.position.x) + (a.position.y - b.position.y) * (a.position.y - b.position.y));
}
