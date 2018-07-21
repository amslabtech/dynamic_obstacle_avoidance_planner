#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <tf/tf.h>

double LENGTH;//[m]
double A;//[m]
double FREQUENCY;//[/s]
const double STEP = 0.05;//[m]

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sin_curve");
  ros::NodeHandle nh;
  ros::NodeHandle local_nh("~");

  local_nh.getParam("LENGTH", LENGTH);
  local_nh.getParam("A", A);
  local_nh.getParam("FREQUENCY", FREQUENCY);

  ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("/path", 100);

  ros::Rate loop_rate(10);

  while(ros::ok()){
    nav_msgs::Path path;
    path.header.frame_id = "odom";

    for(double x=0;x<LENGTH;x+=STEP){
      double y = A * sin(FREQUENCY * x / LENGTH * 2.0 * M_PI);
      geometry_msgs::PoseStamped pose;
      pose.header.frame_id = "odom";
      pose.pose.position.x = x;
      pose.pose.position.y = y;
      pose.pose.orientation = tf::createQuaternionMsgFromYaw(0);
      path.poses.push_back(pose);
    }
    path_pub.publish(path);

    loop_rate.sleep();
    ros::spinOnce();
  }
  return 0;
}
