#include <ros/ros.h>
#include <tf/tf.h>
#include <geometry_msgs/PoseArray.h>

std::string WORLD_FRAME;
int main(int argc, char** argv)
{
  ros::init(argc, argv, "waypoints_test");
  ros::NodeHandle nh;

  ros::NodeHandle local_nh("~");
  local_nh.getParam("/dynamic_avoidance/WORLD_FRAME", WORLD_FRAME);

  ros::Publisher waypoints_pub = nh.advertise<geometry_msgs::PoseArray>("/waypoints", 100);

  std::cout << "=== waypoints_test ===" << std::endl;
  geometry_msgs::PoseArray wp;
  wp.header.frame_id = WORLD_FRAME;
  wp.poses.resize(2);
  wp.poses[0].orientation = wp.poses[1].orientation = tf::createQuaternionMsgFromYaw(0);
  wp.poses[0].position.x = -15;
  wp.poses[0].position.y = 0;
  wp.poses[1].position.x = 5;
  wp.poses[1].position.y = 0;

  ros::Rate loop_rate(10);

  while(ros::ok()){
    waypoints_pub.publish(wp);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
