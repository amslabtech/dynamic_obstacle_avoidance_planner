#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

std::string WORLD_FRAME;
std::string ROBOT_FRAME;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "base_costmap");
  ros::NodeHandle nh;

  ros::NodeHandle local_nh("~");
  local_nh.getParam("/dynamic_avoidance/ROBOT_FRAME", ROBOT_FRAME);
  local_nh.getParam("/dynamic_avoidance/WORLD_FRAME", WORLD_FRAME);

  tf::TransformBroadcaster br;

  geometry_msgs::TransformStamped base_costmap;

  tf::TransformListener listener;

  base_costmap.header.frame_id = WORLD_FRAME;
  base_costmap.header.stamp = ros::Time::now();
  base_costmap.child_frame_id = "local_costmap";
  //base_costmap.transform.translation.x = 0;
  //base_costmap.transform.translation.y = 0;
  base_costmap.transform.rotation = tf::createQuaternionMsgFromYaw(0);
  //br.sendTransform(base_costmap);

  ros::Rate loop_rate(100);

  std::cout << "base_link to local_costmap" << std::endl;

  while(ros::ok()){
    bool transformed = false;
    tf::StampedTransform transform;
    try{
      listener.lookupTransform(WORLD_FRAME, ROBOT_FRAME, ros::Time(0), transform);
      transformed = true;
    }catch(tf::TransformException ex){
      std::cout << ex.what() << std::endl;
    }
    if(transformed){
      base_costmap.header.stamp = ros::Time::now();
      base_costmap.transform.translation.x = transform.getOrigin().getX();
      base_costmap.transform.translation.y = transform.getOrigin().getY();
      //base_costmap.transform.rotation = tf::createQuaternionMsgFromYaw(-tf::getYaw(transform.getRotation()));
      base_costmap.transform.rotation = tf::createQuaternionMsgFromYaw(0);
      br.sendTransform(base_costmap);
    }

    loop_rate.sleep();
    ros::spinOnce();
  }
  return 0;
}
