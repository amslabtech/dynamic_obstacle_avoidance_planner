#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>

sensor_msgs::Joy joy_data;
geometry_msgs::Twist velocity;

void joy_callback(const sensor_msgs::JoyConstPtr& msg)
{
  joy_data = *msg;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop_joy");
  ros::NodeHandle nh;

  ros::Publisher velocity_pub = nh.advertise<geometry_msgs::Twist>("/mygazebo/velocity", 100);

  ros::Subscriber joy_sub = nh.subscribe("/joy", 100, joy_callback);

  ros::Rate loop_rate(10);

  while(ros::ok()){
    if(!joy_data.axes.empty()){
      velocity.linear.x = joy_data.axes[1] * 25;
      velocity.linear.y = joy_data.axes[0] * 25;
      velocity_pub.publish(velocity);
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
