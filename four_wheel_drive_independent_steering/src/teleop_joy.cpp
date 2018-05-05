#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>

sensor_msgs::Joy joy_data;

const double MAX_VELOCITY = 2.0;//[m/s]
const double MAX_ANGLAR_VELOCITY = 4.0;//[rad/s]

void joy_callback(const sensor_msgs::JoyConstPtr& msg)
{
  joy_data = *msg;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop_joy");
  ros::NodeHandle nh;

  ros::Publisher velocity_pub = nh.advertise<geometry_msgs::Twist>("/fwdis/velocity", 100);

  ros::Subscriber joy_sub = nh.subscribe("/joy", 100, joy_callback);

  ros::Rate loop_rate(10);

  while(ros::ok()){
    if(!joy_data.axes.empty()){
      geometry_msgs::Twist velocity;

      if((joy_data.axes[1] == 0.0) && (joy_data.axes[0] == 0.0)){
        if(joy_data.buttons[6] || joy_data.buttons[7]){
          double omega = (joy_data.axes[4] - joy_data.axes[3]) / 2.0;
          velocity.angular.z = omega * MAX_ANGLAR_VELOCITY;
        }
      }else{
        velocity.linear.x = joy_data.axes[1] * MAX_VELOCITY;
        velocity.linear.y = joy_data.axes[0] * MAX_VELOCITY;
        double omega = (joy_data.axes[4] - joy_data.axes[3]) / 2.0;
        velocity.angular.z = omega * MAX_ANGLAR_VELOCITY;
      }

      velocity_pub.publish(velocity);
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
