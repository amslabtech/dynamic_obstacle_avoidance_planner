#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>

geometry_msgs::Twist velocity;
bool velocity_subscribed = false;

const double WHEEL_RADIUS = 0.075;//[m]

void velocity_callback(const geometry_msgs::TwistConstPtr &msg)
{
  velocity = *msg;
  velocity_subscribed = true;
}

std_msgs::Float64 _angle;
std_msgs::Float64 _velocity;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "fwdis_controller");
  ros::NodeHandle nh;

  ros::Publisher frw_pub = nh.advertise<std_msgs::Float64>("/mygazebo/front_right_wheel_joint/command", 100);
  ros::Publisher flw_pub = nh.advertise<std_msgs::Float64>("/mygazebo/front_left_wheel_joint/command", 100);
  ros::Publisher rrw_pub = nh.advertise<std_msgs::Float64>("/mygazebo/rear_right_wheel_joint/command", 100);
  ros::Publisher rlw_pub = nh.advertise<std_msgs::Float64>("/mygazebo/rear_left_wheel_joint/command", 100);
  ros::Publisher frs_pub = nh.advertise<std_msgs::Float64>("/mygazebo/front_right_steering_joint/command", 100);
  ros::Publisher fls_pub = nh.advertise<std_msgs::Float64>("/mygazebo/front_left_steering_joint/command", 100);
  ros::Publisher rrs_pub = nh.advertise<std_msgs::Float64>("/mygazebo/rear_right_steering_joint/command", 100);
  ros::Publisher rls_pub = nh.advertise<std_msgs::Float64>("/mygazebo/rear_left_steering_joint/command", 100);

  ros::Subscriber velocity_sub = nh.subscribe("/mygazebo/velocity", 100, velocity_callback);

  ros::Rate loop_rate(10);

  while(ros::ok()){
    if(velocity_subscribed){
      if((velocity.linear.x == 0.0) && (velocity.linear.y == 0.0)){
        _velocity.data = 0;
        frw_pub.publish(_velocity);
        flw_pub.publish(_velocity);
        rrw_pub.publish(_velocity);
        rlw_pub.publish(_velocity);
      }else{
        std::cout << "loop" << std::endl;
        _angle.data = atan2(velocity.linear.y, velocity.linear.x);
        bool inverce_flag = false;
        if(_angle.data > M_PI/1.5){
          _angle.data -= M_PI;
          inverce_flag = true;
        }else if(_angle.data < -M_PI/1.5){
          _angle.data += M_PI;
          inverce_flag = true;
        }
        frs_pub.publish(_angle);
        fls_pub.publish(_angle);
        rrs_pub.publish(_angle);
        rls_pub.publish(_angle);

        _velocity.data = sqrt(velocity.linear.x * velocity.linear.x + velocity.linear.y * velocity.linear.y) / WHEEL_RADIUS;
        if(inverce_flag){
          _velocity.data = -_velocity.data;
        }
        frw_pub.publish(_velocity);
        flw_pub.publish(_velocity);
        rrw_pub.publish(_velocity);
        rlw_pub.publish(_velocity);
      }
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
