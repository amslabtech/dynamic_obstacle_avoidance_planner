#include <ros/ros.h>
#include <cppad/ipopt/solve.hpp>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "mpc_path_tracker");
  ros::NodeHandle nh;
  ros::NodeHandle local_nh("~");

  ros::Rate loop_rate(10);

  while(ros::ok()){

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
