#ifndef __MEASURER_H
#define __MEASURER_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float64MultiArray.h>

#include <Eigen/Dense>

class Measurer
{
public:
    Measurer(void);

    void process(void);
    void pose_callback(const geometry_msgs::PoseStampedConstPtr&);
    void show_results(void);

protected:
    double GOAL_X;
    double GOAL_Y;
    double GOAL_TOLERANCE;

    ros::NodeHandle nh;
    ros::NodeHandle local_nh;

    ros::Publisher result_pub;
    ros::Subscriber pose_sub;

    bool first_callback_flag;
    double start_time;
    double last_time;
    double traveled_distance;
    Eigen::Vector2d goal;
};

#endif// __MEASURER_H
