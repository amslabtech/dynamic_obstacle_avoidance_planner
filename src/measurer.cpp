#include "dynamic_obstacle_avoidance_planner/measurer.h"

Measurer::Measurer(void)
:local_nh("~")
{
    result_pub = nh.advertise<std_msgs::Float64MultiArray>("/result", 1);
    pose_sub = nh.subscribe("/pose", 1, &Measurer::pose_callback, this);

    local_nh.param<double>("GOAL_X", GOAL_X, {10.0});
    local_nh.param<double>("GOAL_Y", GOAL_Y, {0.0});
    local_nh.param<double>("GOAL_TOLERANCE", GOAL_TOLERANCE, {0.5});

    first_callback_flag = true;
    start_time = 0;
    last_time = 0;
    traveled_distance = 0;
    goal << GOAL_X, GOAL_Y;
    // std::cout << GOAL_X << ", " << GOAL_Y << ", " << GOAL_TOLERANCE << std::endl;
    // std::cout << goal << std::endl;
}

void Measurer::process(void)
{
    ros::spin();
}

void Measurer::pose_callback(const geometry_msgs::PoseStampedConstPtr& msg)
{
    static Eigen::Vector2d last_pose(msg->pose.position.x, msg->pose.position.y);
    Eigen::Vector2d current_pose(msg->pose.position.x, msg->pose.position.y);

    last_time = msg->header.stamp.toSec();
    if(!first_callback_flag){
        traveled_distance += (current_pose - last_pose).norm();
    }else{
        first_callback_flag = false;
        start_time = msg->header.stamp.toSec();
        traveled_distance = 0;
    }
    last_pose = current_pose;
    // std::cout << "cb:" << std::endl;
    // std::cout << current_pose.transpose() << std::endl;
    // std::cout << goal.transpose() << std::endl;
    if((current_pose - goal).norm() < GOAL_TOLERANCE){
        std::cout << "goal!!!" << std::endl;
        std_msgs::Float64MultiArray result;
        result.data.push_back(traveled_distance);
        result.data.push_back(last_time - start_time);
        result_pub.publish(result);
        ros::spinOnce();
        ros::shutdown();
    }
}

void Measurer::show_results(void)
{
    std::cout << "---" << std::endl;
    std::cout << "traveled distance: " << traveled_distance << "[m]" << std::endl;
    std::cout << "traveling time: " << last_time - start_time << "[s]" << std::endl;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "measurer");
    Measurer m;
    m.process();
    return 0;
}
