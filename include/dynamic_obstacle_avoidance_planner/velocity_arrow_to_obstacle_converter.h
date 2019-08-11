#ifndef __VELOCITY_ARROW_TO_OBSTACLE_CONVERTER
#define __VELOCITY_ARROW_TO_OBSTACLE_CONVERTER

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/MarkerArray.h>

class VelocityArrowToObstacleConverter
{
public:
    VelocityArrowToObstacleConverter(void);
    void waypoint_callback(const geometry_msgs::PoseArrayConstPtr&);
    void velocity_arrow_callback(const visualization_msgs::MarkerArrayConstPtr&);
    void process(void);

private:
    double PREDICTION_TIME;
    int PREDICTION_STEP;
    std::string WORLD_FRAME;
    std::string OBS_FRAME;
    double HZ;
    double INTERVAL;

    ros::NodeHandle nh;
    ros::NodeHandle local_nh;
    ros::Publisher obs_num_pub;
    ros::Publisher predicted_paths_pub;
    ros::Subscriber velocity_arrow_sub;
    geometry_msgs::PoseArray predicted_paths;
    bool velocity_arrow_updated;
    tf::TransformBroadcaster br;
    tf::StampedTransform _transform;
    visualization_msgs::MarkerArray velocity_arrows;
    geometry_msgs::TransformStamped transform;
    std::vector<geometry_msgs::Twist> velocities;
    geometry_msgs::PoseArray obs_poses;
    std_msgs::Int32 obs_num;
};

#endif// __VELOCITY_ARROW_TO_OBSTACLE_CONVERTER
