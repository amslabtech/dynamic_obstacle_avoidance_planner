#ifndef __DYNAMIC_LOCAL_COSTMAP_GENERATOR_H
#define __DYNAMIC_LOCAL_COSTMAP_GENERATOR_H

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/Int32.h>

#include <Eigen/Dense>

#include "dynamic_obstacle_avoidance_planner/obstacle_tracker_kf.h"

class DynamicLocalCostmapGenerator
{
public:
    DynamicLocalCostmapGenerator(void);

    void process(void);
    void robot_path_callback(const geometry_msgs::PoseArrayConstPtr&);
    void obstacle_pose_callback(const geometry_msgs::PoseArrayConstPtr&);
    void setup_map(void);
    bool predict_intersection(geometry_msgs::Pose, geometry_msgs::Pose, geometry_msgs::Pose, geometry_msgs::Pose);
    void predict_intersection_point(geometry_msgs::Pose, geometry_msgs::Pose, geometry_msgs::Pose, geometry_msgs::Pose, geometry_msgs::PoseStamped&);
    bool predict_approaching(geometry_msgs::Pose, geometry_msgs::Pose);
    bool predict_approaching(geometry_msgs::Pose&, geometry_msgs::Pose&, geometry_msgs::Pose&);
    void set_cost_with_velocity(geometry_msgs::PoseStamped&, geometry_msgs::Twist&, geometry_msgs::Twist&);
    inline int get_i_from_x(double);
    inline int get_j_from_y(double);
    inline int get_index(double, double);
    inline int get_distance_grid(int, int, int, int);

private:
    double PREDICTION_TIME;// [s], trafjectory prediction time
    double HZ;
    double DT;// [s]
    int PREDICTION_STEP;
    double MAP_WIDTH;// [m]
    double RESOLUTION;// [m]
    double RADIUS;// radius for collision check[m]
    int SEARCH_RANGE;
    double COST_COLLISION;
    double MIN_COST;
    double MAX_COST;
    std::string WORLD_FRAME;
    std::string OBS_FRAME;
    std::string ROBOT_FRAME;

    ros::NodeHandle nh;
    ros::NodeHandle local_nh;
    ros::Publisher costmap_pub;
    ros::Subscriber robot_predicted_path_sub;
    ros::Subscriber obstacle_predicted_paths_sub;
    ros::Subscriber obs_num_sub;
    ros::Subscriber obstacle_pose_sub;
    geometry_msgs::PoseArray robot_path;
    geometry_msgs::PoseArray obstacle_paths;
    geometry_msgs::PoseArray obstacle_pose;
    nav_msgs::OccupancyGrid local_costmap;
    int obs_num;
    ObstacleTrackerKF tracker;
};

#endif// __DYNAMIC_LOCAL_COSTMAP_GENERATOR_H
