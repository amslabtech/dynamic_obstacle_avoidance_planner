#ifndef __AVOIDANCE_PATH_PLANNER_H
#define __AVOIDANCE_PATH_PLANNER_H

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>


class AvoidancePathPlanner
{
public:
    AvoidancePathPlanner(void);

    void map_callback(const nav_msgs::OccupancyGridConstPtr&);
    void waypoints_callback(const geometry_msgs::PoseArrayConstPtr&);
    double get_difference(const nav_msgs::Path&, const nav_msgs::Path&);
    bool is_contained(const std::vector<int>&, int);
    double calculate_astar(const geometry_msgs::PoseStamped&, const geometry_msgs::PoseStamped&, nav_msgs::Path&);
    void intersection_on_map(geometry_msgs::PoseStamped&, geometry_msgs::PoseStamped&);
    // from dynamic_local_costmap.cpp
    bool predict_intersection(const geometry_msgs::Pose&, const geometry_msgs::Pose&, const geometry_msgs::Pose&, const geometry_msgs::Pose&);
    // from dynamic_local_costmap.cpp
    void predict_intersection_point(const geometry_msgs::Pose&, const geometry_msgs::Pose&, const geometry_msgs::Pose&, const geometry_msgs::Pose&, geometry_msgs::PoseStamped&);
    int get_distance_to_global_path(int, int);
    inline int get_i_from_x(double);
    inline int get_j_from_y(double);
    inline int get_index(double, double);
    inline int get_heuristic(int, int);
    void process(void);

    class Cell
    {
    public:
        Cell(void)
        {
            is_wall = false;
            cost = 0;
            step = 0;
            sum = -1;
            parent_index = -1;
        }

        int cost;
        int step;
        int sum;
        int parent_index;
        bool is_wall;
    };

private:
    double MARGIN_WALL;
    std::string INTERMEDIATE_PATH_TOPIC_NAME;
    double HZ = 10;

    ros::NodeHandle nh;
    ros::NodeHandle local_nh;
    ros::Publisher path_pub;
    // path2用
    ros::Publisher path2_pub;
    ros::Subscriber map_sub;
    ros::Subscriber waypoints_sub;
    geometry_msgs::PoseArray waypoints;
    geometry_msgs::PoseStamped waypoint0;// from
    geometry_msgs::PoseStamped waypoint1;// to
    nav_msgs::OccupancyGrid local_costmap;
    nav_msgs::Path path;
    nav_msgs::Path path2;// for comparing
    nav_msgs::Path previous_path;
    // for a*
    std::vector<Cell> cells;
    std::vector<int> open_list;
    std::vector<int> close_list;

    bool waypoints_received = false;
    bool map_received = false;

};
#endif// __AVOIDANCE_PATH_PLANNER_H
