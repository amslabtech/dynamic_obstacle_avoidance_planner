#ifndef __LOCAL_COSTMAP_GENERATOR_H
#define __LOCAL_COSTMAP_GENERATOR_H

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class LocalCostmapGenerator
{
public:
    LocalCostmapGenerator(void);
    void cloud_callback(const sensor_msgs::PointCloud2ConstPtr&);
    void dynamic_callback(const nav_msgs::OccupancyGridConstPtr&);
    void process(void);
    inline int get_i_from_x(double);
    inline int get_j_from_y(double);
    inline int get_index(double, double);

private:
    double HZ;
    double RADIUS;
    double RADIUS_GRID;
    std::string ROBOT_FRAME;
    std::string WORLD_FRAME;

    ros::NodeHandle nh;
    ros::NodeHandle local_nh;
    ros::Publisher map_pub;
    ros::Subscriber cloud_sub;
    ros::Subscriber dynamic_sub;
    tf::TransformListener listener;
    tf::StampedTransform _transform;
    geometry_msgs::TransformStamped transform;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr;
    bool local_costmap_subscribed;
    bool cloud_updated;
    double map_max_limit_x;
    double map_min_limit_x;
    double map_max_limit_y;
    double map_min_limit_y;
    int max_index;
    nav_msgs::OccupancyGrid local_costmap;
};

#endif// __LOCAL_COSTMAP_GENERATOR_H
