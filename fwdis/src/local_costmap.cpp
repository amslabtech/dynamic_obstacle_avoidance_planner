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

const double INTERVAL = 0.05;

std::string ROBOT_FRAME;
std::string WORLD_FRAME;

nav_msgs::OccupancyGrid local_costmap;

class LocalCostmap
{
public:
  LocalCostmap(void);
  void cloud_callback(const sensor_msgs::PointCloud2ConstPtr&);
  void dynamic_callback(const nav_msgs::OccupancyGridConstPtr&);
  void process(void);

private:
  ros::NodeHandle nh;
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
};

// map function
inline int get_i_from_x(double);
inline int get_j_from_y(double);
inline int get_index(double, double);

int main(int argc, char** argv)
{
  ros::init(argc, argv, "local_costmap");
  ros::NodeHandle nh;
  ros::NodeHandle local_nh("~");
  local_nh.getParam("/dynamic_avoidance/ROBOT_FRAME", ROBOT_FRAME);
  local_nh.getParam("/dynamic_avoidance/WORLD_FRAME", WORLD_FRAME);

  LocalCostmap local_costmap_creator;
  local_costmap_creator.process();
}

LocalCostmap::LocalCostmap(void)
{
  map_pub = nh.advertise<nav_msgs::OccupancyGrid>("/local_costmap", 1);
  cloud_sub = nh.subscribe("/cluster/human/removed", 1, &LocalCostmap::cloud_callback, this);
  dynamic_sub = nh.subscribe("/dynamic_local_costmap", 1, &LocalCostmap::dynamic_callback, this);
  cloud_ptr = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  local_costmap_subscribed = false;
  cloud_updated = false;
}

void LocalCostmap::process(void)
{
  ros::Rate loop_rate(1.0 / INTERVAL);

  std::cout << "local_costmap" << std::endl;

  while(ros::ok()){
    if(local_costmap_subscribed){
      if(cloud_updated){
        pcl_ros::transformPointCloud(ROBOT_FRAME, *cloud_ptr, *cloud_ptr, listener);
        int size = cloud_ptr->points.size();
        for(int i=0;i<size;i++){
          if(cloud_ptr->points[i].x < map_max_limit_x && cloud_ptr->points[i].x > map_min_limit_x && cloud_ptr->points[i].y < map_max_limit_y && cloud_ptr->points[i].y > map_min_limit_y){
            local_costmap.data[get_index(cloud_ptr->points[i].x, cloud_ptr->points[i].y)] = 100;
          }
        }
        cloud_updated = false;
      }
      map_pub.publish(local_costmap);
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
}

void LocalCostmap::cloud_callback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
  sensor_msgs::PointCloud2 _cloud;
  _cloud = *msg;
  pcl::fromROSMsg(_cloud, *cloud_ptr);
  cloud_updated = true;
}

void LocalCostmap::dynamic_callback(const nav_msgs::OccupancyGridConstPtr& msg)
{
  local_costmap = *msg;
  map_max_limit_x = local_costmap.info.resolution * local_costmap.info.height - local_costmap.info.origin.position.x;
  map_min_limit_x = -local_costmap.info.resolution * local_costmap.info.height - local_costmap.info.origin.position.x;
  map_max_limit_y = local_costmap.info.resolution * local_costmap.info.width - local_costmap.info.origin.position.y;
  map_min_limit_y = -local_costmap.info.resolution * local_costmap.info.width - local_costmap.info.origin.position.y;
  local_costmap_subscribed = true;
}

inline int get_i_from_x(double x)
{
  return floor((x - local_costmap.info.origin.position.x) / local_costmap.info.resolution + 0.5);
}

inline int get_j_from_y(double y)
{
  return floor((y - local_costmap.info.origin.position.y) / local_costmap.info.resolution + 0.5);
}

inline int get_index(double x, double y)
{
  return local_costmap.info.width * get_j_from_y(y) + get_i_from_x(x);
}
