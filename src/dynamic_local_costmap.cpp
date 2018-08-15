#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/OccupancyGrid.h>

const double PREDICTION_TIME = 3.5;// [s], 軌道予測時間
const double DT = 0.1;// [s]
const int PREDICTION_STEP = PREDICTION_TIME / DT;
const double WIDTH = 10;// [m]
const double RESOLUTION = 0.05;// [m]

geometry_msgs::PoseArray robot_path;
geometry_msgs::PoseArray obstacle_pathes;
nav_msgs::OccupancyGrid local_costmap;

void setup_map(void);

void robot_path_callback(const geometry_msgs::PoseArrayConstPtr& msg)
{
  robot_path = *msg;
}

void obstacle_pathes_callback(const geometry_msgs::PoseArrayConstPtr& msg)
{
  obstacle_pathes = *msg;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "dynamic_local_costmap");
  ros::NodeHandle nh;
  ros::NodeHandle local_nh("~");

  ros::Publisher costmap_pub = nh.advertise<nav_msgs::OccupancyGrid>("/local_costmap", 100);
  ros::Subscriber robot_predicted_path_sub = nh.subscribe("/robot_predicted_path", 100, robot_path_callback);
  ros::Subscriber obstacle_predicted_pathes_sub = nh.subscribe("/predicted_pathes", 100, obstacle_pathes_callback);

  setup_map();

  ros::Rate loop_rate(10);

  while(ros::ok()){

    costmap_pub.publish(local_costmap);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}

void setup_map(void)
{
  local_costmap.header.frame_id = "local_costmap";
  local_costmap.info.resolution = RESOLUTION;
  local_costmap.info.width = WIDTH / RESOLUTION;
  local_costmap.info.height = WIDTH / RESOLUTION;
  local_costmap.info.origin.position.x = -WIDTH / 2.0;
  local_costmap.info.origin.position.y = -WIDTH / 2.0;
  local_costmap.info.origin.orientation = tf::createQuaternionMsgFromYaw(0.0);
  local_costmap.data.resize(local_costmap.info.width * local_costmap.info.height);
  for(int i=0;i<local_costmap.info.height;i++){
    for(int j=0;j<local_costmap.info.width;j++){
      local_costmap.data[i*local_costmap.info.width+j] = 0;
    }
  }
}
