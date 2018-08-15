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
double RADIUS;

geometry_msgs::PoseArray robot_path;
geometry_msgs::PoseArray obstacle_pathes;
nav_msgs::OccupancyGrid local_costmap;

void setup_map(void);
bool predict_intersection(geometry_msgs::Pose, geometry_msgs::Pose, geometry_msgs::Pose, geometry_msgs::Pose);
void predict_intersection_point(geometry_msgs::Pose, geometry_msgs::Pose, geometry_msgs::Pose, geometry_msgs::Pose, geometry_msgs::Pose&);
bool predict_approaching(geometry_msgs::Pose, geometry_msgs::Pose);

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

  local_nh.getParam("RADIUS", RADIUS);

  ros::Publisher costmap_pub = nh.advertise<nav_msgs::OccupancyGrid>("/local_costmap", 100);
  ros::Subscriber robot_predicted_path_sub = nh.subscribe("/robot_predicted_path", 100, robot_path_callback);
  ros::Subscriber obstacle_predicted_pathes_sub = nh.subscribe("/predicted_pathes", 100, obstacle_pathes_callback);

  setup_map();

  ros::Rate loop_rate(10);

  while(ros::ok()){
    if(!robot_path.poses.empty() && !obstacle_pathes.poses.empty()){

      costmap_pub.publish(local_costmap);
    }
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

/*
 * P1(a->b), P2(c->d)の衝突判定
 * https://qiita.com/ykob/items/ab7f30c43a0ed52d16f2
 */
bool predict_intersection(geometry_msgs::Pose a, geometry_msgs::Pose b, geometry_msgs::Pose c, geometry_msgs::Pose d)
{
  double ta = (c.position.x - d.position.x) * (a.position.y - c.position.y) + (c.position.y - d.position.y) * (c.position.x - a.position.x);
  double tb = (c.position.x - d.position.x) * (b.position.y - c.position.y) + (c.position.y - d.position.y) * (c.position.x - b.position.x);
  double tc = (a.position.x - b.position.x) * (c.position.y - a.position.y) + (a.position.y - b.position.y) * (a.position.x - c.position.x);
  double td = (a.position.x - b.position.x) * (d.position.y - a.position.y) + (a.position.y - b.position.y) * (a.position.x - d.position.x);
  return tc * td <= 0 && ta * tb <= 0;
}

/*
 * P1(a->b), P2(c->d)の衝突位置
 * http://www.hiramine.com/programming/graphics/2d_segmentintersection.html
 */
void predict_intersection_point(geometry_msgs::Pose a, geometry_msgs::Pose b, geometry_msgs::Pose c, geometry_msgs::Pose d, geometry_msgs::Pose& result)
{
  double denominator = (b.position.x - a.position.x) * (d.position.y - c.position.y) - (b.position.y - a.position.y) * (d.position.x - c.position.x);
  double r = ((d.position.y - c.position.y) * (c.position.x - a.position.x) - (d.position.x - c.position.x) * (c.position.y - a.position.y)) / denominator;
  //double s = ((b.position.y - a.position.y) * (c.position.x - a.position.x) - (b.position.x - a.position.x) * (c.position.y - a.position.y)) / denominator;
  result.position.x = a.position.x + r * (b.position.x - a.position.x);
  result.position.y = a.position.y + r * (b.position.y - a.position.y);
}

bool predict_approaching(geometry_msgs::Pose p1, geometry_msgs::Pose p2)
{
  return RADIUS < sqrt((p1.position.x - p2.position.x) * (p1.position.x - p2.position.x) + (p1.position.y - p2.position.y) * (p1.position.y - p2.position.y));
}
