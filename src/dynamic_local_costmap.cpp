#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/Int32.h>

const double PREDICTION_TIME = 3.5;// [s], 軌道予測時間
const double DT = 0.1;// [s]
const int PREDICTION_STEP = PREDICTION_TIME / DT;
const double WIDTH = 10;// [m]
const double RESOLUTION = 0.05;// [m]
double RADIUS;
int obs_num;

geometry_msgs::PoseArray robot_path;
geometry_msgs::PoseArray obstacle_pathes;
nav_msgs::OccupancyGrid local_costmap;

void setup_map(void);
bool predict_intersection(geometry_msgs::Pose, geometry_msgs::Pose, geometry_msgs::Pose, geometry_msgs::Pose);
void predict_intersection_point(geometry_msgs::Pose, geometry_msgs::Pose, geometry_msgs::Pose, geometry_msgs::Pose, geometry_msgs::PoseStamped&);
bool predict_approaching(geometry_msgs::Pose, geometry_msgs::Pose);
void set_cost(geometry_msgs::PoseStamped, double, int);

// map function
int get_i_from_x(double);
int get_j_from_y(double);
int get_index(double, double);

void robot_path_callback(const geometry_msgs::PoseArrayConstPtr& msg)
{
  robot_path = *msg;
}

void obstacle_pathes_callback(const geometry_msgs::PoseArrayConstPtr& msg)
{
  obstacle_pathes = *msg;
}

void obs_num_callback(const std_msgs::Int32ConstPtr& msg)
{
  obs_num = msg->data;
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
  ros::Subscriber obs_num_sub = nh.subscribe("/obs_num", 100, obs_num_callback);

  tf::TransformListener listener;

  ros::Rate loop_rate(10);

  while(ros::ok()){
    if(!robot_path.poses.empty() && !obstacle_pathes.poses.empty()){
      tf::StampedTransform transform;
      bool transformed = false;
      try{
        listener.lookupTransform("map", "local_costmap", ros::Time(0), transform);
        transformed = true;
      }catch(tf::TransformException ex){
        std::cout << ex.what();
      }
      if(transformed){
        // costmap初期化
        setup_map();
        std::cout << "===calculate cost===" << std::endl;
        // 交差
        for(int i=0;i<PREDICTION_STEP;i++){
          for(int j=0;j<obs_num;j++){
            // 衝突判定(左)
            if(predict_intersection(robot_path.poses[i], robot_path.poses[i+(PREDICTION_STEP+1)], obstacle_pathes.poses[j*(PREDICTION_STEP+1)+i], obstacle_pathes.poses[j*(PREDICTION_STEP+1)+i+obs_num*(PREDICTION_STEP+1)])){
              std::cout << "cross collision at robot left:" << i << std::endl;;
              geometry_msgs::PoseStamped collision_pose;
              collision_pose.pose.orientation = robot_path.poses[i].orientation;
              collision_pose.header.frame_id = "map";
              // 衝突位置の計算(左)
              predict_intersection_point(robot_path.poses[i], robot_path.poses[i+(PREDICTION_STEP+1)], obstacle_pathes.poses[j*(PREDICTION_STEP+1)+i], obstacle_pathes.poses[j*(PREDICTION_STEP+1)+i+obs_num*(PREDICTION_STEP+1)], collision_pose);
              geometry_msgs::PoseStamped _collision_pose;
              listener.transformPose("local_costmap", collision_pose, _collision_pose);
              set_cost(_collision_pose, RADIUS, i);
            }
            // 衝突判定(右)
            if(predict_intersection(robot_path.poses[i], robot_path.poses[i+(PREDICTION_STEP+1)*2], obstacle_pathes.poses[j*(PREDICTION_STEP+1)*2+i], obstacle_pathes.poses[j*(PREDICTION_STEP+1)+i+obs_num*(PREDICTION_STEP+1)])){
              std::cout << "cross collision at robot right:" << i << std::endl;;
              geometry_msgs::PoseStamped collision_pose;
              collision_pose.pose.orientation = robot_path.poses[i].orientation;
              collision_pose.header.frame_id = "map";
              // 衝突位置の計算(右)
              predict_intersection_point(robot_path.poses[i], robot_path.poses[i+(PREDICTION_STEP+1)*2], obstacle_pathes.poses[j*(PREDICTION_STEP+1)+i], obstacle_pathes.poses[j*(PREDICTION_STEP+1)+i+obs_num*(PREDICTION_STEP+1)], collision_pose);
              geometry_msgs::PoseStamped _collision_pose;
              listener.transformPose("local_costmap", collision_pose, _collision_pose);
              set_cost(_collision_pose, RADIUS, i);
            }
          }
        }
        // 接近
        for(int i=0;i<PREDICTION_STEP;i++){
          for(int j=0;j<obs_num;j++){
            // ロボット予測進路
            for(int k=0;k<3;k++){
              // 障害物予測進路
              for(int l=0;l<2;l++){
                if(predict_approaching(robot_path.poses[i+k*(PREDICTION_STEP+1)], obstacle_pathes.poses[j*(PREDICTION_STEP+1)+i+obs_num*(PREDICTION_STEP+1)*l])){
                  //std::cout << "approaching collision step:" << i << std::endl;
                  geometry_msgs::PoseStamped collision_pose;
                  collision_pose.pose = robot_path.poses[i+k*(PREDICTION_STEP+1)];
                  collision_pose.header.frame_id = "map";
                  geometry_msgs::PoseStamped _collision_pose;
                  listener.transformPose("local_costmap", collision_pose, _collision_pose);
                  set_cost(_collision_pose, RADIUS, i);
                }
              }
            }
          }
        }
        std::cout << "===publish costmap===" << std::endl;
        costmap_pub.publish(local_costmap);
      }else{
          std::cout << "path not received" << std::endl;
      }
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}

void setup_map(void)
{
  local_costmap.data.clear();
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
void predict_intersection_point(geometry_msgs::Pose a, geometry_msgs::Pose b, geometry_msgs::Pose c, geometry_msgs::Pose d, geometry_msgs::PoseStamped& result)
{
  double denominator = (b.position.x - a.position.x) * (d.position.y - c.position.y) - (b.position.y - a.position.y) * (d.position.x - c.position.x);
  double r = ((d.position.y - c.position.y) * (c.position.x - a.position.x) - (d.position.x - c.position.x) * (c.position.y - a.position.y)) / denominator;
  //double s = ((b.position.y - a.position.y) * (c.position.x - a.position.x) - (b.position.x - a.position.x) * (c.position.y - a.position.y)) / denominator;
  result.pose.position.x = a.position.x + r * (b.position.x - a.position.x);
  result.pose.position.y = a.position.y + r * (b.position.y - a.position.y);
}

bool predict_approaching(geometry_msgs::Pose p1, geometry_msgs::Pose p2)
{
  return RADIUS * RADIUS > (p1.position.x - p2.position.x) * (p1.position.x - p2.position.x) + (p1.position.y - p2.position.y) * (p1.position.y - p2.position.y);
}

void set_cost(geometry_msgs::PoseStamped collision_pose, double radius, int step)
{
  //std::cout << "cost: " << PREDICTION_STEP - step + 1 << std::endl;
  double x = collision_pose.pose.position.x;
  double y = collision_pose.pose.position.y;
  for(int i=0;i<local_costmap.info.height;i++){
    for(int j=0;j<local_costmap.info.width;j++){
      double _x = i * local_costmap.info.resolution + local_costmap.info.origin.position.x;
      double _y = j * local_costmap.info.resolution + local_costmap.info.origin.position.y;
      if((x-_x)*(x-_x)+(y-_y)*(y-_y) < radius*radius){
        // 適当
        double cost = PREDICTION_STEP - step + 1;
        if(local_costmap.data[local_costmap.info.width * j + i] < cost){
          //std::cout << i << ", " << j << std::endl;
          local_costmap.data[local_costmap.info.width * j + i] = cost;
        }
      }
    }
  }
}

int get_i_from_x(double x)
{
  return floor((x - local_costmap.info.origin.position.x) / local_costmap.info.resolution + 0.5);
}

int get_j_from_y(double y)
{
  return floor((y - local_costmap.info.origin.position.y) / local_costmap.info.resolution + 0.5);
}

int get_index(double x, double y)
{
  return local_costmap.info.width * get_j_from_y(y) + get_i_from_x(x);

}
