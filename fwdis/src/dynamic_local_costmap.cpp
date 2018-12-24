#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/Int32.h>

double PREDICTION_TIME = 3.5;// [s], 軌道予測時間
const double DT = 0.1;// [s]
int PREDICTION_STEP = PREDICTION_TIME / DT;
const double WIDTH = 10;// [m]
double RESOLUTION = 0.10;// [m]
const double HZ = 10;
double RADIUS;// 衝突判定半径[m]
int obs_num = 1;//si
const int SEARCH_RANGE = 30;
const double COST_COL = 90;
const double MIN_COST = 10;
std::string WORLD_FRAME;
std::string OBS_FRAME;
std::string ROBOT_FRAME;

geometry_msgs::PoseArray robot_path;
geometry_msgs::PoseArray obstacle_paths;
nav_msgs::OccupancyGrid local_costmap;

void setup_map(void);
bool predict_intersection(geometry_msgs::Pose, geometry_msgs::Pose, geometry_msgs::Pose, geometry_msgs::Pose);
void predict_intersection_point(geometry_msgs::Pose, geometry_msgs::Pose, geometry_msgs::Pose, geometry_msgs::Pose, geometry_msgs::PoseStamped&);
bool predict_approaching(geometry_msgs::Pose, geometry_msgs::Pose);
bool predict_approaching(geometry_msgs::Pose&, geometry_msgs::Pose&, geometry_msgs::Pose&);
void set_cost_with_velocity(geometry_msgs::PoseStamped&, geometry_msgs::Twist&, geometry_msgs::Twist&);

// map function
inline int get_i_from_x(double);
inline int get_j_from_y(double);
inline int get_index(double, double);
inline int get_distance_grid(int, int, int, int);

void robot_path_callback(const geometry_msgs::PoseArrayConstPtr& msg)
{
  robot_path = *msg;
}

void obstacle_paths_callback(const geometry_msgs::PoseArrayConstPtr& msg)
{
  obstacle_paths = *msg;
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

  local_nh.getParam("/dynamic_avoidance/PREDICTION_TIME", PREDICTION_TIME);
  local_nh.getParam("/dynamic_avoidance/RADIUS", RADIUS);
  local_nh.getParam("/dynamic_avoidance/RESOLUTION", RESOLUTION);
  local_nh.getParam("/dynamic_avoidance/ROBOT_FRAME", ROBOT_FRAME);
  local_nh.getParam("/dynamic_avoidance/OBSTACLES_FRAME", OBS_FRAME);
  local_nh.getParam("/dynamic_avoidance/WORLD_FRAME", WORLD_FRAME);
  PREDICTION_STEP = PREDICTION_TIME / DT;

  ros::Publisher costmap_pub = nh.advertise<nav_msgs::OccupancyGrid>("/local_costmap", 100);
  ros::Subscriber robot_predicted_path_sub = nh.subscribe("/robot_predicted_path", 100, robot_path_callback);
  ros::Subscriber obstacle_predicted_paths_sub = nh.subscribe("/predicted_paths", 100, obstacle_paths_callback);
  ros::Subscriber obs_num_sub = nh.subscribe("/obs_num", 100, obs_num_callback);

  tf::TransformBroadcaster broadcaster;
  geometry_msgs::TransformStamped base_link_to_local_costmap;

  tf::TransformListener listener;

  ros::Rate loop_rate(HZ);

  while(ros::ok()){
    if(!robot_path.poses.empty() && !obstacle_paths.poses.empty()){
      std::cout << "=== dynamic local costmap ===" << std::endl;
      ros::Time start_time = ros::Time::now();
      tf::StampedTransform transform;
      bool transformed = false;
      try{
        listener.lookupTransform(WORLD_FRAME, "local_costmap", ros::Time(0), transform);
        // pathの座標系変換
        for(int i=0;i<robot_path.poses.size();i++){
          geometry_msgs::PoseStamped temp;
          temp.header.frame_id = WORLD_FRAME;
          temp.pose = robot_path.poses[i];
          listener.transformPose("local_costmap", temp, temp);
          robot_path.poses[i] = temp.pose;
          robot_path.header = temp.header;
        }
        for(int i=0;i<obstacle_paths.poses.size();i++){
          geometry_msgs::PoseStamped temp;
          temp.header.frame_id = WORLD_FRAME;
          temp.pose = obstacle_paths.poses[i];
          listener.transformPose("local_costmap", temp, temp);
          obstacle_paths.poses[i] = temp.pose;
          obstacle_paths.header = temp.header;
        }
        transformed = true;
      }catch(tf::TransformException ex){
        std::cout << ex.what() << std::endl;;
      }
      std::cout << "transformed" << std::endl;

      if(transformed){
        // costmap初期化
        setup_map();
        std::cout << "===calculate cost===" << std::endl;
        // 接近
        double set_cost_v_start = ros::Time::now().toSec();
        for(int j=0;j<obs_num;j++){
          for(int k=1;k<3;k++){
            for(int i=0;i<PREDICTION_STEP;i++){
              if(predict_approaching(robot_path.poses[i], robot_path.poses[i+k*(PREDICTION_STEP+1)], obstacle_paths.poses[j*(PREDICTION_STEP+1)+i])){
                geometry_msgs::PoseStamped collision_pose;
                collision_pose.pose = obstacle_paths.poses[j*(PREDICTION_STEP+1)+i];
                collision_pose.header.frame_id = WORLD_FRAME;
                if(i > 0){
                  geometry_msgs::Twist vr;
                  vr.linear.x = (robot_path.poses[i].position.x - robot_path.poses[i - 1].position.x) * HZ;
                  vr.linear.y = (robot_path.poses[i].position.y - robot_path.poses[i - 1].position.y) * HZ;
                  geometry_msgs::Twist vo;
                  vo.linear.x = (obstacle_paths.poses[j*(PREDICTION_STEP+1)+i].position.x - obstacle_paths.poses[j*(PREDICTION_STEP+1)+i - 1].position.x) * HZ;
                  vo.linear.y = (obstacle_paths.poses[j*(PREDICTION_STEP+1)+i].position.y - obstacle_paths.poses[j*(PREDICTION_STEP+1)+i - 1].position.y) * HZ;
                  set_cost_with_velocity(collision_pose, vr, vo);
                  // 衝突予測以後の未来は考えない
                  break;
                }
              }
            }
          }
        }
        std::cout << "set_cost_v : " << ros::Time::now().toSec() - set_cost_v_start << "[s]" << std::endl;
        std::cout << "===publish costmap===" << std::endl;
        costmap_pub.publish(local_costmap);
        std::cout << ros::Time::now() - start_time << "[s]" << std::endl;
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
  return tc * td < 0 && ta * tb < 0;
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

void set_cost_with_velocity(geometry_msgs::PoseStamped& collision_pose, geometry_msgs::Twist& vr, geometry_msgs::Twist& vo)
{
  /*
   * 衝突位置
   * cpでのロボット速度ベクトル
   * cpでの障害物速度ベクトル
   */
  geometry_msgs::Twist synthetic_vector;
  synthetic_vector.linear.x = (vo.linear.x - vr.linear.x) * 0.5;
  synthetic_vector.linear.y = (vo.linear.y - vr.linear.y) * 0.5;
  double x = collision_pose.pose.position.x;
  double y = collision_pose.pose.position.y;
  double radius_min = 0.05;// 最小コスト半径
  double radius_max = 1.5 * RADIUS;// 最大回避領域コスト半径
  double radius_col_max = 1.0 * RADIUS;// 最大衝突領域半径
  double radius_col_min = radius_min;// 最小衝突領域半径
  double v = sqrt(synthetic_vector.linear.x * synthetic_vector.linear.x + synthetic_vector.linear.y * synthetic_vector.linear.y);
  const double LENGTH = v * PREDICTION_TIME;
  double l = 0;
  for(int s=0;s<PREDICTION_STEP;s+=1){// PREDICTION_STEPである理由はない
    int grid_x = get_i_from_x(x);
    int grid_y = get_j_from_y(y);
    int lower_i = grid_x - SEARCH_RANGE;
    if(lower_i < 0){
      lower_i = 0;
    }
    int upper_i = grid_x + SEARCH_RANGE;
    if(upper_i > local_costmap.info.width-1){
      upper_i = local_costmap.info.width-1;
    }
    int lower_j = grid_y - SEARCH_RANGE;
    if(lower_j < 0){
      lower_j = 0;
    }
    int upper_j = grid_y + SEARCH_RANGE;
    if(upper_j > local_costmap.info.width-1){
      upper_j = local_costmap.info.width-1;
    }
    // l[m]での衝突領域半径
    double radius_col_l = radius_col_min + (radius_col_max - radius_col_min) * (LENGTH - l) / LENGTH;
    double radius_col_l_grid = radius_col_l / RESOLUTION;
    double cost_l_col = COST_COL * (LENGTH - l) / LENGTH;
    // l[m]での回避領域半径
    double radius_l = radius_min + (radius_max - radius_min) * (LENGTH - l) / LENGTH;
    double radius_l_grid = radius_l / RESOLUTION;
    for(int i=lower_i;i<upper_i;i++){
      for(int j=lower_j;j<upper_j;j++){
        int d_ix = grid_x - i;
        if(d_ix < 0){
          d_ix = -d_ix;
        }
        int d_jy = grid_y - j;
        if(d_jy < 0){
          d_jy = -d_jy;
        }
        double dist = sqrt(d_ix * d_ix + d_jy * d_jy);
        if(dist <= radius_col_l_grid){
          if(local_costmap.data[local_costmap.info.width * j + i] < cost_l_col + MIN_COST){
            local_costmap.data[local_costmap.info.width * j + i] = cost_l_col + MIN_COST;
          }
        }else if(dist <= radius_l_grid){
          double cost_l = cost_l_col - cost_l_col * (dist*RESOLUTION - radius_col_l) / (radius_l - radius_col_l);
          if(local_costmap.data[local_costmap.info.width * j + i] < cost_l + MIN_COST){
            local_costmap.data[local_costmap.info.width * j + i] = cost_l + MIN_COST;
          }
        }
      }
    }
    x += synthetic_vector.linear.x * DT;
    y += synthetic_vector.linear.y * DT;
    l += v * DT;
  }
}
//参考: https://qiita.com/yellow_73/items/bcd4e150e7caa0210ee6
bool predict_approaching(geometry_msgs::Pose& pr1, geometry_msgs::Pose& pr2, geometry_msgs::Pose& po)
{
  double a = pr2.position.x - pr1.position.x;
  double b = pr2.position.y - pr1.position.y;
  double a2 = a * a;
  double b2 = b * b;
  double r2 = a2 + b2;
  double tt = -(a * (pr1.position.x - po.position.x) + b * (pr1.position.y - po.position.y));
  double distance = 0;
  if(tt < 0){
    distance = (pr1.position.x - po.position.x) * (pr1.position.x - po.position.x) + (pr1.position.y - po.position.y) * (pr1.position.y - po.position.y);
  }else if(tt > r2){
    distance = (pr2.position.x - po.position.x) * (pr2.position.x - po.position.x) + (pr2.position.y - po.position.y) * (pr2.position.y - po.position.y);
  }else{
    double f1 = a * (pr1.position.y - po.position.y) - b * (pr1.position.x - po.position.x);
    distance = (f1 * f1) / r2;
  }
  if(distance < RADIUS){
    return true;
  }else{
    return false;
  }
}

int get_distance_grid(int i0, int j0, int i1, int j1)
{
  int di = i0 - i1;
  if(di < 0){
    di = -di;
  }
  int dj = j0 - j1;
  if(dj < 0){
    dj = -dj;
  }
  if(di >= dj){
    return di;
  }else{
    return dj;
  }
}
