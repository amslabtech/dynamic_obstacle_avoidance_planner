#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>

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

geometry_msgs::PoseArray waypoints;
geometry_msgs::PoseStamped waypoint0;// from
geometry_msgs::PoseStamped waypoint1;// to
nav_msgs::OccupancyGrid local_costmap;
nav_msgs::Path path;
nav_msgs::Path path2;// 比較用2本目
nav_msgs::Path previous_path;
// for a*
std::vector<Cell> cells;
std::vector<int> open_list;
std::vector<int> close_list;

inline int get_i_from_x(double);
inline int get_j_from_y(double);
inline int get_index(double, double);
inline int get_heuristic(int, int);
double get_difference(nav_msgs::Path&, nav_msgs::Path&);
bool is_contained(std::vector<int>&, int);

double MARGIN_WALL;
std::string INTERMEDIATE_PATH_TOPIC_NAME;

const double HZ = 10;

bool waypoints_received = false;
bool map_received = false;

double calculate_astar(geometry_msgs::PoseStamped&, geometry_msgs::PoseStamped&, nav_msgs::Path&);
void intersection_on_map(geometry_msgs::PoseStamped&, geometry_msgs::PoseStamped&);
// from dynamic_local_costmap.cpp
bool predict_intersection(geometry_msgs::Pose, geometry_msgs::Pose, geometry_msgs::Pose, geometry_msgs::Pose);
// from dynamic_local_costmap.cpp
void predict_intersection_point(geometry_msgs::Pose, geometry_msgs::Pose, geometry_msgs::Pose, geometry_msgs::Pose, geometry_msgs::PoseStamped&);
int get_distance_to_global_path(int, int);

void map_callback(const nav_msgs::OccupancyGridConstPtr& msg)
{
  //std::cout << "===map callback===" << std::endl;
  //ros::Time start_time = ros::Time::now();
  local_costmap = *msg;
  //std::cout << local_costmap.data.size() << std::endl;

  //std::vector<int> wall_list;

  cells.clear();
  cells.resize(local_costmap.info.height*local_costmap.info.width);
  for(int i=0;i<local_costmap.info.height*local_costmap.info.width;i++){
    //cells[i].is_wall = (local_costmap.data[i]==100);
    cells[i].sum = -1;
    cells[i].parent_index = -1;
    cells[i].cost = local_costmap.data[i];
    if(cells[i].is_wall){
      cells[i].cost = 254;
    }
  }
  //std::cout << ros::Time::now() - start_time << "[s]" << std::endl;
  //std::cout << "map callback end" << std::endl;
  map_received = true;
}

void waypoints_callback(const geometry_msgs::PoseArrayConstPtr& msg)
{
  waypoints = *msg;
  if(waypoints.poses.size()==2){
    waypoints_received = true;
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "avoidance_path_planner");
  ros::NodeHandle nh;
  ros::NodeHandle local_nh("~");

  local_nh.getParam("MARGIN_WALL", MARGIN_WALL);
  local_nh.getParam("/dynamic_avoidance/INTERMEDIATE_PATH_TOPIC_NAME", INTERMEDIATE_PATH_TOPIC_NAME);

  ros::Publisher path_pub = nh.advertise<nav_msgs::Path>(INTERMEDIATE_PATH_TOPIC_NAME, 100);

  // path2用
  ros::Publisher path2_pub = nh.advertise<nav_msgs::Path>(INTERMEDIATE_PATH_TOPIC_NAME + std::to_string(2), 100);

  ros::Subscriber map_sub = nh.subscribe("/local_costmap", 1, map_callback);
  ros::Subscriber waypoints_sub = nh.subscribe("/waypoints", 1, waypoints_callback);

  tf::TransformListener listener;

  bool first_flag = true;

  open_list.reserve(40000);
  close_list.reserve(40000);

  ros::Rate loop_rate(HZ);

  while(ros::ok()){
    std::cout << "=== avoidance path planner ===" << std::endl;
    if(map_received && waypoints_received){
      ros::Time start_time = ros::Time::now();

      //std::cout << "=== transform waypoints ===" << std::endl;
      geometry_msgs::PoseStamped _waypoint0;
      _waypoint0.header = waypoints.header;
      _waypoint0.pose = waypoints.poses[0];
      geometry_msgs::PoseStamped _waypoint1;
      _waypoint1.header = waypoints.header;
      _waypoint1.pose = waypoints.poses[1];
      bool transformed = false;
      try{
        listener.transformPose("local_costmap", _waypoint0, waypoint0);
        listener.transformPose("local_costmap", _waypoint1, waypoint1);
        transformed = true;
      }catch(tf::TransformException ex){
        std::cout << ex.what() << std::endl;
      }

      if(transformed){
        geometry_msgs::PoseStamped start;
        geometry_msgs::PoseStamped goal;
        intersection_on_map(start, goal);
        // 始点は原点
        start.pose.position.x = 0;
        start.pose.position.y = 0;
        start.pose.orientation = tf::createQuaternionMsgFromYaw(0);
        double cost1 = calculate_astar(start, goal, path);
        /*
        for(int i=1;i<path.poses.size()-20;i++){
          cells[get_index(path.poses[i].pose.position.x, path.poses[i].pose.position.y)].cost = 100;
        }
        double cost2 = calculate_astar(start, goal, path2);
        if(get_difference(previous_path, path) <= get_difference(previous_path, path2)){
          if(cost1 <= cost2){
            path_pub.publish(path);
            previous_path = path;
          }else{
            path_pub.publish(path2);
            previous_path = path2;
          }
        }else{
          if(cost2 <= cost1){
            path_pub.publish(path2);
            previous_path = path2;
          }else{
            path_pub.publish(path);
            previous_path = path;
          }
        }
        */
        path_pub.publish(path);
        previous_path = path;
        map_received = false;
        std::cout << ros::Time::now() - start_time << "[s]" << std::endl;
      }
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}

inline int get_i_from_x(double x)
{
  int val = (x - local_costmap.info.origin.position.x) * (1.0 / local_costmap.info.resolution) + 0.5;
  int i = (int)val;
  return i - (i > val);
}

inline int get_j_from_y(double y)
{
  int val = (y - local_costmap.info.origin.position.y) * (1.0 / local_costmap.info.resolution) + 0.5;
  int i = (int)val;
  return i - (i > val);
}

inline int get_index(double x, double y)
{
  return local_costmap.info.width * get_j_from_y(y) + get_i_from_x(x);
}

inline int get_heuristic(int diff_x, int diff_y)
{
  /*
  if(diff_x < 0){
    diff_x *= -1;
  }
  if(diff_y < 0){
    diff_y *= -1;
  }
  if(diff_x >= diff_y){
    return diff_x;
  }else{
    return diff_y;
  }
  */
  return sqrt(diff_x*diff_x+diff_y*diff_y);
}

double calculate_astar(geometry_msgs::PoseStamped& _start, geometry_msgs::PoseStamped& _goal, nav_msgs::Path& _path)
{
  open_list.clear();
  close_list.clear();
  for(int i=0;i<local_costmap.info.height*local_costmap.info.width;i++){
    cells[i].sum = -1;
    cells[i].parent_index = -1;
    cells[i].step = 0;
    if(cells[i].is_wall){
      cells[i].cost = 100;
    }
  }

  int start_index = get_index(_start.pose.position.x, _start.pose.position.y);
  int start_i = get_i_from_x(_start.pose.position.x);
  int start_j = get_j_from_y(_start.pose.position.y);
  int goal_index = get_index(_goal.pose.position.x, _goal.pose.position.y);
  int goal_i = get_i_from_x(_goal.pose.position.x);
  int goal_j = get_j_from_y(_goal.pose.position.y);
  //std::cout << "calculating path" << std::endl;
  /*
  std::cout << "from " << _start.pose.position.x << ", " << _start.pose.position.y << ", " << tf::getYaw(_start.pose.orientation) << ", " << start_index << std::endl;
  std::cout << start_i << ", " << start_j << std::endl;
  std::cout << "to " << _goal.pose.position.x << ", " << _goal.pose.position.y << ", " << tf::getYaw(_goal.pose.orientation) << ", " << goal_index << std::endl;
  std::cout << goal_i << ", " << goal_j << std::endl;
  */
  open_list.push_back(start_index);
  cells[open_list[0]].sum = cells[open_list[0]].step + get_heuristic(start_i - goal_i, start_j - goal_j) + get_distance_to_global_path(start_i, start_j);

  int count = 0;

  int max_openlist_size = 0;
  double max_loop_time = 0;

  while(!open_list.empty() && ros::ok()){
    double loop_start_time = ros::Time::now().toSec();

    count++;
    if(count > 10000){
      std::cout << "count > 10000" << std::endl;
      return 1000000;
    }
    int n_index = open_list[0];
    int n = cells[n_index].sum;//cells[n_index].step + get_heuristic(goal_i - _i, goal_j - _j);
    for(int i=0;i<open_list.size();i++){//openlist中で最小の要素を選択
      if(cells[open_list[i]].sum < n){
        n_index = open_list[i];
        n = cells[n_index].sum;
      }
    }
    //std::cout << "openlist:" << open_list.size() << std::endl;
    //std::cout << "goal:" << goal_i << ", " << goal_j << std::endl;
    if(n_index != goal_index){
      close_list.push_back(n_index);//選んだものがゴールでなければcloselistへ
      open_list.erase(std::remove(open_list.begin(), open_list.end(), n_index), open_list.end());//openlistから削除
    }else{
      break;
    }

    int _index;
    int _i = n_index % local_costmap.info.width;
    int _j = (n_index - _i) / local_costmap.info.width;
    //std::cout << "current:" << _i << ", " << _j << std::endl;
    //std::cout << "sum:" << cells[n_index].sum << std::endl;
    if(_j-1>=0){
      _index = (_j-1)*local_costmap.info.width+_i;//i, j-1
      if(!is_contained(open_list, _index) && !is_contained(close_list, _index)){
        if(!cells[_index].is_wall){
          cells[_index].step = cells[n_index].step + 1;
          cells[_index].sum = cells[_index].cost + cells[_index].step + get_heuristic(goal_i-_i, goal_j-(_j-1)) + get_distance_to_global_path(_i, _j-1);
          cells[_index].parent_index = n_index;
          open_list.push_back(_index);
        }
      }else if(is_contained(open_list, _index)){
        if(cells[n_index].step + 1 < cells[_index].step){
          cells[_index].parent_index = n_index;
        }
      }else if(is_contained(close_list, _index)){
        if(cells[n_index].step + 1 < cells[_index].step){
          cells[_index].parent_index = n_index;
        }
      }
    }
    if(_j+1<local_costmap.info.width){
      _index = (_j+1)*local_costmap.info.width+_i;//i, j+1
      if(!is_contained(open_list, _index) && !is_contained(close_list, _index)){
        if(!cells[_index].is_wall){
          cells[_index].step = cells[n_index].step + 1;
          cells[_index].sum = cells[_index].cost + cells[_index].step + get_heuristic(goal_i-_i, goal_j-(_j+1)) + get_distance_to_global_path(_i, _j+1);;
          cells[_index].parent_index = n_index;
          open_list.push_back(_index);
        }
      }else if(is_contained(open_list, _index)){
        if(cells[n_index].step + 1 < cells[_index].step){
          cells[_index].parent_index = n_index;
        }
      }else if(is_contained(close_list, _index)){
        if(cells[n_index].step + 1 < cells[_index].step){
          cells[_index].parent_index = n_index;
        }
      }

    }
    if(_i+1<local_costmap.info.height){
      _index = _j*local_costmap.info.width+(_i+1);//i+1, j
      if(!is_contained(open_list, _index) && !is_contained(close_list, _index)){
        if(!cells[_index].is_wall){
          cells[_index].step = cells[n_index].step + 1;
          cells[_index].sum = cells[_index].cost + cells[_index].step + get_heuristic(goal_i-(_i+1), goal_j-_j) + get_distance_to_global_path(_i+1, _j);
          cells[_index].parent_index = n_index;
          open_list.push_back(_index);
        }
      }else if(is_contained(open_list, _index)){
        if(cells[n_index].step + 1 < cells[_index].step){
          cells[_index].parent_index = n_index;
        }
      }else if(is_contained(close_list, _index)){
        if(cells[n_index].step + 1 < cells[_index].step){
          cells[_index].parent_index = n_index;
        }
      }

      if(_j-1>=0){
        _index = (_j-1)*local_costmap.info.width+(_i+1);//i+1, j-1
        if(!is_contained(open_list, _index) && !is_contained(close_list, _index)){
          if(!cells[_index].is_wall){
            cells[_index].step = cells[n_index].step + 1;
            cells[_index].sum = cells[_index].cost + cells[_index].step + get_heuristic(goal_i-(_i+1), goal_j-(_j-1)) + get_distance_to_global_path(_i+1, _j-1);
            cells[_index].parent_index = n_index;
            open_list.push_back(_index);
          }
        }else if(is_contained(open_list, _index)){
          if(cells[n_index].step + 1 < cells[_index].step){
            cells[_index].parent_index = n_index;
          }
        }else if(is_contained(close_list, _index)){
          if(cells[n_index].step + 1 < cells[_index].step){
            cells[_index].parent_index = n_index;
          }
        }

      }
      if(_j+1<local_costmap.info.width){
        _index = (_j+1)*local_costmap.info.width+(_i+1);//i+1, j+1
        if(!is_contained(open_list, _index) && !is_contained(close_list, _index)){
          if(!cells[_index].is_wall){
            cells[_index].step = cells[n_index].step + 1;
            cells[_index].sum = cells[_index].cost + cells[_index].step + get_heuristic(goal_i-(_i+1), goal_j-(_j+1)) + get_distance_to_global_path(_i+1, _j+1);
            cells[_index].parent_index = n_index;
            open_list.push_back(_index);
          }
        }else if(is_contained(open_list, _index)){
          if(cells[n_index].step + 1 < cells[_index].step){
            cells[_index].parent_index = n_index;
          }
        }else if(is_contained(close_list, _index)){
          if(cells[n_index].step + 1 < cells[_index].step){
            cells[_index].parent_index = n_index;
          }
        }

      }

    }
    if(_i-1>=0){
      _index = _j*local_costmap.info.width+(_i-1);//i-1, j
      if(!is_contained(open_list, _index) && !is_contained(close_list, _index)){
        if(!cells[_index].is_wall){
          cells[_index].step = cells[n_index].step + 1;
          cells[_index].sum = cells[_index].cost + cells[_index].step + get_heuristic(goal_i-(_i-1), goal_j-_j) + get_distance_to_global_path(_i-1, _j);
          cells[_index].parent_index = n_index;
          open_list.push_back(_index);
        }
      }else if(is_contained(open_list, _index)){
        if(cells[n_index].step + 1 < cells[_index].step){
          cells[_index].parent_index = n_index;
        }
      }else if(is_contained(close_list, _index)){
        if(cells[n_index].step + 1 < cells[_index].step){
          cells[_index].parent_index = n_index;
        }
      }

      if(_j-1>=0){
        _index = (_j-1)*local_costmap.info.width+(_i-1);//i-1, j-1
        if(!is_contained(open_list, _index) && !is_contained(close_list, _index)){
          if(!cells[_index].is_wall){
            cells[_index].step = cells[n_index].step + 1;
            cells[_index].sum = cells[_index].cost + cells[_index].step + get_heuristic(goal_i-(_i-1), goal_j-(_j-1)) + get_distance_to_global_path(_i-1, _j-1);
            cells[_index].parent_index = n_index;
            open_list.push_back(_index);
          }
        }else if(is_contained(open_list, _index)){
          if(cells[n_index].step + 1 < cells[_index].step){
            cells[_index].parent_index = n_index;
          }
        }else if(is_contained(close_list, _index)){
          if(cells[n_index].step + 1 < cells[_index].step){
            cells[_index].parent_index = n_index;
          }
        }

      }
      if(_j+1<local_costmap.info.width){
        _index = (_j+1)*local_costmap.info.width+(_i-1);//i-1, j+1
        if(!is_contained(open_list, _index) && !is_contained(close_list, _index)){
          if(!cells[_index].is_wall){
            cells[_index].step = cells[n_index].step + 1;
            cells[_index].sum = cells[_index].cost + cells[_index].step + get_heuristic(goal_i-(_i-1), goal_j-(_j+1)) + get_distance_to_global_path(_i-1, _j+1);
            cells[_index].parent_index = n_index;
            open_list.push_back(_index);
          }
        }else if(is_contained(open_list, _index)){
          if(cells[n_index].step + 1 < cells[_index].step){
            cells[_index].parent_index = n_index;
          }
        }else if(is_contained(close_list, _index)){
          if(cells[n_index].step + 1 < cells[_index].step){
            cells[_index].parent_index = n_index;
          }
        }
      }
      double loop_time = ros::Time::now().toSec() - loop_start_time;
      if(loop_time > max_loop_time){
        max_loop_time = loop_time;
        max_openlist_size = open_list.size();
      }
    }
  }
  std::cout << "max_loop_time = " << max_loop_time << std::endl;
  std::cout << "openlist = " << max_openlist_size << std::endl;
  std::cout << "count = " << count << std::endl;
  nav_msgs::Path temp_path;
  temp_path.header.frame_id = "local_costmap";
  int path_index = goal_index;
  geometry_msgs::PoseStamped path_pose;
  path_pose.pose.orientation.w = 1;
  path_pose.header.frame_id = "local_costmap";
  double total_cost = 0;
  while(1){
    total_cost += cells[path_index].sum;
    path_pose.pose.position.x = (path_index % local_costmap.info.width) * local_costmap.info.resolution + local_costmap.info.origin.position.x;
    path_pose.pose.position.y = (path_index - (path_index % local_costmap.info.width)) / local_costmap.info.width * local_costmap.info.resolution + local_costmap.info.origin.position.y;
    path_pose.pose.orientation = _goal.pose.orientation;
    //std::cout << path_pose.pose.position.x << ", " << path_pose.pose.position.y << ", " << path_index << ", " << cells[path_index].cost << ", " << (int)local_costmap.data[path_index] << std::endl;
    //std::cout << cells[path_index].cost << std::endl;
    temp_path.poses.push_back(path_pose);
    path_index = cells[path_index].parent_index;
    //std::cout << "next:" << path_index << std::endl;
    if(path_index < 0){
      std::reverse(temp_path.poses.begin(), temp_path.poses.end());
      _path = temp_path;
      /*
      std::cout << "=== path length ===" << std::endl;
      std::cout << _path.poses.size() << std::endl;
      std::cout << "=== path cost ===" << std::endl;
      std::cout << total_cost << std::endl;
      std::cout << "path generated!" << std::endl;
      */
      return total_cost;
    }
  }
}

void intersection_on_map(geometry_msgs::PoseStamped& out0, geometry_msgs::PoseStamped& out1)
{
  // costmapの四隅
  geometry_msgs::Pose pose_fr;
  pose_fr.position.x = local_costmap.info.origin.position.x + local_costmap.info.resolution * local_costmap.info.width;
  pose_fr.position.y = local_costmap.info.origin.position.y;
  geometry_msgs::Pose pose_fl;
  pose_fl.position.x = local_costmap.info.origin.position.x + local_costmap.info.resolution * local_costmap.info.width;
  pose_fl.position.y = local_costmap.info.origin.position.y + local_costmap.info.resolution * local_costmap.info.height;
  geometry_msgs::Pose pose_rr;
  pose_rr.position.x = local_costmap.info.origin.position.x;
  pose_rr.position.y = local_costmap.info.origin.position.y;
  geometry_msgs::Pose pose_rl;
  pose_rl.position.x = local_costmap.info.origin.position.x;
  pose_rl.position.y = local_costmap.info.origin.position.y + local_costmap.info.resolution * local_costmap.info.height;
  // 交差位置の計算
  std::vector<geometry_msgs::PoseStamped> out;
  out.resize(2);
  out[0].header.frame_id = "local_costmap";
  out[0].pose.orientation = tf::createQuaternionMsgFromYaw(0);
  out[1] = out[0];
  int i=0;
  if(predict_intersection(pose_rl, pose_rr, waypoint0.pose, waypoint1.pose)){
    predict_intersection_point(pose_rl, pose_rr, waypoint0.pose, waypoint1.pose, out[i]);
    i++;
  }
  if(predict_intersection(pose_fl, pose_fr, waypoint0.pose, waypoint1.pose)){
    predict_intersection_point(pose_fl, pose_fr, waypoint0.pose, waypoint1.pose, out[i]);
    i++;
  }
  if(i<2){
    if(predict_intersection(pose_fr, pose_rr, waypoint0.pose, waypoint1.pose)){
      predict_intersection_point(pose_fr, pose_rr, waypoint0.pose, waypoint1.pose, out[i]);
      i++;
    }
  }
  if(i<2){
    if(predict_intersection(pose_fl, pose_rl, waypoint0.pose, waypoint1.pose)){
      predict_intersection_point(pose_fl, pose_rl, waypoint0.pose, waypoint1.pose, out[i]);
      i++;
    }
  }
  if(i==2){
  // waypoint0と両交点までの距離
    double distance0 = (waypoint0.pose.position.x - out[0].pose.position.x)*(waypoint0.pose.position.x - out[0].pose.position.x) + (waypoint0.pose.position.y - out[0].pose.position.y)*(waypoint0.pose.position.y - out[0].pose.position.y);
    double distance1 = (waypoint0.pose.position.x - out[1].pose.position.x)*(waypoint0.pose.position.x - out[1].pose.position.x) + (waypoint0.pose.position.y - out[1].pose.position.y)*(waypoint0.pose.position.y - out[1].pose.position.y);
    if(distance0 > distance1){
      geometry_msgs::PoseStamped temp;
      temp = out[0];
      out[0] = out[1];
      out[1] = temp;
    }
    out0 = out[0];
    if(get_i_from_x(out0.pose.position.x) >= local_costmap.info.width){
      out0.pose.position.x = (local_costmap.info.width - 1) * local_costmap.info.resolution + local_costmap.info.origin.position.x;
    }else if(get_i_from_x(out0.pose.position.x) < 0){
      out0.pose.position.x = local_costmap.info.origin.position.x;
    }
    if(get_j_from_y(out0.pose.position.y) >= local_costmap.info.height){
      out0.pose.position.y = (local_costmap.info.height - 1) * local_costmap.info.resolution + local_costmap.info.origin.position.y;
    }else if(get_j_from_y(out0.pose.position.x) < 0){
      out0.pose.position.y = local_costmap.info.origin.position.y;
    }
    out1 = out[1];
    if(get_i_from_x(out1.pose.position.x) >= local_costmap.info.width){
      out1.pose.position.x = (local_costmap.info.width - 1) * local_costmap.info.resolution + local_costmap.info.origin.position.x;
    }else if(get_i_from_x(out1.pose.position.x) < 0){
      out1.pose.position.x = local_costmap.info.origin.position.x;
    }
    if(get_j_from_y(out1.pose.position.y) >= local_costmap.info.height){
      out1.pose.position.y = (local_costmap.info.height - 1) * local_costmap.info.resolution + local_costmap.info.origin.position.y;
    }else if(get_j_from_y(out1.pose.position.x) < 0){
      out1.pose.position.y = local_costmap.info.origin.position.y;
    }
  }else if(i==1){
    // どちらか一方のwaypointがcostmap内
    if((waypoint0.pose.position.x >= local_costmap.info.origin.position.x) && (waypoint0.pose.position.x <= local_costmap.info.origin.position.x + local_costmap.info.height * local_costmap.info.resolution) && (waypoint0.pose.position.y >= local_costmap.info.origin.position.y) && (waypoint0.pose.position.y <= local_costmap.info.origin.position.y + local_costmap.info.width * local_costmap.info.resolution)){
      // waypoint0がcostmap内の場合
      out1 = out[0];
      if(get_i_from_x(out1.pose.position.x) >= local_costmap.info.width){
        out1.pose.position.x = (local_costmap.info.width - 1) * local_costmap.info.resolution + local_costmap.info.origin.position.x;
      }else if(get_i_from_x(out1.pose.position.x) < 0){
        out1.pose.position.x = local_costmap.info.origin.position.x;
      }
      if(get_j_from_y(out1.pose.position.y) >= local_costmap.info.height){
        out1.pose.position.y = (local_costmap.info.height - 1) * local_costmap.info.resolution + local_costmap.info.origin.position.y;
      }else if(get_j_from_y(out1.pose.position.x) < 0){
        out1.pose.position.y = local_costmap.info.origin.position.y;
      }
    }else{
      // waypoint1がcostmap内の場合
      out1 = waypoint1;
    }

  }else{
    std::cout << "no intersection" << std::endl;
    out1 = waypoint1;
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

/*
 * 参考: https://qiita.com/yellow_73/items/bcd4e150e7caa0210ee6
 */
int get_distance_to_global_path(int i, int j)
{
  //return 0;
  int x1 = get_i_from_x(waypoint0.pose.position.x);
  int x2 = get_i_from_x(waypoint1.pose.position.x);
  int y1 = get_j_from_y(waypoint0.pose.position.y);
  int y2 = get_j_from_y(waypoint1.pose.position.y);
  int a = x2 - x1;
  int b = y2 - y1;
  int a2 = a * a;
  int b2 = b * b;
  int f1 = a * (y1 - j) - b * (x1 - i);
  return 0.05 * (f1 * f1) / (double)(a2 + b2);
}

double get_difference(nav_msgs::Path& _path1, nav_msgs::Path& _path2)
{
  double sum = 0;
  for(int i=0;(i<_path1.poses.size()) && (i<_path2.poses.size());i++){
    double dx = _path1.poses[i].pose.position.x - _path2.poses[i].pose.position.x;
    double dy = _path1.poses[i].pose.position.y - _path2.poses[i].pose.position.y;
    sum += dx * dx + dy * dy;
  }
  return sum;
}

bool is_contained(std::vector<int>& v, int val)
{
  for(int i=0;i<v.size();i++){
    if(v[i] == val){
      return true;
    }
  }
  return false;
}
