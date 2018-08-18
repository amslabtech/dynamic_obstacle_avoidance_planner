#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

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

nav_msgs::OccupancyGrid local_costmap;
nav_msgs::Path path;
// for a*
std::vector<Cell> cells;
std::vector<int> open_list;
std::vector<int> close_list;

int get_i_from_x(double);
int get_j_from_y(double);
int get_index(double, double);
int get_heuristic(int, int);

double MARGIN_WALL;

void calculate_astar(geometry_msgs::PoseStamped&, geometry_msgs::PoseStamped&);

void map_callback(const nav_msgs::OccupancyGridConstPtr& msg)
{
  std::cout << "===map callback===" << std::endl;
  local_costmap = *msg;
  std::cout << local_costmap.data.size() << std::endl;

  int MARGIN_WALL_STEP = 254 / (MARGIN_WALL / local_costmap.info.resolution);
  std::cout << "MARGIN_WALL_STEP:" << MARGIN_WALL_STEP << std::endl;

  std::vector<int> wall_list;

  cells.clear();
  cells.resize(local_costmap.info.height*local_costmap.info.width);
  for(int i=0;i<local_costmap.info.height*local_costmap.info.width;i++){
    cells[i].is_wall = (local_costmap.data[i]==255);
    cells[i].sum = -1;
    cells[i].parent_index = -1;
    if(cells[i].is_wall){
      wall_list.push_back(i);
      cells[i].cost = 254;
    }
  }
  std::cout << "wall:" <<  wall_list.size() << std::endl;
  int i=0;
  while(ros::ok()){
    if(i==wall_list.size()){
      break;
    }
    int cost = cells[wall_list[i]].cost;
    if(cost < MARGIN_WALL_STEP){
      std::cout << "end" << cost << std::endl;
      break;
    }
    int _i = wall_list[i] % local_costmap.info.width;
    int _j = (wall_list[i] - _i) / local_costmap.info.height;
    if(_i-1>0){
      int index = (_i-1) + (_j) * local_costmap.info.width;
      if(cells[index].cost < cost){
        cells[index].cost = cost - MARGIN_WALL_STEP;
        wall_list.push_back(index);
      }
    }
    if(_i+1<local_costmap.info.width){
      int index = (_i+1) + (_j) * local_costmap.info.width;
      if(cells[index].cost < cost){
        cells[index].cost = cost - MARGIN_WALL_STEP;
        wall_list.push_back(index);
      }
    }
    if(_j-1>0){
      int index = (_i) + (_j-1) * local_costmap.info.width;
      if(cells[index].cost < cost){
        cells[index].cost = cost - MARGIN_WALL_STEP;
        wall_list.push_back(index);
      }
    }
    if(_j+1<local_costmap.info.width){
      int index = (_i) + (_j+1) * local_costmap.info.width;
      if(cells[index].cost < cost){
        cells[index].cost = cost - MARGIN_WALL_STEP;
        wall_list.push_back(index);
      }
    }
    i++;
  }
  std::cout << "map callback end" << std::endl;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "avoidance_path_planner");
  ros::NodeHandle nh;
  ros::NodeHandle local_nh("~");

  local_nh.getParam("MARGIN_WALL", MARGIN_WALL);

  ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("/intermediate_path", 100);

  ros::Subscriber map_sub = nh.subscribe("/local_costmap", 100, map_callback);

  tf::TransformListener listener;

  ros::Rate loop_rate(10);

  while(ros::ok()){
    if(!local_costmap.data.empty()){
      std::cout << "=== avoidance path planner ===" << std::endl;
      ros::Time start_time = ros::Time::now();
      geometry_msgs::PoseStamped start;
      start.header.frame_id = "local_costmap";
      start.pose.position.x = 0;
      start.pose.position.y = 0;
      start.pose.orientation = tf::createQuaternionMsgFromYaw(0);
      geometry_msgs::PoseStamped goal;
      goal.header.frame_id = "local_costmap";
      goal.pose.position.x = 4;
      goal.pose.position.y = 0;
      goal.pose.orientation = tf::createQuaternionMsgFromYaw(0);
      calculate_astar(start, goal);
      path_pub.publish(path);
      std::cout << ros::Time::now() - start_time << "[s]" << std::endl;
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
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

int get_heuristic(int diff_x, int diff_y)
{
  return sqrt(diff_x*diff_x + diff_y*diff_y);
}

void calculate_astar(geometry_msgs::PoseStamped& _start, geometry_msgs::PoseStamped& _goal)
{
  open_list.clear();
  close_list.clear();
  for(int i=0;i<local_costmap.info.height*local_costmap.info.width;i++){
    cells[i].sum = -1;
    cells[i].parent_index = -1;
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
  std::cout << "calculating path" << std::endl;
  std::cout << "from " << _start.pose.position.x << ", " << _start.pose.position.y << ", " << tf::getYaw(_start.pose.orientation) << ", " << start_index << std::endl;
  std::cout << start_i << ", " << start_j << std::endl;
  std::cout << "to " << _goal.pose.position.x << ", " << _goal.pose.position.y << ", " << tf::getYaw(_goal.pose.orientation) << ", " << goal_index << std::endl;
  std::cout << goal_i << ", " << goal_j << std::endl;
  open_list.push_back(start_index);
  cells[open_list[0]].sum = cells[open_list[0]].step + get_heuristic(start_i - goal_i, start_j - goal_j);
  while(!open_list.empty() && ros::ok()){
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
      open_list.erase(std::remove(open_list.begin(), open_list.end(), n_index), open_list.end());      //openlistから削除
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
      if((std::find(open_list.begin(), open_list.end(), _index) == open_list.end()) && (std::find(close_list.begin(), close_list.end(), _index) == close_list.end())){
        if(!cells[_index].is_wall){
          cells[_index].step = cells[n_index].step + 1;
          cells[_index].sum = cells[_index].cost + cells[_index].step + get_heuristic(goal_i-_i, goal_j-(_j-1));
          cells[_index].parent_index = n_index;
          open_list.push_back(_index);
        }
      }else if(std::find(open_list.begin(), open_list.end(), _index) != open_list.end()){
        if(cells[n_index].step + 1 < cells[_index].step){
          cells[_index].parent_index = n_index;
        }
      }else if(std::find(close_list.begin(), close_list.end(), _index) != close_list.end()){
        if(cells[n_index].step + 1 < cells[_index].step){
          cells[_index].parent_index = n_index;
        }
      }
    }
    if(_j+1<local_costmap.info.width){
      _index = (_j+1)*local_costmap.info.width+_i;//i, j+1
      if((std::find(open_list.begin(), open_list.end(), _index) == open_list.end()) && (std::find(close_list.begin(), close_list.end(), _index) == close_list.end())){
        if(!cells[_index].is_wall){
          cells[_index].step = cells[n_index].step + 1;
          cells[_index].sum = cells[_index].cost + cells[_index].step + get_heuristic(goal_i-_i, goal_j-(_j+1));
          cells[_index].parent_index = n_index;
          open_list.push_back(_index);
        }
      }else if(std::find(open_list.begin(), open_list.end(), _index) != open_list.end()){
        if(cells[n_index].step + 1 < cells[_index].step){
          cells[_index].parent_index = n_index;
        }
      }else if(std::find(close_list.begin(), close_list.end(), _index) != close_list.end()){
        if(cells[n_index].step + 1 < cells[_index].step){
          cells[_index].parent_index = n_index;
        }
      }

    }
    if(_i+1<local_costmap.info.height){
      _index = _j*local_costmap.info.width+(_i+1);//i+1, j
      if((std::find(open_list.begin(), open_list.end(), _index) == open_list.end()) && (std::find(close_list.begin(), close_list.end(), _index) == close_list.end())){
        if(!cells[_index].is_wall){
          cells[_index].step = cells[n_index].step + 1;
          cells[_index].sum = cells[_index].cost + cells[_index].step + get_heuristic(goal_i-(_i+1), goal_j-_j);
          cells[_index].parent_index = n_index;
          open_list.push_back(_index);
        }
      }else if(std::find(open_list.begin(), open_list.end(), _index) != open_list.end()){
        if(cells[n_index].step + 1 < cells[_index].step){
          cells[_index].parent_index = n_index;
        }
      }else if(std::find(close_list.begin(), close_list.end(), _index) != close_list.end()){
        if(cells[n_index].step + 1 < cells[_index].step){
          cells[_index].parent_index = n_index;
        }
      }

      if(_j-1>=0){
        _index = (_j-1)*local_costmap.info.width+(_i+1);//i+1, j-1
        if((std::find(open_list.begin(), open_list.end(), _index) == open_list.end()) && (std::find(close_list.begin(), close_list.end(), _index) == close_list.end())){
          if(!cells[_index].is_wall){
            cells[_index].step = cells[n_index].step + 1;
            cells[_index].sum = cells[_index].cost + cells[_index].step + get_heuristic(goal_i-(_i+1), goal_j-(_j-1));
            cells[_index].parent_index = n_index;
            open_list.push_back(_index);
          }
        }else if(std::find(open_list.begin(), open_list.end(), _index) != open_list.end()){
          if(cells[n_index].step + 1 < cells[_index].step){
            cells[_index].parent_index = n_index;
          }
        }else if(std::find(close_list.begin(), close_list.end(), _index) != close_list.end()){
          if(cells[n_index].step + 1 < cells[_index].step){
            cells[_index].parent_index = n_index;
          }
        }

      }
      if(_j+1<local_costmap.info.width){
        _index = (_j+1)*local_costmap.info.width+(_i+1);//i+1, j+1
        if((std::find(open_list.begin(), open_list.end(), _index) == open_list.end()) && (std::find(close_list.begin(), close_list.end(), _index) == close_list.end())){
          if(!cells[_index].is_wall){
            cells[_index].step = cells[n_index].step + 1;
            cells[_index].sum = cells[_index].cost + cells[_index].step + get_heuristic(goal_i-(_i+1), goal_j-(_j+1));
            cells[_index].parent_index = n_index;
            open_list.push_back(_index);
          }
        }else if(std::find(open_list.begin(), open_list.end(), _index) != open_list.end()){
          if(cells[n_index].step + 1 < cells[_index].step){
            cells[_index].parent_index = n_index;
          }
        }else if(std::find(close_list.begin(), close_list.end(), _index) != close_list.end()){
          if(cells[n_index].step + 1 < cells[_index].step){
            cells[_index].parent_index = n_index;
          }
        }

      }

    }
    if(_i-1>=0){
      _index = _j*local_costmap.info.width+(_i-1);//i-1, j
      if((std::find(open_list.begin(), open_list.end(), _index) == open_list.end()) && (std::find(close_list.begin(), close_list.end(), _index) == close_list.end())){
        if(!cells[_index].is_wall){
          cells[_index].step = cells[n_index].step + 1;
          cells[_index].sum = cells[_index].cost + cells[_index].step + get_heuristic(goal_i-(_i-1), goal_j-_j);
          cells[_index].parent_index = n_index;
          open_list.push_back(_index);
        }
      }else if(std::find(open_list.begin(), open_list.end(), _index) != open_list.end()){
        if(cells[n_index].step + 1 < cells[_index].step){
          cells[_index].parent_index = n_index;
        }
      }else if(std::find(close_list.begin(), close_list.end(), _index) != close_list.end()){
        if(cells[n_index].step + 1 < cells[_index].step){
          cells[_index].parent_index = n_index;
        }
      }

      if(_j-1>=0){
        _index = (_j-1)*local_costmap.info.width+(_i-1);//i-1, j-1
        if((std::find(open_list.begin(), open_list.end(), _index) == open_list.end()) && (std::find(close_list.begin(), close_list.end(), _index) == close_list.end())){
          if(!cells[_index].is_wall){
            cells[_index].step = cells[n_index].step + 1;
            cells[_index].sum = cells[_index].cost + cells[_index].step + get_heuristic(goal_i-(_i-1), goal_j-(_j-1));
            cells[_index].parent_index = n_index;
            open_list.push_back(_index);
          }
        }else if(std::find(open_list.begin(), open_list.end(), _index) != open_list.end()){
          if(cells[n_index].step + 1 < cells[_index].step){
            cells[_index].parent_index = n_index;
          }
        }else if(std::find(close_list.begin(), close_list.end(), _index) != close_list.end()){
          if(cells[n_index].step + 1 < cells[_index].step){
            cells[_index].parent_index = n_index;
          }
        }

      }
      if(_j+1<local_costmap.info.width){
        _index = (_j+1)*local_costmap.info.width+(_i-1);//i-1, j+1
        if((std::find(open_list.begin(), open_list.end(), _index) == open_list.end()) && (std::find(close_list.begin(), close_list.end(), _index) == close_list.end())){
          if(!cells[_index].is_wall){
            cells[_index].step = cells[n_index].step + 1;
            cells[_index].sum = cells[_index].cost + cells[_index].step + get_heuristic(goal_i-(_i-1), goal_j-(_j+1));
            cells[_index].parent_index = n_index;
            open_list.push_back(_index);
          }
        }else if(std::find(open_list.begin(), open_list.end(), _index) != open_list.end()){
          if(cells[n_index].step + 1 < cells[_index].step){
            cells[_index].parent_index = n_index;
          }
        }else if(std::find(close_list.begin(), close_list.end(), _index) != close_list.end()){
          if(cells[n_index].step + 1 < cells[_index].step){
            cells[_index].parent_index = n_index;
          }
        }

      }

    }
  }
  nav_msgs::Path _path;
  _path.header.frame_id = "local_costmap";
  int path_index = goal_index;
  geometry_msgs::PoseStamped path_pose;
  path_pose.pose.orientation.w = 1;
  path_pose.header.frame_id = "local_costmap";
  while(1){
    path_pose.pose.position.x = (path_index % local_costmap.info.width) * local_costmap.info.resolution + local_costmap.info.origin.position.x;
    path_pose.pose.position.y = (path_index - (path_index % local_costmap.info.width)) / local_costmap.info.width * local_costmap.info.resolution + local_costmap.info.origin.position.y;
    path_pose.pose.orientation = _goal.pose.orientation;
    //std::cout << path_pose.pose.position.x << ", " << path_pose.pose.position.y << ", " << path_index << ", " << cells[path_index].cost << ", " << (int)local_costmap.data[path_index] << std::endl;
    //std::cout << cells[path_index].cost << std::endl;
    _path.poses.push_back(path_pose);
    path_index = cells[path_index].parent_index;
    //std::cout << "next:" << path_index << std::endl;
    if(path_index < 0){
      std::reverse(_path.poses.begin(), _path.poses.end());
      path = _path;
      std::cout << "=== path length ===" << std::endl;
      std::cout << path.poses.size() << std::endl;
      break;
    }
  }

  std::cout << "path generated!" << std::endl;
}
