#include "dynamic_obstacle_avoidance_planner/avoidance_path_planner.h"

AvoidancePathPlanner::AvoidancePathPlanner(void)
:local_nh("~")
{
    local_nh.param("MARGIN_WALL", MARGIN_WALL, {1.0});
    local_nh.param("/dynamic_avoidance/INTERMEDIATE_PATH_TOPIC_NAME", INTERMEDIATE_PATH_TOPIC_NAME, {"/intermediate_path"});

    path_pub = nh.advertise<nav_msgs::Path>(INTERMEDIATE_PATH_TOPIC_NAME, 1);
    // path2用
    path2_pub = nh.advertise<nav_msgs::Path>(INTERMEDIATE_PATH_TOPIC_NAME + std::to_string(2), 1);
    map_sub = nh.subscribe("/local_costmap", 1, &AvoidancePathPlanner::map_callback, this);
    waypoints_sub = nh.subscribe("/waypoints", 1, &AvoidancePathPlanner::waypoints_callback, this);

    std::cout << "=== avoidance path planner ===" << std::endl;
    std::cout << "MARGIN_WALL: " << MARGIN_WALL << std::endl;
    std::cout << "INTERMEDIATE_PATH_TOPIC_NAME: " << INTERMEDIATE_PATH_TOPIC_NAME << std::endl;
}

void AvoidancePathPlanner::map_callback(const nav_msgs::OccupancyGridConstPtr& msg)
{
    std::cout << "=== map callback ===" << std::endl;
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

void AvoidancePathPlanner::waypoints_callback(const geometry_msgs::PoseArrayConstPtr& msg)
{
    std::cout << "=== wp callback ===" << std::endl;
    waypoints = *msg;
    if(waypoints.poses.size()==2){
        waypoints_received = true;
    }
}

inline int AvoidancePathPlanner::get_i_from_x(double x)
{
    int val = (x - local_costmap.info.origin.position.x) * (1.0 / local_costmap.info.resolution) + 0.5;
    int i = (int)val;
    return i - (i > val);
}

inline int AvoidancePathPlanner::get_j_from_y(double y)
{
    int val = (y - local_costmap.info.origin.position.y) * (1.0 / local_costmap.info.resolution) + 0.5;
    int i = (int)val;
    return i - (i > val);
}

inline int AvoidancePathPlanner::get_index(double x, double y)
{
    return local_costmap.info.width * get_j_from_y(y) + get_i_from_x(x);
}

inline int AvoidancePathPlanner::get_heuristic(int diff_x, int diff_y)
{
    return sqrt(diff_x*diff_x+diff_y*diff_y);
}

double AvoidancePathPlanner::calculate_astar(const geometry_msgs::PoseStamped& _start, const geometry_msgs::PoseStamped& _goal, nav_msgs::Path& _path)
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
    std::cout << "calculating path" << std::endl;
    std::cout << "from " << _start.pose.position.x << ", " << _start.pose.position.y << ", " << tf::getYaw(_start.pose.orientation) << ", " << start_index << std::endl;
    std::cout << start_i << ", " << start_j << std::endl;
    std::cout << "to " << _goal.pose.position.x << ", " << _goal.pose.position.y << ", " << tf::getYaw(_goal.pose.orientation) << ", " << goal_index << std::endl;
    std::cout << goal_i << ", " << goal_j << std::endl;
    open_list.push_back(start_index);
    cells[open_list[0]].sum = cells[open_list[0]].step + get_heuristic(start_i - goal_i, start_j - goal_j) + get_distance_to_global_path(start_i, start_j);

    int count = 0;

    int max_openlist_size = 0;
    double max_loop_time = 0;

    while(ros::ok()){
        if(open_list.empty()){
            std::cout << "open list is empty!" << std::endl;
            break;
        }
        double loop_start_time = ros::Time::now().toSec();

        count++;
        if(count > 10000){
            std::cout << "count > 10000" << std::endl;
            return 1000000;
        }
        int n_index = open_list[0];
        int n = cells[n_index].sum;
        for(int i=0;i<open_list.size();i++){// choose a element which has smallest cost from openlist
            if(cells[open_list[i]].sum < n){
                n_index = open_list[i];
                n = cells[n_index].sum;
            }
        }
        // std::cout << "openlist:" << open_list.size() << std::endl;
        // std::cout << "goal:" << goal_i << ", " << goal_j << std::endl;
        if(n_index != goal_index){
            close_list.push_back(n_index);
            open_list.erase(std::remove(open_list.begin(), open_list.end(), n_index), open_list.end());
        }else{
            std::cout << "--- goal ---" << std::endl;
            break;
        }

        int n_i = n_index % local_costmap.info.width;
        int n_j = (n_index - n_i) / local_costmap.info.width;
        // std::cout << "current:" << n_i << ", " << n_j << std::endl;
        // std::cout << "sum:" << cells[n_index].sum << std::endl;
        for(int _i=n_i-1;_i<=n_i+1;_i++){
            for(int _j=n_j-1;_j<=n_j+1;_j++){
                if(!((_i == n_i) && (_j == n_j))){
                    if(_i>=0 && _i<local_costmap.info.width && _j>=0 && _j<local_costmap.info.width){
                        int _index = _j * local_costmap.info.width + _i;
                        // std::cout << "_i, _j: " << _i << ", " << _j << std::endl;
                        if(!is_contained(open_list, _index) && !is_contained(close_list, _index)){
                            if(!cells[_index].is_wall){
                                // std::cout << "=== open ===" << std::endl;
                                cells[_index].step = cells[n_index].step + 1;
                                if(_i != n_i && _j != n_j){
                                    cells[_index].step++;
                                }
                                cells[_index].sum = cells[_index].cost + cells[_index].step + get_heuristic(goal_i-_i, goal_j-_j) + get_distance_to_global_path(_i, _j);
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
            }
        }
    }
    std::cout << "count = " << count << std::endl;
    nav_msgs::Path temp_path;
    temp_path.header.frame_id = "local_costmap";
    int path_index = goal_index;
    std::cout << "goal_index: " << goal_index << std::endl;
    geometry_msgs::PoseStamped path_pose;
    path_pose.pose.orientation.w = 1;
    path_pose.header.frame_id = "local_costmap";
    double total_cost = 0;
    while(1){
        total_cost += cells[path_index].sum;
        path_pose.pose.position.x = (path_index % local_costmap.info.width) * local_costmap.info.resolution + local_costmap.info.origin.position.x;
        path_pose.pose.position.y = (path_index - (path_index % local_costmap.info.width)) / local_costmap.info.width * local_costmap.info.resolution + local_costmap.info.origin.position.y;
        path_pose.pose.orientation = _goal.pose.orientation;
        std::cout << path_pose.pose.position.x << ", " << path_pose.pose.position.y << ", " << path_index << ", " << cells[path_index].cost << ", " << (int)local_costmap.data[path_index] << std::endl;
        std::cout << cells[path_index].cost << std::endl;
        temp_path.poses.push_back(path_pose);
        path_index = cells[path_index].parent_index;
        std::cout << "next:" << path_index << std::endl;
        if(path_index < 0){
            std::reverse(temp_path.poses.begin(), temp_path.poses.end());
            _path = temp_path;
            std::cout << "=== path length ===" << std::endl;
            std::cout << _path.poses.size() << std::endl;
            std::cout << "=== path cost ===" << std::endl;
            std::cout << total_cost << std::endl;
            std::cout << "path generated!" << std::endl;
            return total_cost;
        }
    }
}

void AvoidancePathPlanner::intersection_on_map(geometry_msgs::PoseStamped& out0, geometry_msgs::PoseStamped& out1)
{
    // four corners of costmap
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
    // calculate intersection point
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
        // distance form waypoint0 to each intersections
        double distance0 = (waypoint0.pose.position.x - out[0].pose.position.x) * (waypoint0.pose.position.x - out[0].pose.position.x) + (waypoint0.pose.position.y - out[0].pose.position.y) * (waypoint0.pose.position.y - out[0].pose.position.y);
        double distance1 = (waypoint0.pose.position.x - out[1].pose.position.x) * (waypoint0.pose.position.x - out[1].pose.position.x) + (waypoint0.pose.position.y - out[1].pose.position.y) * (waypoint0.pose.position.y - out[1].pose.position.y);
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
        // one of waypoints is in costmap
        if((waypoint0.pose.position.x >= local_costmap.info.origin.position.x) && (waypoint0.pose.position.x <= local_costmap.info.origin.position.x + local_costmap.info.height * local_costmap.info.resolution) && (waypoint0.pose.position.y >= local_costmap.info.origin.position.y) && (waypoint0.pose.position.y <= local_costmap.info.origin.position.y + local_costmap.info.width * local_costmap.info.resolution)){
            // waypoint0 is in costmap
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
            // waypoint1 is in costmap
            out1 = waypoint1;
        }

    }else{
        std::cout << "no intersection" << std::endl;
        out1 = waypoint1;
    }
}

/*
 * P1(a->b), P2(c->d) intersection
 * https://qiita.com/ykob/items/ab7f30c43a0ed52d16f2
 */
bool AvoidancePathPlanner::predict_intersection(const geometry_msgs::Pose& a, const geometry_msgs::Pose& b, const geometry_msgs::Pose& c, const geometry_msgs::Pose& d)
{
    double ta = (c.position.x - d.position.x) * (a.position.y - c.position.y) + (c.position.y - d.position.y) * (c.position.x - a.position.x);
    double tb = (c.position.x - d.position.x) * (b.position.y - c.position.y) + (c.position.y - d.position.y) * (c.position.x - b.position.x);
    double tc = (a.position.x - b.position.x) * (c.position.y - a.position.y) + (a.position.y - b.position.y) * (a.position.x - c.position.x);
    double td = (a.position.x - b.position.x) * (d.position.y - a.position.y) + (a.position.y - b.position.y) * (a.position.x - d.position.x);
    return tc * td <= 0 && ta * tb <= 0;
}

/*
 * P1(a->b), P2(c->d) intersection point
 * http://www.hiramine.com/programming/graphics/2d_segmentintersection.html
 */
void AvoidancePathPlanner::predict_intersection_point(const geometry_msgs::Pose& a, const geometry_msgs::Pose& b, const geometry_msgs::Pose& c, const geometry_msgs::Pose& d, geometry_msgs::PoseStamped& result)
{
    double denominator = (b.position.x - a.position.x) * (d.position.y - c.position.y) - (b.position.y - a.position.y) * (d.position.x - c.position.x);
    double r = ((d.position.y - c.position.y) * (c.position.x - a.position.x) - (d.position.x - c.position.x) * (c.position.y - a.position.y)) / denominator;
    //double s = ((b.position.y - a.position.y) * (c.position.x - a.position.x) - (b.position.x - a.position.x) * (c.position.y - a.position.y)) / denominator;
    result.pose.position.x = a.position.x + r * (b.position.x - a.position.x);
    result.pose.position.y = a.position.y + r * (b.position.y - a.position.y);
}

/*
 * reference: https://qiita.com/yellow_73/items/bcd4e150e7caa0210ee6
 */
int AvoidancePathPlanner::get_distance_to_global_path(int i, int j)
{
    int x1 = get_i_from_x(waypoint0.pose.position.x);
    int x2 = get_i_from_x(waypoint1.pose.position.x);
    int y1 = get_j_from_y(waypoint0.pose.position.y);
    int y2 = get_j_from_y(waypoint1.pose.position.y);
    int a = x2 - x1;
    int b = y2 - y1;
    int a2 = a * a;
    int b2 = b * b;
    int f1 = a * (y1 - j) - b * (x1 - i);
    return local_costmap.info.resolution * (f1 * f1) / (double)(a2 + b2);
}

double AvoidancePathPlanner::get_difference(const nav_msgs::Path& _path1, const nav_msgs::Path& _path2)
{
    double sum = 0;
    for(int i=0;(i<_path1.poses.size()) && (i<_path2.poses.size());i++){
        double dx = _path1.poses[i].pose.position.x - _path2.poses[i].pose.position.x;
        double dy = _path1.poses[i].pose.position.y - _path2.poses[i].pose.position.y;
        sum += dx * dx + dy * dy;
    }
    return sum;
}

bool AvoidancePathPlanner::is_contained(const std::vector<int>& v, int val)
{
    for(const auto& vi : v){
        if(vi == val){
            return true;
        }
    }
    return false;
}

void AvoidancePathPlanner::process(void)
{
    tf::TransformListener listener;

    bool first_flag = true;

    open_list.reserve(40000);
    close_list.reserve(40000);

    ros::Rate loop_rate(HZ);

    while(ros::ok()){
        if(map_received && waypoints_received){
            std::cout << "=== avoidance path planner ===" << std::endl;
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
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "avoidance_path_planner");
    AvoidancePathPlanner app;
    app.process();
    return 0;
}

