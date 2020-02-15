#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Int32.h>

class CollisionDetector
{
public:
    CollisionDetector(void);

    void process(void);
    bool detect_collision(geometry_msgs::Pose, geometry_msgs::Pose);
    geometry_msgs::PoseArray get_obstacles(void);

private:
    double RADIUS;
    std::string WORLD_FRAME;
    std::string ROBOT_FRAME;
    std::string OBS_PREFIX;

    ros::NodeHandle nh;
    ros::NodeHandle local_nh;

    tf::TransformListener listener;
};

CollisionDetector::CollisionDetector(void)
:local_nh("~")
{
    local_nh.param<double>("/dynamic_avoidance/RADIUS", RADIUS, {0.6});
    local_nh.param<std::string>("/dynamic_avoidance/ROBOT_FRAME", ROBOT_FRAME, {"base_link"});
    local_nh.param<std::string>("/dynamic_avoidance/WORLD_FRAME", WORLD_FRAME, {"world"});
    local_nh.param<std::string>("/dynamic_avoidance/OBSTACLES_FRAME", OBS_PREFIX, {"obs"});
}

void CollisionDetector::process(void)
{
    geometry_msgs::PoseStamped current_robot;
    current_robot.header.frame_id = WORLD_FRAME;

    ros::Rate loop_rate(10);

    std::cout << "=== collision detector ===" << std::endl;
    while(ros::ok()){
        bool transformed_flag = false;
        try{
            tf::StampedTransform transform;
            listener.lookupTransform(WORLD_FRAME, ROBOT_FRAME, ros::Time(0), transform);
            current_robot.header.stamp = transform.stamp_;
            tf::poseTFToMsg(transform, current_robot.pose);
            transformed_flag = true;
        }catch(tf::TransformException& ex){
            std::cout << ex.what() << std::endl;
        }
        if(transformed_flag){
            geometry_msgs::PoseArray obstacles = get_obstacles();
            int obs_num = obstacles.poses.size();
            // std::cout << "obs num:" << obs_num << std::endl;
            for(int i=0;i<obs_num;i++){
                if(detect_collision(current_robot.pose, obstacles.poses[i])){
                    ROS_INFO_STREAM("\033[31m" << "collision detected with obs" << std::to_string(i) << "\033[0m");
                }
            }
            obstacles.poses.clear();
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "collision_detector");
    CollisionDetector cd;
    cd.process();
    return 0;
}

bool CollisionDetector::detect_collision(geometry_msgs::Pose p0, geometry_msgs::Pose p1)
{
    /*
     * collision->true;
     */
    double dx = p0.position.x - p1.position.x;
    double dy = p0.position.y - p1.position.y;
    double distance = sqrt(dx*dx + dy*dy);
    static double min_distance = 1e6;
    if(distance < min_distance){
        min_distance = distance;
        std::cout << "min distance: " << min_distance << "[m]" << std::endl;
    }
    if(distance <= RADIUS){
        std::cout << distance << "[m]" << std::endl;
        return true;
    }
    return false;
}

geometry_msgs::PoseArray CollisionDetector::get_obstacles(void)
{
    geometry_msgs::PoseArray obstacles;
    tf::FrameGraph::Request req;
    tf::FrameGraph::Response res;
    if(listener.getFrames(req, res)){
        std::vector<std::string> obs_names;
        while(1){
            // std::cout << res.dot_graph << std::endl;
            int position = res.dot_graph.find("-> \"" + OBS_PREFIX);
            // std::cout << "position: " << position << std::endl;
            if(position != std::string::npos){
                position = res.dot_graph.find(OBS_PREFIX);
                res.dot_graph.erase(0, position);
                int double_quotation_pos = res.dot_graph.find("\"");
                std::string obs_name = res.dot_graph.substr(0, double_quotation_pos);
                res.dot_graph.erase(0, double_quotation_pos);
                // std::cout << obs_name << std::endl;
                obs_names.emplace_back(obs_name);
            }else{
                // std::cout << OBS_PREFIX << " was not found" << std::endl;
                break;
            }
        }
        obstacles.header.frame_id = WORLD_FRAME;
        obstacles.header.stamp = ros::Time::now();
        for(const auto& obs_name : obs_names){
            try{
                tf::StampedTransform transform;
                listener.lookupTransform(WORLD_FRAME, obs_name, ros::Time(0), transform);
                obstacles.header.stamp = transform.stamp_;
                geometry_msgs::Pose pose;
                pose.position.x = transform.getOrigin().x();
                pose.position.y = transform.getOrigin().y();
                pose.position.z = transform.getOrigin().z();
                tf::quaternionTFToMsg(transform.getRotation(), pose.orientation);
                obstacles.poses.push_back(pose);
            }catch(tf::TransformException& ex){
                std::cout << ex.what() << std::endl;
            }
        }
    }else{
        std::cout << "cannot get frames" << std::endl;
    }
    return obstacles;
}
