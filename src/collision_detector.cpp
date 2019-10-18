#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Int32.h>

double RADIUS = 0;
std::string WORLD_FRAME;
std::string ROBOT_FRAME;
std::string OBS_FRAME;

geometry_msgs::PoseArray obstacles;

bool collision_detection(geometry_msgs::Pose, geometry_msgs::Pose);

void obstacle_callback(const geometry_msgs::PoseArrayConstPtr& msg)
{
    obstacles = *msg;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "collision_detector");
    ros::NodeHandle nh;
    ros::NodeHandle local_nh("~");

    local_nh.getParam("/dynamic_avoidance/RADIUS", RADIUS);
    local_nh.getParam("/dynamic_avoidance/ROBOT_FRAME", ROBOT_FRAME);
    local_nh.getParam("/dynamic_avoidance/WORLD_FRAME", WORLD_FRAME);
    local_nh.getParam("/dynamic_avoidance/OBSTACLES_FRAME", OBS_FRAME);

    ros::Subscriber obstacles_sub = nh.subscribe("/dynamic_obstacles", 1, obstacle_callback);

    tf::TransformListener listener;

    geometry_msgs::PoseStamped current_robot;
    current_robot.header.frame_id = WORLD_FRAME;

    ros::Rate loop_rate(10);

    std::cout << "=== collision detector ===" << std::endl;
    while(ros::ok()){
        int obs_num = obstacles.poses.size();
        if(obs_num > 0){
            bool transformed = false;
            try{
                tf::StampedTransform _transform;
                listener.lookupTransform(WORLD_FRAME, ROBOT_FRAME, ros::Time(0), _transform);
                geometry_msgs::TransformStamped transform;
                tf::transformStampedTFToMsg(_transform, transform);
                current_robot.pose.position.x = transform.transform.translation.x;
                current_robot.pose.position.y = transform.transform.translation.y;
                current_robot.pose.orientation = transform.transform.rotation;
                for(int i=0;i<obs_num;i++){
                    geometry_msgs::PoseStamped pose;
                    pose.header = obstacles.header;
                    pose.pose = obstacles.poses[i];
                    listener.transformPose(WORLD_FRAME, pose, pose);
                }
                transformed = true;
            }catch(tf::TransformException ex){
                std::cout << ex.what() << std::endl;
            }
            if(transformed){
                for(int i=0;i<obs_num;i++){
                    if(collision_detection(current_robot.pose, obstacles.poses[i])){
                        std::cout << "\033[31m" << "collision detected with obs" << std::to_string(i) << "\033[0m" << std::endl;
                    }
                }
            }
        }
        obstacles.poses.clear();
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

bool collision_detection(geometry_msgs::Pose p0, geometry_msgs::Pose p1)
{
    /*
     * 衝突->true;
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
