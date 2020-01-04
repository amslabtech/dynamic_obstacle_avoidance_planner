#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include "dynamic_obstacle_avoidance_planner/obstacle_tracker_kf.h"

class ObstacleTrackerKFDemo
{
public:
    ObstacleTrackerKFDemo(void);

    void obstacles_callback(const geometry_msgs::PoseArrayConstPtr&);
    void process(void);

private:
    std::string FIXED_FRAME;

    ros::NodeHandle nh;
    ros::NodeHandle local_nh;
    ros::Publisher velocity_arrows_pub;
    ros::Subscriber obstacles_sub;

    tf::TransformListener listener;

    ObstacleTrackerKF tracker;
};

ObstacleTrackerKFDemo::ObstacleTrackerKFDemo(void)
:local_nh("~")
{
    velocity_arrows_pub = local_nh.advertise<visualization_msgs::MarkerArray>("velocity_arrows", 1);
    obstacles_sub = nh.subscribe("/dynamic_obstacles", 1, &ObstacleTrackerKFDemo::obstacles_callback, this);

    local_nh.param<std::string>("FIXED_FRAME", FIXED_FRAME, {"/odom"});
}

void ObstacleTrackerKFDemo::obstacles_callback(const geometry_msgs::PoseArrayConstPtr& msg)
{
    std::cout << "obstacles callback" << std::endl;
    geometry_msgs::PoseArray obstacle_pose;
    obstacle_pose = *msg;
    try{
        for(auto& p : obstacle_pose.poses){
            geometry_msgs::PoseStamped p_;
            p_.header = obstacle_pose.header;
            p_.pose = p;
            listener.transformPose(FIXED_FRAME, p_, p_);
            p = p_.pose;
        }
        obstacle_pose.header.frame_id = FIXED_FRAME;
    }catch(tf::TransformException ex){
        std::cout << ex.what() << std::endl;
        return;
    }
    tracker.set_obstacles_pose(obstacle_pose);
    std::vector<Eigen::Vector3d> poses;
    tracker.get_poses(poses);
    std::vector<Eigen::Vector3d> velocities;
    tracker.get_velocities(velocities);

    static int last_obs_num = 0;
    int obs_num = poses.size();
    visualization_msgs::MarkerArray velocity_arrows;
    for(int i=0;i<obs_num;i++){
        visualization_msgs::Marker v_arrow;
        v_arrow.header.stamp = msg->header.stamp;
        v_arrow.header.frame_id = FIXED_FRAME;
        v_arrow.id = i;
        v_arrow.ns = "obstacle_tracker_kf_demo";
        v_arrow.type = visualization_msgs::Marker::ARROW;
        v_arrow.action = visualization_msgs::Marker::ADD;
        v_arrow.lifetime = ros::Duration();
        v_arrow.pose.position.x = poses[i](0);
        v_arrow.pose.position.y = poses[i](1);
        v_arrow.pose.position.z = poses[i](2);
        double direction = atan2(velocities[i](1), velocities[i](0));
        v_arrow.pose.orientation = tf::createQuaternionMsgFromYaw(direction);
        v_arrow.scale.x = velocities[i].norm();
        v_arrow.scale.y = v_arrow.scale.z = 0.3;
        v_arrow.color.r = 1.0;
        v_arrow.color.a = 1.0;
        velocity_arrows.markers.push_back(v_arrow);
    }
    for(int i=obs_num;i<last_obs_num;i++){
        visualization_msgs::Marker v_arrow;
        v_arrow.id = i;
        v_arrow.ns = "obstacle_tracker_kf_demo";
        v_arrow.action = visualization_msgs::Marker::DELETE;
        v_arrow.pose.orientation = tf::createQuaternionMsgFromYaw(0);
        velocity_arrows.markers.push_back(v_arrow);
    }
    velocity_arrows_pub.publish(velocity_arrows);
    last_obs_num = obs_num;
}

void ObstacleTrackerKFDemo::process(void)
{
    std::cout << "=== obstacle_tracker_kf_demo ===" << std::endl;
    ros::spin();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "obstacle_tracker");
    ObstacleTrackerKFDemo obstacle_tracker_kf_demo;
    obstacle_tracker_kf_demo.process();
    return 0;
}
