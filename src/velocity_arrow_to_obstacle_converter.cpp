#include "dynamic_obstacle_avoidance_planner/velocity_arrow_to_obstacle_converter.h"

VelocityArrowToObstacleConverter::VelocityArrowToObstacleConverter(void)
:local_nh("~")
{
    local_nh.param("/dynamic_avoidance/PREDICTION_TIME", PREDICTION_TIME, {3.5});
    local_nh.param("/dynamic_avoidance/WORLD_FRAME", WORLD_FRAME, {"map"});
    local_nh.param("/dynamic_avoidance/OBSTACLES_FRAME", OBS_FRAME, {"obs"});
    local_nh.param("HZ", HZ, {10});
    INTERVAL = 1.0 / HZ;
    PREDICTION_STEP = PREDICTION_TIME / INTERVAL;

    obs_num_pub = nh.advertise<std_msgs::Int32>("/obs_num", 1);
    predicted_paths_pub = nh.advertise<geometry_msgs::PoseArray>("/predicted_paths", 1);
    velocity_arrow_sub = nh.subscribe("/velocity_arrows", 1, &VelocityArrowToObstacleConverter::velocity_arrow_callback, this);
    velocity_arrow_updated = false;
    obs_poses.header.frame_id = WORLD_FRAME;
    obs_num.data = 0;
    predicted_paths.header.frame_id = WORLD_FRAME;

    std::cout << "=== velocity arrow to obstacle converter ===" << std::endl;
    std::cout << "PREDICTION_TIME: " << PREDICTION_TIME << std::endl;
    std::cout << "PREDICTION_STEP: " << PREDICTION_STEP << std::endl;
    std::cout << "WORLD_FRAME: " << WORLD_FRAME << std::endl;
    std::cout << "OBS_FRAME: " << OBS_FRAME << std::endl;

}

void VelocityArrowToObstacleConverter::process(void)
{
    ros::Rate loop_rate(HZ);

    while(ros::ok()){
        if(velocity_arrow_updated){
            std::cout << "=== velocity arrow to obstacle converter ===" << std::endl;
            predicted_paths.poses.clear();
            //std::cout << "obs_num : " << obs_num.data << std::endl;
            for(int i=0;i<obs_num.data;i++){
                double v = velocities[i].linear.x;
                double yaw = tf::getYaw(obs_poses.poses[i].orientation);
                double vx = v * cos(yaw);
                double vy = v * sin(yaw);
                double omega = 0;
                predicted_paths.poses.push_back(obs_poses.poses[i]);
                for(int j=0;j<PREDICTION_STEP;j++){
                    geometry_msgs::Pose pose;
                    pose.position.x = predicted_paths.poses[i*(PREDICTION_STEP+1)+j].position.x + vx * INTERVAL;
                    pose.position.y = predicted_paths.poses[i*(PREDICTION_STEP+1)+j].position.y + vy * INTERVAL;
                    yaw += omega * INTERVAL;
                    yaw = atan2(sin(yaw), cos(yaw));
                    pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
                    //vx = v * cos(yaw);
                    //vy = v * sin(yaw);
                    v = sqrt(vx*vx + vy*vy);
                    omega = omega;
                    predicted_paths.poses.push_back(pose);
                }
            }
            obs_num_pub.publish(obs_num);
            predicted_paths_pub.publish(predicted_paths);
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
}

void VelocityArrowToObstacleConverter::velocity_arrow_callback(const visualization_msgs::MarkerArrayConstPtr& msg)
{
    velocities.clear();
    obs_poses.poses.clear();
    velocity_arrows = *msg;
    obs_num.data = velocity_arrows.markers.size();
    for(auto velocity_arrow : velocity_arrows.markers){
        geometry_msgs::Twist velocity;
        velocity.linear.x = velocity_arrow.scale.x;
        velocities.push_back(velocity);
        obs_poses.poses.push_back(velocity_arrow.pose);
    }
    velocity_arrow_updated = true;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "velocity_arrow_to_obstacle_converter_converter");
    VelocityArrowToObstacleConverter vatoc;
    vatoc.process();
    return 0;
}
