#include "dynamic_obstacle_avoidance_planner/tf_to_obstacles.h"

TFToObstacles::TFToObstacles(void)
:local_nh("~")
{
    local_nh.param("WORLD_FRAME", WORLD_FRAME, {"map"});
    local_nh.param("OBS_PREFIX", OBS_PREFIX, {"obs"});
    local_nh.param("HZ", HZ, {20});

    obstacles_pose_pub = nh.advertise<geometry_msgs::PoseArray>("/dynamic_obstacles", 1);

    std::cout << "=== tf_to_obstacles ===" << std::endl;
    std::cout << "WORLD_FRAME: " << WORLD_FRAME << std::endl;
    std::cout << "OBS_PREFIX: " << OBS_PREFIX << std::endl;
    std::cout << "HZ: " << HZ << std::endl;
}

void TFToObstacles::process(void)
{
    ros::Rate loop_rate(HZ);

    while(ros::ok()){
        std::cout << "=== tf_to_obstacles ===" << std::endl;
        tf::FrameGraph::Request req;
        tf::FrameGraph::Response res;
        if(listener.getFrames(req, res)){
            std::vector<std::string> obs_names;
            while(1){
                // std::cout << res.dot_graph << std::endl;
                int position = res.dot_graph.find("-> \"" + OBS_PREFIX);
                if(position != std::string::npos){
                    position = res.dot_graph.find(OBS_PREFIX);
                    res.dot_graph.erase(0, position);
                    int double_quotation_pos = res.dot_graph.find("\"");
                    std::string obs_name = res.dot_graph.substr(0, double_quotation_pos);
                    res.dot_graph.erase(0, double_quotation_pos);
                    std::cout << obs_name << std::endl;
                    obs_names.emplace_back(obs_name);
                }else{
                    std::cout << OBS_PREFIX << " was not found" << std::endl;
                    break;
                }
            }
            geometry_msgs::PoseArray obstacles;
            obstacles.header.frame_id = WORLD_FRAME;
            obstacles.header.stamp = ros::Time::now();
            for(const auto& obs_name : obs_names){
                try{
                    tf::StampedTransform transform;
                    listener.lookupTransform(WORLD_FRAME, obs_name, ros::Time(0), transform);
                    geometry_msgs::Pose pose;
                    pose.position.x = transform.getOrigin().x();
                    pose.position.y = transform.getOrigin().y();
                    pose.position.z = transform.getOrigin().z();
                    tf::quaternionTFToMsg(transform.getRotation(), pose.orientation);
                    obstacles.poses.push_back(pose);
                }catch(tf::TransformException ex){
                    std::cout << ex.what() << std::endl;
                }
            }
            obstacles_pose_pub.publish(obstacles);
        }else{
            std::cout << "cannot get frames" << std::endl;
        }
        loop_rate.sleep();
        ros::spinOnce();
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "tf_to_obstacles");
    TFToObstacles tf_to_obstacles;
    tf_to_obstacles.process();
    return 0;
}
