#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseArray.h>


double HZ = 100.0;

visualization_msgs::Marker lines;

std::string ROBOT_FRAME;
std::string OBS_PREFIX;
std::string WORLD_FRAME;
double PREDICTION_TIME;
int PREDICTION_STEP;
const double TRAJ_WIDTH = 0.03;

typedef std::map<std::string, visualization_msgs::Marker> NamedTrajectory;

void path_callback(const geometry_msgs::PoseArrayConstPtr& msg)
{
    lines.points.clear();
    geometry_msgs::PoseArray pose_array;
    pose_array = *msg;
    lines.header = pose_array.header;
    lines.color.r = 0;
    lines.color.g = 0;
    lines.color.b = 1;
    lines.color.a = 1;
    for(int i=0;i<PREDICTION_STEP;i++){
        lines.points.push_back(pose_array.poses[i].position);
        lines.points.push_back(pose_array.poses[i+PREDICTION_STEP].position);
        lines.points.push_back(pose_array.poses[i].position);
        lines.points.push_back(pose_array.poses[i+2*PREDICTION_STEP].position);
    }
}

visualization_msgs::Marker get_marker(int type, double sx, double sy, double sz, double r, double g, double b, const geometry_msgs::Pose& p, const std::string& frame_id, const std::string& ns)
{
    visualization_msgs::Marker m;
    m.header.frame_id = frame_id;
    m.action = visualization_msgs::Marker::ADD;
    m.ns = ns;
    m.pose = p;
    m.type = type;
    m.scale.x = sx;
    m.scale.y = sy;
    m.scale.z = sz;
    m.color.r = r;
    m.color.g = g;
    m.color.b = b;
    m.color.a = 1;
    return m;
}

visualization_msgs::MarkerArray get_obs_trajectory_markers(const NamedTrajectory& trajectories)
{
    visualization_msgs::MarkerArray ma;
    for(const auto& traj : trajectories){
        ma.markers.push_back(traj.second);
    }
    return ma;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "trajectory_logger");
    ros::NodeHandle nh;

    ros::NodeHandle local_nh;
    local_nh.param("/dynamic_avoidance/ROBOT_FRAME", ROBOT_FRAME, {"base_link"});
    local_nh.param("/dynamic_avoidance/WORLD_FRAME", WORLD_FRAME, {"map"});
    local_nh.param("/dynamic_avoidance/PREDICTION_TIME", PREDICTION_TIME, {3.5});
    local_nh.param("/dynamic_avoidance/OBSTACLES_FRAME", OBS_PREFIX, {"obs"});
    PREDICTION_STEP = PREDICTION_TIME / 0.1 + 1;

    std::cout << "=== trajectory_logger ===" << std::endl;

    ros::Rate loop_rate(HZ);

    tf::TransformListener listener;

    ros::Publisher robot_viz_pub = nh.advertise<visualization_msgs::Marker>("/marker/robot", 1);
    ros::Publisher obs_viz_pub = nh.advertise<visualization_msgs::MarkerArray>("/marker/obs", 1);
    ros::Publisher robot_trajectory_viz_pub = nh.advertise<visualization_msgs::Marker>("/robot_passed_trajectory", 1);
    ros::Publisher obs_trajectory_viz_pub = nh.advertise<visualization_msgs::MarkerArray>("/obs_passed_trajectory", 1);

    ros::Subscriber robot_path_sub = nh.subscribe("/robot_predicted_path", 1, path_callback);
    ros::Publisher robot_line_pub = nh.advertise<visualization_msgs::Marker>("/lines", 1);

    visualization_msgs::MarkerArray robot_viz;
    visualization_msgs::MarkerArray obs_viz;
    visualization_msgs::Marker robot_trajectory_viz;

    lines.type = visualization_msgs::Marker::LINE_LIST;
    lines.lifetime = ros::Duration(0.01);
    lines.action = visualization_msgs::Marker::ADD;
    lines.header.frame_id = WORLD_FRAME;
    lines.ns = "lines";
    lines.scale.x = 0.02;

    robot_trajectory_viz.type = visualization_msgs::Marker::LINE_STRIP;
    robot_trajectory_viz.lifetime = ros::Duration();
    robot_trajectory_viz.action = visualization_msgs::Marker::ADD;
    robot_trajectory_viz.header.frame_id = WORLD_FRAME;
    robot_trajectory_viz.ns = "robot_trajectory";
    robot_trajectory_viz.scale.x = TRAJ_WIDTH;
    robot_trajectory_viz.color.b = 1.0;
    robot_trajectory_viz.color.a = 0.8;
    robot_trajectory_viz.pose.orientation.w = 1;

    NamedTrajectory obs_trajectories;

    while(ros::ok()){
        static int count = 0;

        tf::FrameGraph::Request req;
        tf::FrameGraph::Response res;
        std::vector<std::string> obs_names;
        if(listener.getFrames(req, res)){
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
                    if(obs_trajectories.find(obs_name) == obs_trajectories.end()){
                        // obstacle with new name
                        visualization_msgs::Marker obs_traj_marker;
                        obs_traj_marker.type = visualization_msgs::Marker::LINE_STRIP;
                        obs_traj_marker.lifetime = ros::Duration();
                        obs_traj_marker.action = visualization_msgs::Marker::ADD;
                        obs_traj_marker.header.frame_id = WORLD_FRAME;
                        obs_traj_marker.scale.x = TRAJ_WIDTH;
                        obs_traj_marker.color.r = 1.0;
                        obs_traj_marker.color.a = 0.8;
                        obs_traj_marker.pose.orientation.w = 1;
                        obs_traj_marker.ns = obs_name;
                        obs_trajectories[obs_name] = obs_traj_marker;
                    }
                }else{
                    std::cout << OBS_PREFIX << " was not found" << std::endl;
                    break;
                }
            }
        }
        try{
            tf::StampedTransform robot_transform;
            listener.lookupTransform(WORLD_FRAME, ROBOT_FRAME, ros::Time(0), robot_transform);
            geometry_msgs::PoseStamped pose;
            tf::poseStampedTFToMsg(tf::Stamped<tf::Transform>(robot_transform, robot_transform.stamp_, robot_transform.frame_id_), pose);
            visualization_msgs::Marker viz;
            viz = get_marker(visualization_msgs::Marker::CUBE, 0.6, 0.6, 0.2, 0, 1, 0, pose.pose, WORLD_FRAME, "robot");
            viz.header.stamp = ros::Time::now();
            viz.lifetime = ros::Duration();
            static bool first_flag = true;
            robot_viz_pub.publish(viz);
            if(!first_flag){
                robot_viz_pub.publish(viz);
            }
            first_flag = false;
            robot_line_pub.publish(lines);
            robot_trajectory_viz.points.push_back(pose.pose.position);
            robot_trajectory_viz_pub.publish(robot_trajectory_viz);
        }catch(tf::TransformException ex){
            std::cout << ex.what() << std::endl;
        }
        visualization_msgs::MarkerArray obs_markers;
        for(const auto& obs_name : obs_names){
            try{
                tf::StampedTransform obs_transform;
                listener.lookupTransform(WORLD_FRAME, obs_name, ros::Time(0), obs_transform);
                geometry_msgs::PoseStamped pose;
                tf::poseStampedTFToMsg(tf::Stamped<tf::Transform>(obs_transform, obs_transform.stamp_, obs_transform.frame_id_), pose);
                visualization_msgs::Marker viz;
                viz = get_marker(visualization_msgs::Marker::CUBE, 0.6, 0.6, 0.2, 1, 0, 0, pose.pose, WORLD_FRAME, obs_name);
                viz.header.stamp = ros::Time::now();
                viz.lifetime = ros::Duration();
                std::cout << viz << std::endl;
                obs_markers.markers.push_back(viz);
                obs_trajectories[obs_name].points.push_back(pose.pose.position);
                int size = obs_trajectories[obs_name].points.size();
                const int TRAJ_LIMIT = 300;
                if(size > TRAJ_LIMIT){
                    // erase old trajectory
                    obs_trajectories[obs_name].points.erase(obs_trajectories[obs_name].points.begin(), obs_trajectories[obs_name].points.begin() + (size - TRAJ_LIMIT));
                }
            }catch(tf::TransformException ex){
                std::cout << ex.what() << std::endl;
            }
        }
        obs_viz_pub.publish(obs_markers);
        obs_trajectory_viz_pub.publish(get_obs_trajectory_markers(obs_trajectories));
        count++;
        count %= 10;

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
