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

int main(int argc, char** argv)
{
    ros::init(argc, argv, "trajectory_logger");
    ros::NodeHandle nh;

    ros::NodeHandle local_nh;
    local_nh.getParam("/dynamic_avoidance/ROBOT_FRAME", ROBOT_FRAME);
    local_nh.getParam("/dynamic_avoidance/WORLD_FRAME", WORLD_FRAME);
    local_nh.getParam("/dynamic_avoidance/PREDICTION_TIME", PREDICTION_TIME);
    local_nh.param("OBS_PREFIX", OBS_PREFIX, {"obs"});
    PREDICTION_STEP = PREDICTION_TIME / 0.1 + 1;

    std::cout << "=== trajectory_logger ===" << std::endl;

    ros::Rate loop_rate(HZ);

    tf::TransformListener listener;

    ros::Publisher robot_path_pub = nh.advertise<nav_msgs::Path>("/trajectory/robot", 1);
    ros::Publisher obs_path_pub = nh.advertise<nav_msgs::Path>("/trajectory/obs", 1);
    ros::Publisher robot_viz_pub = nh.advertise<visualization_msgs::Marker>("/marker/robot", 1);
    ros::Publisher obs_viz_pub = nh.advertise<visualization_msgs::MarkerArray>("/marker/obs", 1);
    ros::Publisher robot_viz_array_pub = nh.advertise<visualization_msgs::MarkerArray>("/markers/robot", 1);
    ros::Publisher obs_viz_array_pub = nh.advertise<visualization_msgs::MarkerArray>("/markers/obs", 1);

    ros::Subscriber robot_path_sub = nh.subscribe("/robot_predicted_path", 1, path_callback);
    ros::Publisher robot_line_pub = nh.advertise<visualization_msgs::Marker>("/lines", 1);

    nav_msgs::Path robot_path;
    nav_msgs::Path obs_path;
    visualization_msgs::MarkerArray robot_viz;
    visualization_msgs::MarkerArray obs_viz;

    robot_path.header.frame_id = WORLD_FRAME;
    obs_path.header.frame_id = WORLD_FRAME;

    lines.type = visualization_msgs::Marker::LINE_LIST;
    lines.lifetime = ros::Duration(0.01);
    lines.action = visualization_msgs::Marker::ADD;
    lines.header.frame_id = WORLD_FRAME;
    lines.ns = "lines";
    lines.scale.x = 0.01;

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
                }else{
                    std::cout << OBS_PREFIX << " was not found" << std::endl;
                    break;
                }
            }
        }
        try{
            tf::StampedTransform robot_transform;
            listener.lookupTransform(WORLD_FRAME, ROBOT_FRAME, ros::Time(0), robot_transform);
            robot_path.header.stamp = robot_transform.stamp_;
            geometry_msgs::PoseStamped pose;
            tf::poseStampedTFToMsg(tf::Stamped<tf::Transform>(robot_transform, robot_transform.stamp_, robot_transform.frame_id_), pose);
            robot_path.poses.push_back(pose);
            visualization_msgs::Marker viz;
            viz = get_marker(visualization_msgs::Marker::CYLINDER, 0.6, 0.6, 0.2, 0, 1, 0, pose.pose, WORLD_FRAME, "robot");
            viz.header.stamp = ros::Time::now();
            viz.lifetime = ros::Duration(0.01);
            robot_viz_pub.publish(viz);
            viz.lifetime = ros::Duration(0);
            if(count == 0){
                viz.id = count;
                robot_viz.markers.push_back(viz);
            }
            robot_path_pub.publish(robot_path);
            robot_viz_array_pub.publish(robot_viz);
            robot_line_pub.publish(lines);
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
                obs_path.poses.push_back(pose);
                visualization_msgs::Marker viz;
                viz = get_marker(visualization_msgs::Marker::CYLINDER, 0.6, 0.6, 0.2, 1, 0, 0, pose.pose, WORLD_FRAME, obs_name);
                viz.header.stamp = ros::Time::now();
                viz.lifetime = ros::Duration(0.01);
                std::cout << viz << std::endl;
                obs_markers.markers.push_back(viz);
                // obs_viz_pub.publish(viz);
                viz.lifetime = ros::Duration(0);
                if(count == 0){
                    viz.id = count;
                    obs_viz.markers.push_back(viz);
                }
                obs_path_pub.publish(obs_path);
                obs_viz_array_pub.publish(obs_viz);
            }catch(tf::TransformException ex){
                std::cout << ex.what() << std::endl;
            }
        }
        obs_viz_pub.publish(obs_markers);
        count++;
        count %= 10;

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
