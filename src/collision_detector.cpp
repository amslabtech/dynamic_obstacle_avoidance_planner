#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Int32.h>

int NUM = 1;
double RADIUS = 0;
std::string WORLD_FRAME;
std::string ROBOT_FRAME;
std::string OBS_FRAME;

void obs_num_callback(const std_msgs::Int32ConstPtr& msg)
{
  NUM = msg->data;
}

bool collision_detection(geometry_msgs::Pose, geometry_msgs::Pose);

int main(int argc, char** argv)
{
  ros::init(argc, argv, "collision_detector");
  ros::NodeHandle nh;
  ros::NodeHandle local_nh("~");

  local_nh.getParam("/dynamic_avoidance/RADIUS", RADIUS);
  local_nh.getParam("/dynamic_avoidance/ROBOT_FRAME", ROBOT_FRAME);
  local_nh.getParam("/dynamic_avoidance/WORLD_FRAME", WORLD_FRAME);
  local_nh.getParam("/dynamic_avoidance/OBSTACLES_FRAME", OBS_FRAME);

  ros::Subscriber obs_num_sub = nh.subscribe("/obs_num", 100, obs_num_callback);

  tf::TransformListener listener;

  geometry_msgs::PoseArray current_poses;
  current_poses.header.frame_id = WORLD_FRAME;

  geometry_msgs::PoseStamped current_robot;
  current_robot.header.frame_id = WORLD_FRAME;

  ros::Rate loop_rate(10);

  std::cout << "=== collision predictor ===" << std::endl;
  while(ros::ok()){
    if(NUM > 0){
      bool transformed = false;
      try{
        {
          tf::StampedTransform _transform;
          listener.lookupTransform(WORLD_FRAME, ROBOT_FRAME, ros::Time(0), _transform);
          geometry_msgs::TransformStamped transform;
          tf::transformStampedTFToMsg(_transform, transform);
          current_robot.pose.position.x = transform.transform.translation.x;
          current_robot.pose.position.y = transform.transform.translation.y;
          current_robot.pose.orientation = transform.transform.rotation;
        }
        for(int i=0;i<NUM;i++){
          std::string frame = OBS_FRAME + std::to_string(i);
          tf::StampedTransform _transform;
          listener.lookupTransform(WORLD_FRAME, frame, ros::Time(0), _transform);
          geometry_msgs::TransformStamped transform;
          tf::transformStampedTFToMsg(_transform, transform);
          geometry_msgs::Pose pose;
          pose.position.x = transform.transform.translation.x;
          pose.position.y = transform.transform.translation.y;
          pose.orientation = transform.transform.rotation;
          current_poses.poses.push_back(pose);
        }
        transformed = true;
      }catch(tf::TransformException ex){
        std::cout << ex.what() << std::endl;
      }
      if(transformed){
        for(int i=0;i<NUM;i++){
          if(collision_detection(current_robot.pose, current_poses.poses[i])){
            std::cout << "collision detected with obs" << std::to_string(i) << std::endl;
          }
        }
      }
    }
    current_poses.poses.clear();
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
  if(distance <= RADIUS){
    std::cout << distance << "[m]" << std::endl;
    return true;
  }
  return false;
}
