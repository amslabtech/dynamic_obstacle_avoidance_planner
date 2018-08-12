#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/Int32.h>

const double PREDICTION_TIME = 3.5;// [s], 軌道予測時間
const double DT = 0.1;// [s]
const int PREDICTION_STEP = PREDICTION_TIME / DT;
const int NUM_OF_PATH = 2;// obs1つあたりpath2本

int NUM = 0;

geometry_msgs::PoseArray predicted_pathes;
geometry_msgs::PoseArray current_poses;
geometry_msgs::PoseArray previous_poses;

std::vector<geometry_msgs::Twist> current_velocities;
std::vector<geometry_msgs::Twist> previous_velocities;

tf::StampedTransform _transform;

void obs_num_callback(const std_msgs::Int32ConstPtr& msg)
{
  NUM = msg->data;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "obstacle_predictor");
  ros::NodeHandle nh;
  ros::NodeHandle local_nh("~");

  ros::Publisher predicted_pathes_pub = nh.advertise<geometry_msgs::PoseArray>("/predicted_pathes", 100);
  ros::Subscriber obs_num_sub = nh.subscribe("/obs_num", 100, obs_num_callback);

  tf::TransformListener listener;

  bool first_transform = true;

  predicted_pathes.header.frame_id = "map";

  ros::Rate loop_rate(10);

  while(ros::ok()){
    if(NUM > 0){
      bool transformed = false;
      try{
        for(int i=0;i<NUM;i++){
          std::string frame = "obs" + std::to_string(i);
          listener.lookupTransform("map", frame, ros::Time(0), _transform);
          std::cout << "obs" + std::to_string(i) + " received" << std::endl;
          geometry_msgs::TransformStamped transform;
          tf::transformStampedTFToMsg(_transform, transform);
          geometry_msgs::Pose pose;
          pose.position.x = transform.transform.translation.x;
          pose.position.y = transform.transform.translation.y;
          pose.position.z = transform.transform.translation.z;
          pose.orientation = transform.transform.rotation;
          current_poses.poses.push_back(pose);
        }
        transformed = true;
      }catch(tf::TransformException ex){
        std::cout << ex.what() << std::endl;
      }
      if(transformed && first_transform){
        first_transform = false;
      }else if(transformed && !first_transform){
        std::cout << "===calculate velocity===" << std::endl;
        for(int i=0;i<NUM;i++){
          geometry_msgs::Twist velocity;
          velocity.linear.x = (current_poses.poses[i].position.x - previous_poses.poses[i].position.x) / DT;
          velocity.linear.y = (current_poses.poses[i].position.y - previous_poses.poses[i].position.y) / DT;
          velocity.angular.z = (tf::getYaw(current_poses.poses[i].orientation) - tf::getYaw(previous_poses.poses[i].orientation)) / DT;
          current_velocities.push_back(velocity);
          //std::cout << "obs" + i << std::endl;
          //std::cout << velocity << std::endl;
        }
        if(previous_velocities.size() == 0){
          previous_velocities = current_velocities;
        }
        std::cout << "===predict path===" << std::endl;
        predicted_pathes.poses.clear();
        // 速度・角速度維持する場合の推定
        for(int i=0;i<NUM;i++){
          predicted_pathes.poses.push_back(current_poses.poses[i]);
          double vx = current_velocities[i].linear.x;
          double vy = current_velocities[i].linear.y;
          double v = sqrt(vx*vx + vy*vy);
          double omega = current_velocities[i].angular.z;
          double yaw = tf::getYaw(current_poses.poses[i].orientation);
          for(int j=0;j<PREDICTION_STEP;j++){
            geometry_msgs::Pose pose;
            pose.position.x = predicted_pathes.poses[i*(PREDICTION_STEP+1)+j].position.x + vx * DT;
            pose.position.y = predicted_pathes.poses[i*(PREDICTION_STEP+1)+j].position.y + vy * DT;
            yaw += omega * DT;
            yaw = atan2(sin(yaw), cos(yaw));
            pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
            vx = v * cos(yaw);
            vy = v * sin(yaw);
            v = sqrt(vx*vx + vy*vy);
            omega = omega;
            predicted_pathes.poses.push_back(pose);
          }
        }
        // 角速度変化させる場合の推定
        const int SIZE_OF_LINEAR_PATH = predicted_pathes.poses.size();
        for(int i=0;i<NUM;i++){
          predicted_pathes.poses.push_back(current_poses.poses[i]);
          double vx = current_velocities[i].linear.x;
          double vy = current_velocities[i].linear.y;
          double v = sqrt(vx*vx + vy*vy);
          double omega = current_velocities[i].angular.z;
          double d_omega = (current_velocities[i].angular.z - previous_velocities[i].angular.z) / DT;
          double yaw = tf::getYaw(current_poses.poses[i].orientation);
          for(int j=0;j<PREDICTION_STEP;j++){
            geometry_msgs::Pose pose;
            pose.position.x = predicted_pathes.poses[i*(PREDICTION_STEP+1)+j+SIZE_OF_LINEAR_PATH].position.x + vx * DT;
            pose.position.y = predicted_pathes.poses[i*(PREDICTION_STEP+1)+j+SIZE_OF_LINEAR_PATH].position.y + vy * DT;
            yaw += omega * DT;
            yaw = atan2(sin(yaw), cos(yaw));
            pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
            vx = v * cos(yaw);
            vy = v * sin(yaw);
            v = sqrt(vx*vx + vy*vy);
            omega = omega + d_omega * DT;
            predicted_pathes.poses.push_back(pose);
          }
        }
        std::cout << predicted_pathes.poses.size() << std::endl;
        predicted_pathes_pub.publish(predicted_pathes);
      }
      previous_velocities = current_velocities;
      current_velocities.clear();
      previous_poses = current_poses;
      current_poses.poses.clear();
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}

