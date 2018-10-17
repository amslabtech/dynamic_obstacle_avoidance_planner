#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>

const double PREDICTION_TIME = 3.5;// [s], 軌道予測時間
const double DT = 0.1;// [s]
const int PREDICTION_STEP = PREDICTION_TIME / DT;
const double HZ = 10;

double ANGULAR_ACCELERATION = 0.0;
double MAX_ANGULAR_VELOCITY = 0.0;

geometry_msgs::PoseArray predicted_path;
geometry_msgs::Pose current_pose;
geometry_msgs::Pose previous_pose;

geometry_msgs::Twist current_velocity;
geometry_msgs::Twist previous_velocity;

tf::StampedTransform _transform;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "robot_predictor");
  ros::NodeHandle nh;
  ros::NodeHandle local_nh("~");

  local_nh.getParam("ANGULAR_ACCELERATION", ANGULAR_ACCELERATION);
  local_nh.getParam("MAX_ANGULAR_VELOCITY", MAX_ANGULAR_VELOCITY);

  ros::Publisher predicted_path_pub = nh.advertise<geometry_msgs::PoseArray>("/robot_predicted_path", 100);

  tf::TransformListener listener;

  bool first_transform = true;

  predicted_path.header.frame_id = "map";

  ros::Rate loop_rate(HZ);

  while(ros::ok()){
    bool transformed = false;
    try{
      listener.lookupTransform("map", "base_link", ros::Time(0), _transform);
      geometry_msgs::TransformStamped transform;
      tf::transformStampedTFToMsg(_transform, transform);
      geometry_msgs::Pose pose;
      pose.position.x = transform.transform.translation.x;
      pose.position.y = transform.transform.translation.y;
      pose.position.z = transform.transform.translation.z;
      pose.orientation = transform.transform.rotation;
      current_pose = pose;
      transformed = true;
    }catch(tf::TransformException ex){
      std::cout << ex.what() << std::endl;
    }
    if(transformed && first_transform){
      first_transform = false;
    }else if(transformed && !first_transform){
      std::cout << "===calculate velocity===" << std::endl;
      geometry_msgs::Twist velocity;
      velocity.linear.x = (current_pose.position.x - previous_pose.position.x) * HZ;
      velocity.linear.y = (current_pose.position.y - previous_pose.position.y) * HZ;
      velocity.angular.z = (tf::getYaw(current_pose.orientation) - tf::getYaw(previous_pose.orientation)) * HZ;
      current_velocity = velocity;
      std::cout << current_velocity << std::endl;
      std::cout << "===predict path===" << std::endl;
      predicted_path.poses.clear();
      // v=const, omega=0
      {
        predicted_path.poses.push_back(current_pose);
        double vx = current_velocity.linear.x;
        double vy = current_velocity.linear.y;
        double v = sqrt(vx*vx + vy*vy);
        double omega = current_velocity.angular.z;
        double yaw = tf::getYaw(current_pose.orientation);
        for(int j=0;j<PREDICTION_STEP;j++){
          geometry_msgs::Pose pose;
          pose.position.x = predicted_path.poses[j].position.x + vx * DT;
          pose.position.y = predicted_path.poses[j].position.y + vy * DT;
          yaw += omega * DT;
          yaw = atan2(sin(yaw), cos(yaw));
          pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
          vx = v * cos(yaw);
          vy = v * sin(yaw);
          v = sqrt(vx*vx + vy*vy);
          omega = omega;
          predicted_path.poses.push_back(pose);
        }
      }
      const int SIZE_OF_LINEAR_PATH = predicted_path.poses.size();
      // v=const, omega+=domega * dt
      {
        predicted_path.poses.push_back(current_pose);
        double vx = current_velocity.linear.x;
        double vy = current_velocity.linear.y;
        double v = sqrt(vx*vx + vy*vy);
        double omega = current_velocity.angular.z;
            omega += ANGULAR_ACCELERATION * DT;
        double yaw = tf::getYaw(current_pose.orientation);
        for(int j=0;j<PREDICTION_STEP;j++){
          geometry_msgs::Pose pose;
          pose.position.x = predicted_path.poses[j+SIZE_OF_LINEAR_PATH].position.x + vx * DT;
          pose.position.y = predicted_path.poses[j+SIZE_OF_LINEAR_PATH].position.y + vy * DT;
          yaw += omega * DT;
          yaw = atan2(sin(yaw), cos(yaw));
          pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
          vx = v * cos(yaw);
          vy = v * sin(yaw);
          v = sqrt(vx*vx + vy*vy);
          if(omega + ANGULAR_ACCELERATION * DT <= MAX_ANGULAR_VELOCITY){
            //omega += ANGULAR_ACCELERATION * DT;
          }
          predicted_path.poses.push_back(pose);
        }
      }
      // v=const, omega-=domega * dt
      {
        predicted_path.poses.push_back(current_pose);
        double vx = current_velocity.linear.x;
        double vy = current_velocity.linear.y;
        double v = sqrt(vx*vx + vy*vy);
        double omega = current_velocity.angular.z;
            omega -= ANGULAR_ACCELERATION * DT;
        double yaw = tf::getYaw(current_pose.orientation);
        for(int j=0;j<PREDICTION_STEP;j++){
          geometry_msgs::Pose pose;
          pose.position.x = predicted_path.poses[j+SIZE_OF_LINEAR_PATH*2].position.x + vx * DT;
          pose.position.y = predicted_path.poses[+j+SIZE_OF_LINEAR_PATH*2].position.y + vy * DT;
          yaw += omega * DT;
          yaw = atan2(sin(yaw), cos(yaw));
          pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
          vx = v * cos(yaw);
          vy = v * sin(yaw);
          v = sqrt(vx*vx + vy*vy);
          if(omega - ANGULAR_ACCELERATION * DT >= -MAX_ANGULAR_VELOCITY){
            //omega -= ANGULAR_ACCELERATION * DT;
          }
          predicted_path.poses.push_back(pose);
        }
      }
      std::cout << predicted_path.poses.size() << std::endl;
      predicted_path_pub.publish(predicted_path);
      previous_velocity = current_velocity;
      previous_pose = current_pose;
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}

