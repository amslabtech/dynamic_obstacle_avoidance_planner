#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/MarkerArray.h>

const double INTERVAL = 0.1;

double PREDICTION_TIME;
int PREDICTION_STEP;
std::string WORLD_FRAME;
std::string OBS_FRAME;

class VelocityArrowToObstacle
{
public:
  VelocityArrowToObstacle(void);
  void waypoint_callback(const geometry_msgs::PoseArrayConstPtr&);
  void velocity_arrow_callback(const visualization_msgs::MarkerArrayConstPtr&);
  void process(void);

private:
  ros::NodeHandle nh;
  ros::Publisher obs_num_pub;
  ros::Publisher predicted_paths_pub;
  ros::Subscriber velocity_arrow_sub;
  geometry_msgs::PoseArray predicted_paths;
  bool velocity_arrow_subscribed;
  tf::TransformBroadcaster br;
  tf::StampedTransform _transform;
  visualization_msgs::MarkerArray velocity_arrows;
  geometry_msgs::TransformStamped transform;
  std::vector<geometry_msgs::Twist> velocities;
  geometry_msgs::PoseArray obs_poses;
  std_msgs::Int32 obs_num;
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "velocity_arrow_to_obstacle");
  ros::NodeHandle nh;
  ros::NodeHandle local_nh("~");
  local_nh.getParam("/dynamic_avoidance/PREDICTION_TIME", PREDICTION_TIME);
  local_nh.getParam("/dynamic_avoidance/WORLD_FRAME", WORLD_FRAME);
  local_nh.getParam("/dynamic_avoidance/OBSTACLES_FRAME", OBS_FRAME);
  PREDICTION_STEP = PREDICTION_TIME / INTERVAL;

  VelocityArrowToObstacle vato;
  vato.process();
}

VelocityArrowToObstacle::VelocityArrowToObstacle(void)
{
  obs_num_pub = nh.advertise<std_msgs::Int32>("/obs_num", 1);
  predicted_paths_pub = nh.advertise<geometry_msgs::PoseArray>("/predicted_paths", 1);
  velocity_arrow_sub = nh.subscribe("/velocity_arrows", 1, &VelocityArrowToObstacle::velocity_arrow_callback, this);
  velocity_arrow_subscribed = false;
  obs_poses.header.frame_id = WORLD_FRAME;
  obs_num.data = 0;
  predicted_paths.header.frame_id = WORLD_FRAME;
}

void VelocityArrowToObstacle::process(void)
{
  ros::Rate loop_rate(20);

  std::cout << "velocity_arrow_to_obstacle" << std::endl;
  std::cout << PREDICTION_TIME << std::endl;
  std::cout << PREDICTION_STEP << std::endl;
  std::cout << WORLD_FRAME << std::endl;
  std::cout << OBS_FRAME << std::endl;

  while(ros::ok()){
    if(velocity_arrow_subscribed){
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

void VelocityArrowToObstacle::velocity_arrow_callback(const visualization_msgs::MarkerArrayConstPtr& msg)
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
  velocity_arrow_subscribed = true;
}
