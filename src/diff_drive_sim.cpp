#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/Int32.h>

std::vector<geometry_msgs::TransformStamped> obs_list;

const double DT = 0.1;//[s]

int NUM;

double get_yaw(geometry_msgs::Quaternion);
void set_pose(int, double, double, double);
void update(int, double, double);

int main(int argc, char** argv)
{
  ros::init(argc, argv, "diff_drive_sim");
  ros::NodeHandle nh;
  ros::NodeHandle local_nh("~");

  local_nh.getParam("NUM", NUM);

  ros::Publisher obs_num_pub = nh.advertise<std_msgs::Int32>("/obs_num", 100);

  obs_list.resize(NUM);

  tf::TransformBroadcaster obs_broadcaster;

  // 初期位置設定
  for(int i=0;i<NUM;i++){
    obs_list[i].header.frame_id = "map";
    obs_list[i].child_frame_id = "obs" + std::to_string(i);
    set_pose(i, 0, 0, 0);
  }
  set_pose(1, 1, 1, 0);
  set_pose(2, 1, -1, 0);
  set_pose(3, -1, 1, 0);
  set_pose(4, -1, -1, 0);

  ros::Rate loop_rate(10);

  while(ros::ok()){
    // 速度
    for(int i=0;i<NUM;i++){
      update(i, 0.5, 0.5);
    }
    obs_broadcaster.sendTransform(obs_list);

    std_msgs::Int32 num;
    num.data = NUM;
    obs_num_pub.publish(num);

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}

double get_yaw(geometry_msgs::Quaternion q)
{
  double r, p, y;
  tf::Quaternion quat(q.x, q.y, q.z, q.w);
  tf::Matrix3x3(quat).getRPY(r, p, y);
  return y;
}

// map frame
void set_pose(int index, double x, double y, double yaw)
{
  obs_list[index].header.stamp = ros::Time::now();
  obs_list[index].transform.translation.x = x;
  obs_list[index].transform.translation.y = y;
  obs_list[index].transform.rotation = tf::createQuaternionMsgFromYaw(yaw);
}

// 各obs frame
void update(int index, double v, double omega)
{
  obs_list[index].header.stamp = ros::Time::now();
  double yaw = tf::getYaw(obs_list[index].transform.rotation);
  obs_list[index].transform.translation.x += v * cos(yaw) * DT;
  obs_list[index].transform.translation.y += v * sin(yaw) * DT;
  obs_list[index].transform.rotation = tf::createQuaternionMsgFromYaw(yaw + omega * DT);
}
