#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/Int32.h>

#include <random>

std::vector<geometry_msgs::TransformStamped> obs_list;

const double DT = 0.1;//[s]
const double HZ = 100;

int NUM;

std::string WORLD_FRAME;
std::string OBS_FRAME;

double get_yaw(geometry_msgs::Quaternion);
void set_pose(int, double, double, double);
void update(int, double, double);

int main(int argc, char** argv)
{
    ros::init(argc, argv, "diff_drive_sim");
    ros::NodeHandle nh;
    ros::NodeHandle local_nh("~");

    local_nh.getParam("/dynamic_avoidance/WORLD_FRAME", WORLD_FRAME);
    local_nh.getParam("/dynamic_avoidance/OBSTACLES_FRAME", OBS_FRAME);

    NUM = 1;

    obs_list.resize(NUM);

    tf::TransformBroadcaster obs_broadcaster;

    // 初期位置設定
    for(int i=0;i<NUM;i++){
        obs_list[i].header.frame_id = WORLD_FRAME;
        obs_list[i].child_frame_id = OBS_FRAME + std::to_string(i);
        set_pose(i, 0, 0, 0);
    }
    set_pose(0, 20, 0.0, M_PI);
    // set_pose(1, 8.2, 4.2, -3*M_PI/4.0);

    ros::Rate loop_rate(HZ);

    double sim_start_time = ros::Time::now().toSec();

    while(ros::ok()){
        // 速度
        update(0, 1.2, 0);
        obs_broadcaster.sendTransform(obs_list);

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
    obs_list[index].transform.translation.x += v * cos(yaw) / HZ;
    obs_list[index].transform.translation.y += v * sin(yaw) / HZ;
    obs_list[index].transform.rotation = tf::createQuaternionMsgFromYaw(yaw + omega / HZ);
}
