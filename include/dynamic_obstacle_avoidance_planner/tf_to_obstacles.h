#ifndef __TF_TO_OBSTACLES_H
#define __TF_TO_OBSTACLES_H

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/FrameGraph.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseArray.h>

class TFToObstacles
{
public:
    TFToObstacles(void);
    void process(void);

private:
    std::string WORLD_FRAME;
    std::string OBS_PREFIX;
    double HZ;

    ros::NodeHandle nh;
    ros::NodeHandle local_nh;
    ros::Publisher obstacles_pose_pub;
    tf::TransformListener listener;
};

#endif// __TF_TO_OBSTACLES_H
