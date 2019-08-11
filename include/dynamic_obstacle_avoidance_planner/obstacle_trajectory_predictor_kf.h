#ifndef __OBSTACLE_TRAJECTORY_PREDICTOR_KF_H
#define __OBSTACLE_TRAJECTORY_PREDICTOR_KF_H

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/Int32.h>

#include <Eigen/Dense>

class ObstacleTrajectoryPredictorKF
{
public:
    ObstacleTrajectoryPredictorKF(void);

    void obs_num_callback(const std_msgs::Int32ConstPtr&);
    void process(void);

    class KalmanFilter
    {
    public:
        KalmanFilter(double, double, double);

        void set_interval(double);
        void update(double, double, double);
        void predict(void);
        Eigen::Vector3d get_velocity(void);

    private:
        const double SIGMA_A = 0.05;
        Eigen::Matrix<double, 3, 1> Z;// 観測
        Eigen::Matrix<double, 6, 1> X;// 状態
        Eigen::Matrix<double, 6, 6> F;// 動作モデル
        Eigen::Matrix<double, 6, 3> G;// ノイズ
        Eigen::Matrix<double, 6, 6> Q;//
        Eigen::Matrix<double, 6, 6> P;//
        Eigen::Matrix<double, 3, 6> H;// 観測モデル
        Eigen::Matrix<double, 3, 3> R;// 観測ノイズ
        Eigen::Matrix<double, 6, 6> I;// 単位行列
        double last_time;
    };

private:
    ros::NodeHandle nh;
    ros::NodeHandle local_nh;
    ros::Publisher predicted_paths_pub;
    ros::Subscriber obs_num_sub;

    double PREDICTION_TIME;// [s], 軌道予測時間
    double DT = 0.1;// [s]
    int PREDICTION_STEP;
    int NUM_OF_PATH = 2;// obs1つあたりpath2本
    double HZ = 10;
    std::string WORLD_FRAME;
    std::string OBS_FRAME;
    int NUM = 0;
    geometry_msgs::PoseArray predicted_paths;
    geometry_msgs::PoseArray current_poses;
    geometry_msgs::PoseArray previous_poses;
    tf::TransformListener listener;
    bool first_transform;
    bool second_transform;
    double last_time;
    std::vector<KalmanFilter> kf;
    tf::StampedTransform _transform;
};

#endif __OBSTACLE_TRAJECTORY_PREDICTOR_KF_H
