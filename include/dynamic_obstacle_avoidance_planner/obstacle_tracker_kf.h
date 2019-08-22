#ifndef __OBSTACLE_TRACKER_KF_H
#define __OBSTACLE_TRACKER_KF_H

#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>

#include <Eigen/Dense>

class KalmanFilter
{
public:
    KalmanFilter(void);

    Eigen::Matrix4d get_f(double);// transition model
    Eigen::Matrix4d get_q(double);// transition noise

    double sigma_a;
private:
};

class Obstacle
{
public:
    Obstacle(void);
    Obstacle(const Obstacle&);
    Obstacle(const Eigen::Vector2d&);

    void update(const Eigen::Vector2d&);
    void predict(void);
    void predict(double);
    Eigen::Vector2d get_position(void);
    double calculate_likelihood(void);

    Eigen::Vector2d position;
    Eigen::Vector4d x;
    Eigen::Matrix4d p;
    Eigen::Matrix<double, 2, 4> h;
    double likelihood;
    double lifetime;
    double age;
    double not_observed_time;
private:
    Eigen::Matrix2d r;
    KalmanFilter kf;
    double last_time;
};

class ObstacleTrackerKF
{
public:
    ObstacleTrackerKF(void);

    std::map<int, Obstacle> obstacles;

    void set_obstacles_pose(const geometry_msgs::PoseArray&);
    void get_velocities(std::vector<Eigen::Vector3d>&);
    void get_poses(std::vector<Eigen::Vector3d>&);

private:
    void associate_obstacles(const std::vector<Eigen::Vector2d>&);
    double get_distance(const Obstacle&, const Eigen::Vector2d&);
    int get_id_from_index(int);
    int get_new_id(void);
    void solve_hungarian_method(Eigen::MatrixXi&);
    void update_tracking(const std::vector<Eigen::Vector2d>&);

    int SAME_OBSTACLE_THRESHOLD;
    double ERASE_LIKELIHOOD_THREHSOLD;
    double NOT_OBSERVED_TIME_THRESHOLD;
    double DEFAULT_LIFE_TIME;
    std::vector<int> candidates;
};

#endif// __OBSTACLE_TRACKER_KF_H
