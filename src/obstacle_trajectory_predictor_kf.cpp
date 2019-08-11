#include "dynamic_obstacle_avoidance_planner/obstacle_trajectory_predictor_kf.h"

ObstacleTrajectoryPredictorKF::ObstacleTrajectoryPredictorKF(void)
:local_nh("~")
{
    local_nh.param("/dynamic_avoidance/PREDICTION_TIME", PREDICTION_TIME, {3.5});
    local_nh.param("/dynamic_avoidance/WORLD_FRAME", WORLD_FRAME, {"map"});
    local_nh.param("/dynamic_avoidance/OBSTACLES_FRAME", OBS_FRAME, {"obs"});
    PREDICTION_STEP = PREDICTION_TIME / DT;

    predicted_paths_pub = nh.advertise<geometry_msgs::PoseArray>("/predicted_paths", 1);
    obs_num_sub = nh.subscribe("/obs_num", 1, &ObstacleTrajectoryPredictorKF::obs_num_callback, this);
    first_transform = true;
    second_transform = true;
    predicted_paths.header.frame_id = WORLD_FRAME;
    std::cout << "=== obstacle_trajectory_predictor_kf ===" << std::endl;
}

void ObstacleTrajectoryPredictorKF::process(void)
{
    ros::spin();
}

void ObstacleTrajectoryPredictorKF::obs_num_callback(const std_msgs::Int32ConstPtr& msg)
{
    std::cout << "=== obstacle_trajectory_predictor_kf ===" << std::endl;
    NUM = msg->data;
    if(NUM > 0){
        ros::Time start_time = ros::Time::now();
        bool transformed = false;
        try{
            for(int i=0;i<NUM;i++){
                std::string frame = OBS_FRAME + std::to_string(i);
                listener.lookupTransform(WORLD_FRAME, frame, ros::Time(0), _transform);
                std::cout << OBS_FRAME + std::to_string(i) + " received" << std::endl;
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
            std::cout << "=== first ===" << std::endl;
            double current_time = ros::Time::now().toSec();
            last_time = current_time;
            first_transform = false;
        }else if(transformed && !first_transform && second_transform){
            std::cout << "=== second ===" << std::endl;
            // 適当
            double current_time = ros::Time::now().toSec();
            double dt = current_time - last_time;
            last_time = current_time;
            for(int i=0;i<NUM;i++){
                double _vx = (current_poses.poses[i].position.x - previous_poses.poses[i].position.x) / dt;
                double _vy = (current_poses.poses[i].position.y - previous_poses.poses[i].position.y) / dt;
                // 進行方向
                //double direction = atan2(_vy, _vx);
                //current_poses.poses[i].orientation = tf::createQuaternionMsgFromYaw(direction);
                KalmanFilter _kf(current_poses.poses[i].position.x, current_poses.poses[i].position.y, tf::getYaw(current_poses.poses[i].orientation));
                kf.push_back(_kf);
            }
            second_transform = false;
        }else if(transformed && !first_transform && !second_transform){
            std::cout << "===predict path===" << std::endl;
            // kalman filterの結果の速度を積分
            predicted_paths.poses.clear();
            for(int i=0;i<NUM;i++){
                // 適当
                double current_time = ros::Time::now().toSec();
                double dt = current_time - last_time;
                last_time = current_time;
                double _vx = (current_poses.poses[i].position.x - previous_poses.poses[i].position.x) / dt;
                double _vy = (current_poses.poses[i].position.y - previous_poses.poses[i].position.y) / dt;
                // 進行方向
                //double direction = atan2(_vy, _vx);
                //current_poses.poses[i].orientation = tf::createQuaternionMsgFromYaw(direction);
                std::cout << current_poses.poses[i].position.x << ", " << current_poses.poses[i].position.y << ", " << tf::getYaw(current_poses.poses[i].orientation)  << std::endl;
                predicted_paths.poses.push_back(current_poses.poses[i]);
                kf[i].predict();
                kf[i].update(current_poses.poses[i].position.x, current_poses.poses[i].position.y, tf::getYaw(current_poses.poses[i].orientation));
                Eigen::Vector3d velocity;
                velocity = kf[i].get_velocity();
                std::cout << "predicted velocity" << std::endl;
                std::cout << velocity << std::endl;
                double v = velocity[0];
                double omega = velocity[1];
                double yaw = tf::getYaw(current_poses.poses[i].orientation);
                yaw = atan2(sin(yaw), cos(yaw));
                double vx = v * cos(yaw);
                double vy = v * sin(yaw);
                vx = velocity[0];
                vy = velocity[1];
                v = sqrt(vx*vx+vy+vy);
                omega = velocity[2];

                for(int j=0;j<PREDICTION_STEP;j++){
                    geometry_msgs::Pose pose;
                    pose.position.x = predicted_paths.poses[i*(PREDICTION_STEP+1)+j].position.x + vx * DT;
                    pose.position.y = predicted_paths.poses[i*(PREDICTION_STEP+1)+j].position.y + vy * DT;
                    yaw += omega * DT;
                    yaw = atan2(sin(yaw), cos(yaw));
                    pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
                    //vx = v * cos(yaw);
                    //vy = v * sin(yaw);
                    v = sqrt(vx*vx + vy*vy);
                    omega = omega;
                    predicted_paths.poses.push_back(pose);
                }
            }
            std::cout << predicted_paths.poses.size() << std::endl;
            predicted_paths_pub.publish(predicted_paths);
        }
        previous_poses = current_poses;
        current_poses.poses.clear();
        std::cout << ros::Time::now() - start_time << "[s]" << std::endl;
    }else{
        std::cout << "no obstacle" << std::endl;
    }
}

ObstacleTrajectoryPredictorKF::KalmanFilter::KalmanFilter(double x, double y, double yaw)
{
    X << x, y, yaw, 0, 0, 0;

    Z << x, y, yaw;

    P <<     0.0,  0.0, 0.0,  0.0,  0.0,  0.0,
             0.0,  0.0, 0.0,  0.0,  0.0,  0.0,
             0.0,  0.0, 0.0,  0.0,  0.0,  0.0,
             0.0,  0.0, 0.0, 1e+0,  0.0,  0.0,
             0.0,  0.0, 0.0,  0.0, 1e+0,  0.0,
             0.0,  0.0, 0.0,  0.0,  0.0, 1e+0;

    R << 1e-3,  0.0,  0.0,
          0.0, 1e-3,  0.0,
          0.0,  0.0, 1e-3;

    H << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
         0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
         0.0, 0.0, 1.0, 0.0, 0.0, 0.0;

    I << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
         0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
         0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
         0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
         0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
         0.0, 0.0, 0.0, 0.0, 0.0, 1.0;

    last_time = ros::Time::now().toSec();
}

void ObstacleTrajectoryPredictorKF::KalmanFilter::set_interval(double dt)
{
    // vx, vy, omega
    G << dt*dt / 2.0,         0.0,         0.0,
                 0.0, dt*dt / 2.0,         0.0,
                 0.0,         0.0, dt*dt / 2.0,
                  dt,         0.0,         0.0,
                 0.0,          dt,         0.0,
                 0.0,         0.0,          dt;

    Q = SIGMA_A * SIGMA_A * G * G.transpose();
}

void ObstacleTrajectoryPredictorKF::KalmanFilter::update(double x, double y, double yaw)
{
    Z << x, y, yaw;
    std::cout << "Z" << std::endl;
    std::cout << Z << std::endl;
    std::cout << "HX" << std::endl;
    std::cout << H*X << std::endl;
    Eigen::Vector3d e = Z - H * X;
    e[2] = atan2(sin(e[2]), cos(e[2]));
    std::cout << "e" << std::endl;
    std::cout << e << std::endl;
    Eigen::Matrix3d S = H * P * H.transpose() + R;
    //std::cout << "S" << std::endl;
    //std::cout << S << std::endl;
    Eigen::Matrix<double, 6, 3> K = P * H.transpose() * S.inverse();
    //std::cout << "K" << std::endl;
    //std::cout << K << std::endl;

    X[2] = atan2(sin(X[2]), cos(X[2]));
    X = X + K * e;
    std::cout << "Ke" << std::endl;
    std::cout << K*e << std::endl;
    X[2] = atan2(sin(X[2]), cos(X[2]));
    //std::cout << "X" << std::endl;
    //std::cout << X << std::endl;
    P = (I - K * H) * P;
    //std::cout << "P" << std::endl;
    //std::cout << P << std::endl;
}

void ObstacleTrajectoryPredictorKF::KalmanFilter::predict(void)
{
    double current_time = ros::Time::now().toSec();
    double dt = current_time - last_time;
    last_time = current_time;

    set_interval(dt);

    F << 1.0, 0.0, 0.0,  dt, 0.0, 0.0,
         0.0, 1.0, 0.0, 0.0,  dt, 0.0,
         0.0, 0.0, 1.0, 0.0, 0.0,  dt,
         0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
         0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
         0.0, 0.0, 0.0, 0.0, 0.0, 1.0;

    X = F * X;
    X[2] = atan2(sin(X[2]), cos(X[2]));
    //std::cout << "X" << std::endl;
    //std::cout << X << std::endl;
    P = F * P * F.transpose() + Q;
    //std::cout << "P" << std::endl;
    //std::cout << P << std::endl;
}

Eigen::Vector3d ObstacleTrajectoryPredictorKF::KalmanFilter::get_velocity(void)
{
    Eigen::Vector3d velocity;
    velocity << X[3], X[4], X[5];
    return velocity;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "obstacle_trajectory_preditor_kf");
    ObstacleTrajectoryPredictorKF otpkf;
    otpkf.process();
    return 0;
}
