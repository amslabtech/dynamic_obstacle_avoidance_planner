#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/Int32.h>

#include <Eigen/Dense>

// kalman filter
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

const double PREDICTION_TIME = 3.5;// [s], 軌道予測時間
const double DT = 0.1;// [s]
const int PREDICTION_STEP = PREDICTION_TIME / DT;
const int NUM_OF_PATH = 2;// obs1つあたりpath2本
const double HZ = 10;

int NUM = 1;

geometry_msgs::PoseArray predicted_paths;
geometry_msgs::PoseArray current_poses;
geometry_msgs::PoseArray previous_poses;

std::vector<KalmanFilter> kf;

tf::StampedTransform _transform;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "obstacle_predictor_kf");
  ros::NodeHandle nh;
  ros::NodeHandle local_nh("~");

  ros::Publisher predicted_paths_pub = nh.advertise<geometry_msgs::PoseArray>("/predicted_paths", 100);

  tf::TransformListener listener;

  bool first_transform = true;

  bool second_transform = true;

  double last_time;

  predicted_paths.header.frame_id = "world";

  ros::Rate loop_rate(HZ);

  while(ros::ok()){
    if(NUM > 0){
      std::cout << "=== obstacle_predictor_kf ===" << std::endl;
      ros::Time start_time = ros::Time::now();
      bool transformed = false;
      try{
        for(int i=0;i<NUM;i++){
          std::string frame = "vicon/obs/obs";
          listener.lookupTransform("world", frame, ros::Time(0), _transform);
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
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}

KalmanFilter::KalmanFilter(double x, double y, double yaw)
{
  X << x, y, yaw, 0, 0, 0;

  Z << x, y, yaw;

  P <<   0.0,   0.0,    0.0,    0.0,    0.0,  0.0,
         0.0,   0.0,    0.0,    0.0,    0.0,  0.0,
         0.0,   0.0,    0.0,    0.0,    0.0,  0.0,
         0.0,   0.0,    0.0,   1e+0,    0.0,  0.0,
         0.0,   0.0,    0.0,    0.0,   1e+0,  0.0,
         0.0,   0.0,    0.0,    0.0,    0.0, 1e+0;

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

void KalmanFilter::set_interval(double dt)
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

void KalmanFilter::update(double x, double y, double yaw)
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

void KalmanFilter::predict(void)
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

  /*
  Eigen::Matrix<double, 5, 5> jF;
  jF << 1.0, 0.0, -X[3] * dt * sin(X[2]), dt * cos(X[2]),            0.0,
        0.0, 1.0,  X[3] * dt * cos(X[2]), dt * sin(X[2]),            0.0,
        0.0, 0.0,                    1.0,            0.0,             dt,
        0.0, 0.0,                    0.0,            1.0,            0.0,
        0.0, 0.0,                    0.0,            0.0,            1.0;
		*/

  X = F * X;
  X[2] = atan2(sin(X[2]), cos(X[2]));
  //std::cout << "X" << std::endl;
  //std::cout << X << std::endl;
  P = F * P * F.transpose() + Q;
  //std::cout << "P" << std::endl;
  //std::cout << P << std::endl;
}

Eigen::Vector3d KalmanFilter::get_velocity(void)
{
  Eigen::Vector3d velocity;
  velocity << X[3], X[4], X[5];
  return velocity;
}
