//ros
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseArray.h>

//ipopt
#include <Eigen/Core>
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>

// kinematic model robot_velocity = forward_matrix * wheel_velocity
Eigen::MatrixXd forward_matrix;
Eigen::VectorXd wheel_velocity;
Eigen::Vector3d robot_velocity;

using CppAD::AD;

class MPC{
public:
  MPC();

  // state, ref_x, ref_y, ref_yaw
  std::vector<double> solve(Eigen::VectorXd, Eigen::VectorXd, Eigen::VectorXd, Eigen::VectorXd);

};

class FG_eval{
public:
  FG_eval(Eigen::VectorXd, Eigen::VectorXd, Eigen::VectorXd);

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;

  void operator()(ADvector&, const ADvector&);

private:
  Eigen::VectorXd ref_x;
  Eigen::VectorXd ref_y;
  Eigen::VectorXd ref_yaw;

};

class MPCPathTracker
{
public:
  MPCPathTracker(void);

  void path_callback(const nav_msgs::PathConstPtr&);

  void process(void);
  void path_to_vector(void);

private:
  ros::NodeHandle nh;
  ros::Publisher velocity_pub;
  ros::Publisher path_pub;
  ros::Subscriber path_sub;
  MPC mpc;
  nav_msgs::Path path;
  tf::TransformListener listener;
  geometry_msgs::PoseStamped current_pose;
  geometry_msgs::PoseStamped previous_pose;
  tf::StampedTransform _transform;
  geometry_msgs::TransformStamped transform;
  Eigen::VectorXd path_x;
  Eigen::VectorXd path_y;
  Eigen::VectorXd path_yaw;
  bool first_transform = true;
  double last_time;

};

// ホライゾン長さ
int T = 15;
// 周期
double DT;// [s]
const double HZ = 10;
// 目標速度
double VREF;// [m/s]
// 最大角速度
double MAX_ANGULAR_VELOCITY;// [rad/s]
// ホイール角加速度
double MAX_WHEEL_ANGULAR_ACCELERATION;// [rad/s^2]
// ホイール角速度
double MAX_WHEEL_ANGULAR_VELOCITY;// [rad/s]
// ホイール半径
double WHEEL_RADIUS;// [m]
// トレッド
double TREAD;// [m]
// ホイールベース
double WHEEL_BASE;// [m]
// グリッドマップ分解能
double RESOLUTION;// [m]
// ロボットの足まわり半径
double ROBOT_RADIUS;
// 足回り配置
double ROBOT_THETA;
// ステア角度制限
double MAX_STEERING_ANGLE;// [rad]

std::string WORLD_FRAME;
std::string ROBOT_FRAME;
std::string VELOCITY_TOPIC_NAME;
std::string INTERMEDIATE_PATH_TOPIC_NAME;

// state
size_t x_start = 0;
size_t y_start = x_start + T;
size_t yaw_start = y_start + T;
size_t vx_start = yaw_start + T;
size_t vy_start = vx_start + T;
size_t omega_start = vy_start + T;
size_t omega_w_fr_start = omega_start + T;
size_t omega_w_fl_start = omega_w_fr_start + T;
size_t omega_w_rr_start = omega_w_fl_start + T;
size_t omega_w_rl_start = omega_w_rr_start + T;
size_t theta_s_fr_start = omega_w_rl_start + T;
size_t theta_s_fl_start = theta_s_fr_start + T;
size_t theta_s_rr_start = theta_s_fl_start + T;
size_t theta_s_rl_start = theta_s_rr_start + T;
// input
size_t domega_w_fr_start = theta_s_rl_start + T;
size_t domega_w_fl_start = domega_w_fr_start + T - 1;
size_t domega_w_rr_start = domega_w_fl_start + T - 1;
size_t domega_w_rl_start = domega_w_rr_start + T - 1;
size_t dtheta_s_fr_start = domega_w_rl_start + T - 1;
size_t dtheta_s_fl_start = dtheta_s_fr_start + T - 1;
size_t dtheta_s_rr_start = dtheta_s_fl_start + T - 1;
size_t dtheta_s_rl_start = dtheta_s_rr_start + T - 1;

// 最適化失敗時は最後の成功データを使う
int failure_count = 0;
std::vector<double> result;

double min_distance(nav_msgs::Path&, geometry_msgs::PoseStamped&);
double get_distance(geometry_msgs::PoseStamped&, geometry_msgs::PoseStamped&);

int main(int argc, char** argv)
{
  ros::init(argc, argv, "diff_drive_mpc");
  ros::NodeHandle local_nh("~");

  local_nh.getParam("HORIZON_T", T);
  local_nh.getParam("VREF", VREF);
  local_nh.getParam("/dynamic_avoidance/MAX_ANGULAR_VELOCITY", MAX_ANGULAR_VELOCITY);
  local_nh.getParam("/dynamic_avoidance/RESOLUTION", RESOLUTION);
  local_nh.getParam("/dynamic_avoidance/ROBOT_FRAME", ROBOT_FRAME);
  local_nh.getParam("/dynamic_avoidance/WORLD_FRAME", WORLD_FRAME);
  local_nh.getParam("/dynamic_avoidance/VELOCITY_TOPIC_NAME", VELOCITY_TOPIC_NAME);
  local_nh.getParam("/dynamic_avoidance/INTERMEDIATE_PATH_TOPIC_NAME", INTERMEDIATE_PATH_TOPIC_NAME);
  local_nh.getParam("/fwdis/WHEEL_RADIUS", WHEEL_RADIUS);
  local_nh.getParam("/fwdis/WHEEL_BASE", WHEEL_BASE);
  local_nh.getParam("/fwdis/TREAD", TREAD);
  local_nh.getParam("/fwdis/MAX_WHEEL_ANGULAR_VELOCITY", MAX_WHEEL_ANGULAR_VELOCITY);
  local_nh.getParam("/fwdis/MAX_WHEEL_ANGULAR_ACCELERATION", MAX_WHEEL_ANGULAR_ACCELERATION);

  ROBOT_RADIUS = sqrt(pow(WHEEL_BASE, 2) + pow(TREAD, 2)) / 2.0;
  ROBOT_THETA = atan(TREAD / WHEEL_BASE);
  forward_matrix.resize(8, 3);
  forward_matrix << 1.0, 0.0,  ROBOT_RADIUS * sin(ROBOT_THETA),
                    0.0, 1.0,  ROBOT_RADIUS * cos(ROBOT_THETA),
                    1.0, 0.0, -ROBOT_RADIUS * sin(ROBOT_THETA),
                    0.0, 1.0,  ROBOT_RADIUS * cos(ROBOT_THETA),
                    1.0, 0.0, -ROBOT_RADIUS * sin(ROBOT_THETA),
                    0.0, 1.0, -ROBOT_RADIUS * cos(ROBOT_THETA),
                    1.0, 0.0,  ROBOT_RADIUS * sin(ROBOT_THETA),
                    0.0, 1.0, -ROBOT_RADIUS * cos(ROBOT_THETA);

  wheel_velocity.resize(8, 1);

  MPCPathTracker mpc_path_tracker;

  ros::Rate loop_rate(HZ);

  while(ros::ok()){
    mpc_path_tracker.process();

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}

MPC::MPC(){}

std::vector<double> MPC::solve(Eigen::VectorXd state, Eigen::VectorXd ref_x, Eigen::VectorXd ref_y, Eigen::VectorXd ref_yaw)
{
  /*
   * state:x, y, yaw, vx, vy, omega, omega_w_fr, omega_w_fl, omega_w_rr, omega_w_rl, theta_s_fr, theta_s_fl, theta_s_rr, theta_s_rl
   */
  bool ok = true;
  size_t i;
  typedef CPPAD_TESTVECTOR(double) Dvector;

  double x = state[i++];
  double y = state[i++];
  double yaw = state[i++];
  double vx = state[i++];
  double vy = state[i++];
  double omega = state[i++];
  double omega_w_fr = state[i++];
  double omega_w_fl = state[i++];
  double omega_w_rr = state[i++];
  double omega_w_rl = state[i++];
  double theta_s_fr = state[i++];
  double theta_s_fl = state[i++];
  double theta_s_rr = state[i++];
  double theta_s_rl = state[i++];
  i=0;

  /*
  std::cout << "--- state ---" << std::endl;
  std::cout << state << std::endl;
  std::cout << "--- path_x ---" << std::endl;
  std::cout << ref_x << std::endl;
  std::cout << "--- path_y ---" << std::endl;
  std::cout << ref_y << std::endl;
  std::cout << "--- path_yaw ---" << std::endl;
  std::cout << ref_yaw << std::endl;
  */

  // 14(state), 8(input)
  size_t n_variables = 14 * T + 8 * (T - 1);

  size_t n_constraints = 14 * T;

  Dvector vars(n_variables);
  for(int i=0;i<n_variables;i++){
    vars[i] = 0.0;
  }

  vars[x_start] = x;
  vars[y_start] = y;
  vars[yaw_start] = yaw;
  vars[vx_start] = vx;
  vars[vy_start] = vy;
  vars[omega_start] = omega;
  vars[omega_w_fr_start] = omega_w_fr;
  vars[omega_w_fl_start] = omega_w_fl;
  vars[omega_w_rr_start] = omega_w_rr;
  vars[omega_w_rl_start] = omega_w_rl;
  vars[theta_s_fr_start] = theta_s_fr;
  vars[theta_s_fl_start] = theta_s_fl;
  vars[theta_s_rr_start] = theta_s_rr;
  vars[theta_s_rl_start] = theta_s_rl;

  Dvector vars_lower_bound(n_variables);
  Dvector vars_upper_bound(n_variables);

  for(int i=0;i<vx_start;i++){
    // x, y, yaw
    vars_lower_bound[i] = -1.0e19;
    vars_upper_bound[i] = 1.0e19;
  }
  for(int i=vx_start;i<omega_start;i++){
    // vx, vy
    vars_lower_bound[i] = 0;
    vars_upper_bound[i] = VREF;
  }
  for(int i=omega_start;i<omega_w_fr_start;i++){
    // omega
    vars_lower_bound[i] = -MAX_ANGULAR_VELOCITY;
    vars_lower_bound[i] = MAX_ANGULAR_VELOCITY;
  }
  for(int i=omega_w_fr_start;i<theta_s_fr_start;i++){
    vars_lower_bound[i] = -MAX_WHEEL_ANGULAR_VELOCITY;
    vars_upper_bound[i] = MAX_WHEEL_ANGULAR_VELOCITY;
  }
  for(int i=theta_s_fr_start;i<domega_w_fr_start;i++){
    vars_lower_bound[i] = -MAX_STEERING_ANGLE;
    vars_upper_bound[i] = MAX_STEERING_ANGLE;
  }
  for(int i=domega_w_fr_start;i<dtheta_s_fr_start;i++){
    vars_lower_bound[i] = -MAX_WHEEL_ANGULAR_ACCELERATION;
    vars_upper_bound[i] = MAX_WHEEL_ANGULAR_ACCELERATION;
  }
  for(int i=dtheta_s_fr_start;i<n_variables;i++){
    // 適当
    vars_lower_bound[i] = -MAX_WHEEL_ANGULAR_ACCELERATION;
    vars_upper_bound[i] = MAX_WHEEL_ANGULAR_ACCELERATION;
  }

  // 等式制約
  Dvector constraints_lower_bound(n_constraints);
  Dvector constraints_upper_bound(n_constraints);

  for(int i=0;i<n_constraints;i++){
    constraints_lower_bound[i] = 0.0;
    constraints_upper_bound[i] = 0.0;
  }

  // t=0の設定
  constraints_lower_bound[x_start] = x;
  constraints_lower_bound[y_start] = y;
  constraints_lower_bound[yaw_start] = yaw;
  constraints_lower_bound[vx_start] = vx;
  constraints_lower_bound[vy_start] = vy;
  constraints_lower_bound[omega_start] = omega;
  constraints_lower_bound[omega_w_fr_start] = omega_w_fr;
  constraints_lower_bound[omega_w_fl_start] = omega_w_fl;
  constraints_lower_bound[omega_w_rr_start] = omega_w_rr;
  constraints_lower_bound[omega_w_rl_start] = omega_w_rl;
  constraints_lower_bound[theta_s_fr_start] = theta_s_fr;
  constraints_lower_bound[theta_s_fl_start] = theta_s_fl;
  constraints_lower_bound[theta_s_rr_start] = theta_s_rr;
  constraints_lower_bound[theta_s_rl_start] = theta_s_rl;

  constraints_upper_bound[x_start] = x;
  constraints_upper_bound[y_start] = y;
  constraints_upper_bound[yaw_start] = yaw;
  constraints_upper_bound[vx_start] = vx;
  constraints_upper_bound[vy_start] = vy;
  constraints_upper_bound[omega_start] = omega;
  constraints_upper_bound[omega_w_fr_start] = omega_w_fr;
  constraints_upper_bound[omega_w_fl_start] = omega_w_fl;
  constraints_upper_bound[omega_w_rr_start] = omega_w_rr;
  constraints_upper_bound[omega_w_rl_start] = omega_w_rl;
  constraints_upper_bound[theta_s_fr_start] = theta_s_fr;
  constraints_upper_bound[theta_s_fl_start] = theta_s_fl;
  constraints_upper_bound[theta_s_rr_start] = theta_s_rr;
  constraints_upper_bound[theta_s_rl_start] = theta_s_rl;

  FG_eval fg_eval(ref_x, ref_y, ref_yaw);

  std::string options;
  options += "Integer print_level  0\n";

  options += "Sparse  true        forward\n";
  options += "Sparse  true        reverse\n";

  options += "Numeric max_cpu_time          0.5\n";

  CppAD::ipopt::solve_result<Dvector> solution;

  std::cout << "optimization start" << std::endl;
  CppAD::ipopt::solve<Dvector, FG_eval>(
      options, vars, vars_lower_bound, vars_upper_bound, constraints_lower_bound,
      constraints_upper_bound, fg_eval, solution);

  std::cout << "optimization end" << std::endl;
  ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;
  std::cout << solution.status << std::endl;
  std::cout << ok << std::endl;

  auto cost = solution.obj_value;
  std::cout << "Cost " << cost << std::endl;

  if(ok){
    failure_count = 0;
    result.clear();
    // 何故か0だとうまく行かない
    result.push_back(solution.x[vx_start+1]);
    result.push_back(solution.x[vy_start+1]);
    result.push_back(solution.x[omega_start+1]);
    //予測軌道
    for(int i = 0; i < T-1; i++){
      result.push_back(solution.x[x_start+i+1]);
      result.push_back(solution.x[y_start+i+1]);
      result.push_back(solution.x[yaw_start+i+1]);
    }
  }else{
    if(failure_count < T - 1){
      failure_count++;
    }
    result.push_back(solution.x[vx_start+1+failure_count]);
    result.push_back(solution.x[vy_start+1+failure_count]);
    result.push_back(solution.x[omega_start+1+failure_count]);
    //予測軌道
    for(int i = failure_count; i < T-1; i++){
      result.push_back(solution.x[x_start+i+1]);
      result.push_back(solution.x[y_start+i+1]);
      result.push_back(solution.x[yaw_start+i+1]);
    }
  }
  /*
  std::cout << "--- result ---" << std::endl;
  for(int i=0;i<result.size();i++){
    std::cout << result[i] << std::endl;
  }
  */
  return result;
}

FG_eval::FG_eval(Eigen::VectorXd ref_x, Eigen::VectorXd ref_y, Eigen::VectorXd ref_yaw)
{
  this->ref_x = ref_x;
  this->ref_y = ref_y;
  this->ref_yaw = ref_yaw;
}

void FG_eval::operator()(ADvector& fg, const ADvector& vars)
{
  std::cout << "FG_eval() start" << std::endl;
  // cost
  fg[0] = 0;
  // state
  for(int i=0;i<T-1;i++){
    // pathとの距離
    fg[0] += 0.2 * (CppAD::pow(vars[x_start + i] - ref_x[i], 2) + CppAD::pow(vars[y_start + i] - ref_y[i], 2));
    // 向き
    //fg[0] += 0.1 * CppAD::pow(vars[yaw_start + i] - ref_yaw[i], 2);
  }
  // input
  for(int i=0;i<T-2;i++){
    // 速度
    fg[0] += 0.1 * CppAD::pow(VREF - vars[vx_start + i], 2);
    // 角加速度
    fg[0] += 0.1 * CppAD::pow(vars[omega_start + i] - vars[omega_start + i+ 1], 2);
  }

  std::cout << "constrains start" << std::endl;
  //constraint
  //初期状態
  fg[1 + x_start] = vars[x_start];
  fg[1 + y_start] = vars[y_start];
  fg[1 + yaw_start] = vars[yaw_start];
  fg[1 + vx_start] = vars[vx_start];
  fg[1 + vy_start] = vars[vy_start];
  fg[1 + omega_start] = vars[omega_start];
  fg[1 + omega_w_fr_start] = vars[omega_w_fr_start];
  fg[1 + omega_w_fl_start] = vars[omega_w_fl_start];
  fg[1 + omega_w_rr_start] = vars[omega_w_rr_start];
  fg[1 + omega_w_rl_start] = vars[omega_w_rl_start];
  fg[1 + theta_s_fr_start] = vars[theta_s_fr_start];
  fg[1 + theta_s_fl_start] = vars[theta_s_fl_start];
  fg[1 + theta_s_rr_start] = vars[theta_s_rr_start];
  fg[1 + theta_s_rl_start] = vars[theta_s_rl_start];

  std::cout << "constraints loop start" << std::endl;

  for(int i=0;i<T-1;i++){
    //t+1
    AD<double> x1 = vars[x_start + i + 1];
    AD<double> y1 = vars[y_start + i + 1];
    AD<double> yaw1 = vars[yaw_start + i + 1];
    AD<double> vx1 = vars[vx_start + i + 1];
    AD<double> vy1 = vars[vy_start + i + 1];
    AD<double> omega1 = vars[omega_start + i + 1];
    AD<double> omega_w_fr1 = vars[omega_w_fr_start + i + 1];
    AD<double> omega_w_fl1 = vars[omega_w_fl_start + i + 1];
    AD<double> omega_w_rr1 = vars[omega_w_rr_start + i + 1];
    AD<double> omega_w_rl1 = vars[omega_w_rl_start + i + 1];
    AD<double> theta_s_fr1 = vars[theta_s_fr_start + i + 1];
    AD<double> theta_s_fl1 = vars[theta_s_fl_start + i + 1];
    AD<double> theta_s_rr1 = vars[theta_s_rr_start + i + 1];
    AD<double> theta_s_rl1 = vars[theta_s_rl_start + i + 1];
    //t
    AD<double> x0 = vars[x_start + i];
    AD<double> y0 = vars[y_start + i];
    AD<double> yaw0 = vars[yaw_start + i];
    AD<double> vx0 = vars[vx_start + i];
    AD<double> vy0 = vars[vy_start + i];
    AD<double> omega0 = vars[omega_start + i];
    AD<double> omega_w_fr0 = vars[omega_w_fr_start + i];
    AD<double> omega_w_fl0 = vars[omega_w_fl_start + i];
    AD<double> omega_w_rr0 = vars[omega_w_rr_start + i];
    AD<double> omega_w_rl0 = vars[omega_w_rl_start + i];
    AD<double> theta_s_fr0 = vars[theta_s_fr_start + i];
    AD<double> theta_s_fl0 = vars[theta_s_fl_start + i];
    AD<double> theta_s_rr0 = vars[theta_s_rr_start + i];
    AD<double> theta_s_rl0 = vars[theta_s_rl_start + i];
    //入力ホライゾンはt+1を考慮しない
    AD<double> domega_w_fr0 = vars[domega_w_fr_start + i];
    AD<double> domega_w_fl0 = vars[domega_w_fl_start + i];
    AD<double> domega_w_rr0 = vars[domega_w_rr_start + i];
    AD<double> domega_w_rl0 = vars[domega_w_rl_start + i];
    AD<double> dtheta_s_fr0 = vars[dtheta_s_fr_start + i];
    AD<double> dtheta_s_fl0 = vars[dtheta_s_fl_start + i];
    AD<double> dtheta_s_rr0 = vars[dtheta_s_rr_start + i];
    AD<double> dtheta_s_rl0 = vars[dtheta_s_rl_start + i];

    //制約
    fg[2 + x_start + i] = x1 - (x0 + vx0 * CppAD::cos(yaw0) * DT);
    fg[2 + y_start + i] = y1 - (y0 + vx0 * CppAD::sin(yaw0) * DT);
    AD<double> sin0 = CppAD::sin(yaw0 + omega0 * DT);
    AD<double> cos0 = CppAD::cos(yaw0 + omega0 * DT);
    //fg[2 + yaw_start + i] = yaw1 - CppAD::atan2(sin0, cos0);
    fg[2 + yaw_start + i] = yaw1 - (yaw0 + omega0 * DT);
  }
  std::cout << "FG_eval() end" << std::endl;
}

MPCPathTracker::MPCPathTracker(void)
{
  velocity_pub = nh.advertise<geometry_msgs::Twist>(VELOCITY_TOPIC_NAME, 100);
  path_pub = nh.advertise<geometry_msgs::PoseArray>("/mpc_path", 100);
  path_sub = nh.subscribe(INTERMEDIATE_PATH_TOPIC_NAME, 100, &MPCPathTracker::path_callback, this);
  path_x = Eigen::VectorXd::Zero(T);
  path_y = Eigen::VectorXd::Zero(T);
  path_yaw = Eigen::VectorXd::Zero(T);
}

void MPCPathTracker::path_callback(const nav_msgs::PathConstPtr& msg)
{
  path = *msg;
}

void MPCPathTracker::process(void)
{
  std::cout << "=== diff drive mpc ===" << std::endl;
  ros::Time start_time = ros::Time::now();
  bool transformed = false;
  geometry_msgs::PoseStamped pose;
  try{
    listener.lookupTransform(WORLD_FRAME, ROBOT_FRAME, ros::Time(0), _transform);
    tf::transformStampedTFToMsg(_transform, transform);
    current_pose.header = transform.header;
    current_pose.pose.position.x = transform.transform.translation.x;
    current_pose.pose.position.y = transform.transform.translation.y;
    current_pose.pose.orientation = transform.transform.rotation;
    pose.header = current_pose.header;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.orientation = transform.transform.rotation;
    transformed = true;
  }catch(tf::TransformException &ex){
    std::cout << ex.what() << std::endl;
  }

  if(!path.poses.empty() && transformed){
    if(first_transform){
      last_time = ros::Time::now().toSec();
      first_transform = false;
    }else{
      //std::cout << current_pose << std::endl;
      double current_time = ros::Time::now().toSec();
      double dt = current_time - last_time;
      last_time = current_time;
      double dx = current_pose.pose.position.x - previous_pose.pose.position.x;
      double dy = current_pose.pose.position.y - previous_pose.pose.position.y;
      double dyaw = tf::getYaw(current_pose.pose.orientation) - tf::getYaw(previous_pose.pose.orientation);
      double v = sqrt(dx * dx + dy * dy) / dt;
      double omega = dyaw / dt;
      double omega_r = (v + omega * TREAD / 2.0) / WHEEL_RADIUS;
      double omega_l = (v - omega * TREAD / 2.0) / WHEEL_RADIUS;

      Eigen::VectorXd state(5);
      state << pose.pose.position.x, pose.pose.position.y, tf::getYaw(pose.pose.orientation), omega_r, omega_l;
      std::cout << "path to vector" << std::endl;
      path_to_vector();
      std::cout << "solving" << std::endl;
      auto result = mpc.solve(state, path_x, path_y, path_yaw);
      std::cout << "solved" << std::endl;
      geometry_msgs::Twist velocity;
      velocity.linear.x = result[0];
      velocity.angular.z = result[1];
      std::cout << velocity << std::endl;
      velocity_pub.publish(velocity);
      // mpc表示
      geometry_msgs::PoseArray mpc_path;
      mpc_path.header.frame_id = ROBOT_FRAME;
      double yaw0 = tf::getYaw(pose.pose.orientation);
      for(int i=0;i<T-1;i++){
        geometry_msgs::Pose temp;
        temp.position.x = result[2+3*i] * cos(-yaw0) - result[3+3*i] * sin(-yaw0);
        temp.position.y = result[2+3*i] * sin(-yaw0) + result[3+3*i] * cos(-yaw0);
        temp.orientation = tf::createQuaternionMsgFromYaw(result[4+3*i] - yaw0);
        mpc_path.poses.push_back(temp);
      }
      path_pub.publish(mpc_path);
      // ~mpc表示
      path.poses.erase(path.poses.begin());
    }
  }
  previous_pose = current_pose;
  std::cout << ros::Time::now() - start_time << "[s]" << std::endl;
}

void MPCPathTracker::path_to_vector(void)
{
  int m = VREF * DT / RESOLUTION + 1;// TODO:delete 1
  int index = 0;
  for(int i=0;i<T;i++){
    if(i*m<path.poses.size()){
      index = i*m;
      path_x[i] = path.poses[index].pose.position.x;
      path_y[i] = path.poses[index].pose.position.y;
      path_yaw[i] = tf::getYaw(path.poses[index].pose.orientation);
    }else{
      path_x[i] = path.poses[path.poses.size() - 1].pose.position.x;
      path_y[i] = path.poses[path.poses.size() - 1].pose.position.y;
      path_yaw[i] = tf::getYaw(path.poses[path.poses.size() - 1].pose.orientation);
    }
  }
}

double min_distance(nav_msgs::Path& path, geometry_msgs::PoseStamped& pose)
{
  int length = path.poses.size();
  double min_distance = 100;
  for(int i=0;i<length;i++){
    double distance = get_distance(path.poses[i], pose);
    if(min_distance > distance){
      min_distance = distance;
    }
  }
  return min_distance;
}

double get_distance(geometry_msgs::PoseStamped& pose0, geometry_msgs::PoseStamped& pose1)
{
  return sqrt((pose0.pose.position.x - pose1.pose.position.x) * (pose0.pose.position.x - pose1.pose.position.x) + (pose0.pose.position.y - pose1.pose.position.y) * (pose0.pose.position.y - pose1.pose.position.y));
}
