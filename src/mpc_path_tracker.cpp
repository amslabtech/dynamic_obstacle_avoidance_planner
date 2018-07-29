//ros
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>

//ipopt
#include <Eigen/Core>
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>

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
  ros::Subscriber path_sub;
  MPC mpc;
  nav_msgs::Path path;
  tf::TransformListener listener;
  geometry_msgs::PoseStamped pose;
  tf::StampedTransform _transform;
  geometry_msgs::TransformStamped transform;
  Eigen::VectorXd path_x;
  Eigen::VectorXd path_y;
  Eigen::VectorXd path_yaw;

};

// ホライゾン長さ
const int T = 10;
// 周期
const double DT = 0.1;// [s]
// 目標速度
const double VREF = 3.0;// [m/s]

// state
size_t x_start = 0;
size_t y_start = x_start + T;
size_t yaw_start = y_start + T;
size_t cte_start = yaw_start + T;
size_t e_theta_start = cte_start + T;
// input
size_t vx_start = e_theta_start + T;
size_t vy_start = vx_start + T;
size_t omega_start = vy_start + T - 1;

double min_distance(nav_msgs::Path&, geometry_msgs::PoseStamped&);
double get_distance(geometry_msgs::PoseStamped&, geometry_msgs::PoseStamped&);

int main(int argc, char** argv)
{
  ros::init(argc, argv, "mpc_path_tracker");
  ros::NodeHandle local_nh("~");

  MPCPathTracker mpc_path_tracker;

  ros::Rate loop_rate(10);

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
   * state:x, y, yaw, cte, e_theta
   */
  bool ok = true;
  size_t i;
  typedef CPPAD_TESTVECTOR(double) Dvector;

  double x = state[0];
  double y = state[1];
  double yaw = state[2];
  double cte = state[3];
  double e_theta = state[4];

  // 5(x, y, yaw, cte, e_theta), 3(vx, vy, omega)
  size_t n_variables = 5 * T + 3 * (T - 1);

  size_t n_constraints = 5 * T;

  Dvector vars(n_variables);
  for(int i=0;i<n_variables;i++){
    vars[i] = 0.0;
  }

  vars[x_start] = x;
  vars[y_start] = y;
  vars[yaw_start] = yaw;
  vars[cte_start] = cte;
  vars[e_theta_start] = e_theta;

  Dvector vars_lower_bound(n_variables);
  Dvector vars_upper_bound(n_variables);

  for(int i=0;i<vx_start;i++){
    //x, y, yaw, cte, e_theta
    vars_lower_bound[i] = -1.0e19;
    vars_upper_bound[i] = 1.0e19;
  }
  for(int i=vx_start;i<vy_start;i++){
    vars_lower_bound[i] = 3.0;
    vars_upper_bound[i] = -3.0;
  }
  for(int i=vy_start;i<omega_start;i++){
    vars_lower_bound[i] = 3.0;
    vars_upper_bound[i] = -3.0;
  }
  for(int i=omega_start;i<n_variables;i++){
    vars_lower_bound[i] = 5.0;
    vars_upper_bound[i] = -5.0;
  }

  //等式制約
  Dvector constraints_lower_bound(n_constraints);
  Dvector constraints_upper_bound(n_constraints);

  for(int i=0;i<n_constraints;i++){
    constraints_lower_bound[i] = 0.0;
    constraints_upper_bound[i] = 0.0;
  }

  //t=0の設定
  constraints_lower_bound[x_start] = x;
  constraints_lower_bound[y_start] = y;
  constraints_lower_bound[yaw_start] = yaw;
  constraints_lower_bound[cte_start] = cte;
  constraints_lower_bound[e_theta_start] = e_theta;

  constraints_upper_bound[x_start] = x;
  constraints_upper_bound[y_start] = y;
  constraints_upper_bound[yaw_start] = yaw;
  constraints_upper_bound[cte_start] = cte;
  constraints_upper_bound[e_theta_start] = e_theta;

  FG_eval fg_eval(ref_x, ref_y, ref_yaw);

  std::string options;
  options += "Integer print_level  0\n";

  options += "Sparse  true        forward\n";
  options += "Sparse  true        reverse\n";

  options += "Numeric max_cpu_time          0.5\n";

  CppAD::ipopt::solve_result<Dvector> solution;

  CppAD::ipopt::solve<Dvector, FG_eval>(
      options, vars, vars_lower_bound, vars_upper_bound, constraints_lower_bound,
      constraints_upper_bound, fg_eval, solution);

  ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

  auto cost = solution.obj_value;
  std::cout << "Cost " << cost << std::endl;

  std::vector<double> result;
  result.push_back(solution.x[vx_start]);
  result.push_back(solution.x[vy_start]);
  result.push_back(solution.x[omega_start]);
  //予測軌道
  for(int i = 0; i < T-1; i++){
    result.push_back(solution.x[x_start+i+1]);
    result.push_back(solution.x[y_start+i+1]);
    result.push_back(solution.x[yaw_start+i+1]);
  }
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
  // cost
  fg[0] = 0;
  // state
  for(int i=0;i<T-1;i++){
    fg[0] += CppAD::pow(vars[e_theta_start + i], 2);
    fg[0] += CppAD::pow(vars[cte_start + i], 2);
  }
  // input
  for(int i=0;i<T-2;i++){
    fg[0] += CppAD::pow(VREF - CppAD::sqrt(CppAD::pow(vars[vx_start + i], 2) + CppAD::pow(vars[vy_start + i], 2)), 2);
  }

  //constraint
  //初期状態
  fg[1 + x_start] = vars[x_start];
  fg[1 + y_start] = vars[y_start];
  fg[1 + yaw_start] = vars[yaw_start];
  fg[1 + vx_start] = vars[vx_start];
  fg[1 + vy_start] = vars[vy_start];
  fg[1 + omega_start] = vars[omega_start];

  for(int i=0;i<T-1;i++){
    //t+1
    AD<double> x1 = vars[x_start + i + 1];
    AD<double> y1 = vars[y_start + i + 1];
    AD<double> yaw1 = vars[yaw_start + i + 1];
    //t
    AD<double> x0 = vars[x_start + i];
    AD<double> y0 = vars[y_start + i];
    AD<double> yaw0 = vars[yaw_start + i];
    //入力ホライゾンはt+1を考慮しない
    AD<double> vx0 = vars[vx_start + i];
    AD<double> vy0 = vars[vy_start + i];
    AD<double> omega0 = vars[omega_start + i];

    AD<double> v0 = CppAD::sqrt(vx0 * vx0 + vy0 * vy0);
    AD<double> psi0 = CppAD::atan2(vy0, vx0);

    //制約
    fg[2 + x_start + i] = x1 - (x0 + vx0 * CppAD::cos(yaw0) * DT - vy0 * CppAD::sin(yaw0) * DT);
    fg[2 + y_start + i] = y1 - (y0 + vx0 * CppAD::sin(yaw0) * DT + vy0 * CppAD::cos(yaw0) * DT);
    fg[2 + yaw_start + i] = CppAD::atan2(CppAD::sin(yaw0 + omega0 * DT), CppAD::cos(yaw0 + omega0 * DT));
  }
}

MPCPathTracker::MPCPathTracker(void)
{
  velocity_pub = nh.advertise<geometry_msgs::Twist>("/velocity", 100);
  path_sub = nh.subscribe("/path", 100, &MPCPathTracker::path_callback, this);
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
  bool transformed = false;
  try{
    listener.lookupTransform("odom", "base_link", ros::Time(0), _transform);
    tf::transformStampedTFToMsg(_transform, transform);
    pose.header = transform.header;
    pose.pose.position.x = transform.transform.translation.x;
    pose.pose.position.y = transform.transform.translation.y;
    pose.pose.position.z = transform.transform.translation.z;
    pose.pose.orientation = transform.transform.rotation;
    transformed = true;
  }catch(tf::TransformException &ex){
    std::cout << ex.what() << std::endl;
  }

  if(!path.poses.empty() && transformed){
    Eigen::VectorXd state;
    state << pose.pose.position.x, pose.pose.position.y, tf::getYaw(pose.pose.orientation);
    path_to_vector();
    auto result = mpc.solve(state, path_x, path_y, path_yaw);
    geometry_msgs::Twist velocity;
    velocity.linear.x = result[0];
    velocity.linear.y = result[1];
    velocity.angular.z = result[2];
    velocity_pub.publish(velocity);

  }
}

void MPCPathTracker::path_to_vector(void)
{
  for(int i=0;i<T;i++){
    if(i*4<T){
      path_x[i] = path.poses[i*4].pose.position.x;
      path_y[i] = path.poses[i*4].pose.position.y;
      path_yaw[i] = tf::getYaw(path.poses[i*4].pose.orientation);
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
