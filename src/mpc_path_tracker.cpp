#include <ros/ros.h>

#include <Eigen/Core>
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>

using CppAD::AD;

class MPC{
public:
  MPC();

  std::vector<double> solve(Eigen::VectorXd, Eigen::VectorXd);
};

class FG_eval{
public:
  FG_eval(Eigen::VectorXd);

  Eigen::VectorXd coeffs;

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;

  void operator()(ADvector&, const ADvector&);

};

//ホライゾン長さ
const int T = 10;
//周期
const double DT = 0.1;

//simulation parameters
size_t x_start = 0;
size_t y_start = x_start + T;
size_t theta_start = y_start + T;
size_t vx_start = theta_start + T;
size_t vy_start = vx_start + T;
size_t omega_start = vy_start + T - 1;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "mpc_path_tracker");
  ros::NodeHandle nh;
  ros::NodeHandle local_nh("~");

  ros::Rate loop_rate(10);

  while(ros::ok()){

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}

MPC::MPC(){}

std::vector<double> MPC::solve(Eigen::VectorXd state, Eigen::VectorXd coeffs)
{

}

FG_eval::FG_eval(Eigen::VectorXd coeffs)
{
  this->coeffs = coeffs;
}

void FG_eval::operator()(ADvector& fg, const ADvector& vars)
{
  //cost
  fg[0] = 0;
  for(int i=0;i<T-1;i++){

  }
  //constraint
  //初期状態
  fg[1 + x_start] = vars[x_start];
  fg[1 + y_start] = vars[y_start];
  fg[1 + theta_start] = vars[theta_start];
  fg[1 + vx_start] = vars[vx_start];
  fg[1 + vy_start] = vars[vy_start];
  fg[1 + omega_start] = vars[omega_start];

}
