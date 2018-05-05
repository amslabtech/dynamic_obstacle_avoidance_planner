#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>

geometry_msgs::Twist velocity;
bool velocity_subscribed = false;

const double WHEEL_RADIUS = 0.075;//[m]
const double WHEEL_BASE = 0.50;//[m]
const double TREAD = 0.50;//[m]

void velocity_callback(const geometry_msgs::TwistConstPtr &msg)
{
  velocity = *msg;
  velocity_subscribed = true;
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "fwdis_controller");
  ros::NodeHandle nh;

  ros::Publisher frw_pub = nh.advertise<std_msgs::Float64>("/fwdis/front_right_wheel_joint/command", 100);
  ros::Publisher flw_pub = nh.advertise<std_msgs::Float64>("/fwdis/front_left_wheel_joint/command", 100);
  ros::Publisher rrw_pub = nh.advertise<std_msgs::Float64>("/fwdis/rear_right_wheel_joint/command", 100);
  ros::Publisher rlw_pub = nh.advertise<std_msgs::Float64>("/fwdis/rear_left_wheel_joint/command", 100);
  ros::Publisher frs_pub = nh.advertise<std_msgs::Float64>("/fwdis/front_right_steering_joint/command", 100);
  ros::Publisher fls_pub = nh.advertise<std_msgs::Float64>("/fwdis/front_left_steering_joint/command", 100);
  ros::Publisher rrs_pub = nh.advertise<std_msgs::Float64>("/fwdis/rear_right_steering_joint/command", 100);
  ros::Publisher rls_pub = nh.advertise<std_msgs::Float64>("/fwdis/rear_left_steering_joint/command", 100);

  ros::Subscriber velocity_sub = nh.subscribe("/fwdis/velocity", 100, velocity_callback);

  ros::Rate loop_rate(10);

  while(ros::ok()){
    if(velocity_subscribed){
      if((velocity.linear.x == 0.0) && (velocity.linear.y == 0.0) && (velocity.angular.z == 0.0)){
        //ニュートラル
        std_msgs::Float64 _velocity;
        _velocity.data = 0;
        frw_pub.publish(_velocity);
        flw_pub.publish(_velocity);
        rrw_pub.publish(_velocity);
        rlw_pub.publish(_velocity);

      }else if((velocity.linear.x == 0.0) && (velocity.linear.y == 0.0) && (velocity.angular.z != 0.0)){
        //旋回
        std_msgs::Float64 _angle_fr_rl;
        _angle_fr_rl.data = M_PI / 4.0;
        frs_pub.publish(_angle_fr_rl);
        rls_pub.publish(_angle_fr_rl);
        std_msgs::Float64 _angle_fl_rr;
        _angle_fl_rr.data = M_PI / -4.0;
        fls_pub.publish(_angle_fl_rr);
        rrs_pub.publish(_angle_fl_rr);

        std_msgs::Float64 _velocity_fr_rr;
        _velocity_fr_rr.data = velocity.angular.z * (0.5 / sqrt(2)) / WHEEL_RADIUS;
        frw_pub.publish(_velocity_fr_rr);
        rrw_pub.publish(_velocity_fr_rr);
        std_msgs::Float64 _velocity_fl_rl;
        _velocity_fl_rl.data = -_velocity_fr_rr.data;
        flw_pub.publish(_velocity_fl_rl);
        rlw_pub.publish(_velocity_fl_rl);

      }else if((velocity.linear.x != 0) && (velocity.angular.z != 0.0)){
        //カーブ
        std_msgs::Float64 _velocity_fl;
        std_msgs::Float64 _velocity_fr;
        std_msgs::Float64 _velocity_rl;
        std_msgs::Float64 _velocity_rr;

        std_msgs::Float64 _angle_fl;
        std_msgs::Float64 _angle_fr;
        std_msgs::Float64 _angle_rl;
        std_msgs::Float64 _angle_rr;

        const double STEERING_ANGLE_LIMIT = M_PI / 4.0;

        if(velocity.linear.x > 0.0){
          //前進
          if(velocity.angular.z > 0.0){
            //左
            double vx = velocity.linear.x;
            double omega = velocity.angular.z;
            double radius_in = vx / omega - TREAD / 2.0;
            double radius_limit = WHEEL_BASE / (2 * sin(STEERING_ANGLE_LIMIT));
            if(radius_in < radius_limit){
              omega = vx / (radius_limit + TREAD / 2.0);
              radius_in = vx / omega  - TREAD / 2.0;
            }
            double value = asin(WHEEL_BASE / (2 * radius_in));
            _angle_fl.data = asin(value);
            if(_angle_fl.data > STEERING_ANGLE_LIMIT){
              _angle_fl.data = STEERING_ANGLE_LIMIT;
            }else if(_angle_fl.data < 0.0){
              _angle_fl.data = 0.0;
            }
            _angle_fr.data = atan(tan(_angle_fl.data) / (2 * tan(_angle_fl.data) + 1));
            _angle_rl.data = -_angle_fl.data;
            _angle_rr.data = -_angle_fr.data;

            _velocity_fl.data = vx - TREAD * omega / 2.0;
            _velocity_fl.data /= WHEEL_RADIUS;
            _velocity_fr.data = vx + TREAD * omega / 2.0;
            _velocity_fr.data /= WHEEL_RADIUS;
            _velocity_rl.data = _velocity_fl.data;
            _velocity_rr.data = _velocity_fr.data;
          }else{
            //右
            double vx = velocity.linear.x;
            double omega = -velocity.angular.z;
            double radius_in = vx / omega - TREAD / 2.0;
            double radius_limit = WHEEL_BASE / (2 * sin(STEERING_ANGLE_LIMIT));
            if(radius_in < radius_limit){
              omega = vx / (radius_limit + TREAD / 2.0);
              radius_in = vx / omega  - TREAD / 2.0;
            }
            double value = asin(WHEEL_BASE / (2 * radius_in));
            _angle_fr.data = -asin(value);
            if(_angle_fr.data < -STEERING_ANGLE_LIMIT){
              _angle_fr.data = -STEERING_ANGLE_LIMIT;
            }else if(_angle_fl.data > 0.0){
              _angle_fr.data = 0.0;
            }
            _angle_fl.data = -atan(tan(-_angle_fr.data) / (2 * tan(-_angle_fr.data) + 1));
            _angle_rl.data = -_angle_fl.data;
            _angle_rr.data = -_angle_fr.data;

            _velocity_fl.data = vx + TREAD * omega / 2.0;
            _velocity_fl.data /= WHEEL_RADIUS;
            _velocity_fr.data = vx - TREAD * omega / 2.0;
            _velocity_fr.data /= WHEEL_RADIUS;
            _velocity_rl.data = _velocity_fl.data;
            _velocity_rr.data = _velocity_fr.data;
          }
        }else{
          //後退
          if(velocity.angular.z > 0.0){
            //左
            double vx = -velocity.linear.x;
            double omega = velocity.angular.z;
            double radius_in = vx / omega - TREAD / 2.0;
            double radius_limit = WHEEL_BASE / (2 * sin(STEERING_ANGLE_LIMIT));
            if(radius_in < radius_limit){
              omega = vx / (radius_limit + TREAD / 2.0);
              radius_in = vx / omega  - TREAD / 2.0;
            }
            double value = asin(WHEEL_BASE / (2 * radius_in));
            _angle_fl.data = asin(value);
            if(_angle_fl.data > STEERING_ANGLE_LIMIT){
              _angle_fl.data = STEERING_ANGLE_LIMIT;
            }else if(_angle_fl.data < 0.0){
              _angle_fl.data = 0.0;
            }
            _angle_fr.data = atan(tan(_angle_fl.data) / (2 * tan(_angle_fl.data) + 1));
            _angle_rl.data = -_angle_fl.data;
            _angle_rr.data = -_angle_fr.data;

            _velocity_fl.data = vx - TREAD * omega / 2.0;
            _velocity_fl.data /= -WHEEL_RADIUS;
            _velocity_fr.data = vx + TREAD * omega / 2.0;
            _velocity_fr.data /= -WHEEL_RADIUS;
            _velocity_rl.data = _velocity_fl.data;
            _velocity_rr.data = _velocity_fr.data;
          }else{
            //右
            double vx = -velocity.linear.x;
            double omega = -velocity.angular.z;
            double radius_in = vx / omega - TREAD / 2.0;
            double radius_limit = WHEEL_BASE / (2 * sin(STEERING_ANGLE_LIMIT));
            if(radius_in < radius_limit){
              omega = vx / (radius_limit + TREAD / 2.0);
              radius_in = vx / omega  - TREAD / 2.0;
            }
            double value = asin(WHEEL_BASE / (2 * radius_in));
            _angle_fr.data = -asin(value);
            if(_angle_fr.data < -STEERING_ANGLE_LIMIT){
              _angle_fr.data = -STEERING_ANGLE_LIMIT;
            }else if(_angle_fl.data > 0.0){
              _angle_fr.data = 0.0;
            }
            _angle_fl.data = -atan(tan(-_angle_fr.data) / (2 * tan(-_angle_fr.data) + 1));
            _angle_rl.data = -_angle_fl.data;
            _angle_rr.data = -_angle_fr.data;

            _velocity_fl.data = vx + TREAD * omega / 2.0;
            _velocity_fl.data /= -WHEEL_RADIUS;
            _velocity_fr.data = vx - TREAD * omega / 2.0;
            _velocity_fr.data /= -WHEEL_RADIUS;
            _velocity_rl.data = _velocity_fl.data;
            _velocity_rr.data = _velocity_fr.data;
          }
        }

        frw_pub.publish(_velocity_fr);
        flw_pub.publish(_velocity_fl);
        rrw_pub.publish(_velocity_rr);
        rlw_pub.publish(_velocity_rl);
        frs_pub.publish(_angle_fr);
        fls_pub.publish(_angle_fl);
        rrs_pub.publish(_angle_rr);
        rls_pub.publish(_angle_rl);

      }else{
        //平行移動
        std_msgs::Float64 _angle;
        _angle.data = atan2(velocity.linear.y, velocity.linear.x);
        bool inverce_flag = false;
        if(_angle.data > M_PI/1.5){
          _angle.data -= M_PI;
          inverce_flag = true;
        }else if(_angle.data < -M_PI/1.5){
          _angle.data += M_PI;
          inverce_flag = true;
        }
        frs_pub.publish(_angle);
        fls_pub.publish(_angle);
        rrs_pub.publish(_angle);
        rls_pub.publish(_angle);

        std_msgs::Float64 _velocity;
        _velocity.data = sqrt(velocity.linear.x * velocity.linear.x + velocity.linear.y * velocity.linear.y) / WHEEL_RADIUS;
        if(inverce_flag){
          _velocity.data = -_velocity.data;
        }
        frw_pub.publish(_velocity);
        flw_pub.publish(_velocity);
        rrw_pub.publish(_velocity);
        rlw_pub.publish(_velocity);
      }
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
