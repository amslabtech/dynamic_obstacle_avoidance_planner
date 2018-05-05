#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "four_wheel_drive_independent_steering/FourWheelDriveIndependentSteering.h"

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

  ros::Publisher command_pub = nh.advertise<four_wheel_drive_independent_steering::FourWheelDriveIndependentSteering>("/fwdis/command", 100);
  ros::Subscriber velocity_sub = nh.subscribe("/fwdis/velocity", 100, velocity_callback);

  ros::Rate loop_rate(10);

  while(ros::ok()){
    four_wheel_drive_independent_steering::FourWheelDriveIndependentSteering command;
    if(velocity_subscribed){
      if((velocity.linear.x == 0.0) && (velocity.linear.y == 0.0) && (velocity.angular.z == 0.0)){
        //ニュートラル
        command_pub.publish(command);

      }else if((velocity.linear.x == 0.0) && (velocity.linear.y == 0.0) && (velocity.angular.z != 0.0)){
        //旋回
        command.front_right_steering_angle = M_PI / 4.0;
        command.rear_left_steering_angle = command.front_right_steering_angle;
        command.front_left_steering_angle = M_PI / -4.0;
        command.rear_right_steering_angle = command.front_left_steering_angle;

        command.front_right_wheel_velocity = velocity.angular.z * (0.5 / sqrt(2)) / WHEEL_RADIUS;
        command.rear_right_wheel_velocity = command.front_right_wheel_velocity;
        command.front_left_wheel_velocity = -command.front_right_wheel_velocity;
        command.rear_left_wheel_velocity = command.front_left_wheel_velocity;

        command_pub.publish(command);
      }else if((velocity.linear.x != 0) && (velocity.angular.z != 0.0)){
        //カーブ
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
            command.front_left_steering_angle = asin(value);
            if(command.front_left_steering_angle > STEERING_ANGLE_LIMIT){
              command.front_left_steering_angle = STEERING_ANGLE_LIMIT;
            }else if(command.front_left_steering_angle < 0.0){
              command.front_left_steering_angle = 0.0;
            }
            command.front_right_steering_angle = atan(tan(command.front_left_steering_angle) / (2 * tan(command.front_left_steering_angle) + 1));
            command.rear_left_steering_angle = -command.front_left_steering_angle;
            command.rear_right_steering_angle = -command.front_right_steering_angle;

            command.front_left_wheel_velocity = vx - TREAD * omega / 2.0;
            command.front_left_wheel_velocity /= WHEEL_RADIUS;
            command.front_right_wheel_velocity = vx + TREAD * omega / 2.0;
            command.front_right_wheel_velocity /= WHEEL_RADIUS;
            command.rear_left_wheel_velocity = command.front_left_wheel_velocity;
            command.rear_right_wheel_velocity = command.front_right_wheel_velocity;
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
            command.front_right_steering_angle = -asin(value);
            if(command.front_right_steering_angle < -STEERING_ANGLE_LIMIT){
              command.front_right_steering_angle = -STEERING_ANGLE_LIMIT;
            }else if(command.front_left_steering_angle > 0.0){
              command.front_right_steering_angle = 0.0;
            }
            command.front_left_steering_angle = -atan(tan(-command.front_right_steering_angle) / (2 * tan(-command.front_right_steering_angle) + 1));
            command.rear_left_steering_angle = -command.front_left_steering_angle;
            command.rear_right_steering_angle = -command.front_right_steering_angle;

            command.front_left_wheel_velocity = vx + TREAD * omega / 2.0;
            command.front_left_wheel_velocity /= WHEEL_RADIUS;
            command.front_right_wheel_velocity = vx - TREAD * omega / 2.0;
            command.front_right_wheel_velocity /= WHEEL_RADIUS;
            command.rear_left_wheel_velocity = command.front_left_wheel_velocity;
            command.rear_right_wheel_velocity = command.front_right_wheel_velocity;
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
            command.front_left_steering_angle = asin(value);
            if(command.front_left_steering_angle > STEERING_ANGLE_LIMIT){
              command.front_left_steering_angle = STEERING_ANGLE_LIMIT;
            }else if(command.front_left_steering_angle < 0.0){
              command.front_left_steering_angle = 0.0;
            }
            command.front_right_steering_angle = atan(tan(command.front_left_steering_angle) / (2 * tan(command.front_left_steering_angle) + 1));
            command.rear_left_steering_angle = -command.front_left_steering_angle;
            command.rear_right_steering_angle = -command.front_right_steering_angle;

            command.front_left_wheel_velocity = vx - TREAD * omega / 2.0;
            command.front_left_wheel_velocity /= -WHEEL_RADIUS;
            command.front_right_wheel_velocity = vx + TREAD * omega / 2.0;
            command.front_right_wheel_velocity /= -WHEEL_RADIUS;
            command.rear_left_wheel_velocity = command.front_left_wheel_velocity;
            command.rear_right_wheel_velocity = command.front_right_wheel_velocity;
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
            command.front_right_steering_angle = -asin(value);
            if(command.front_right_steering_angle < -STEERING_ANGLE_LIMIT){
              command.front_right_steering_angle = -STEERING_ANGLE_LIMIT;
            }else if(command.front_left_steering_angle > 0.0){
              command.front_right_steering_angle = 0.0;
            }
            command.front_left_steering_angle = -atan(tan(-command.front_right_steering_angle) / (2 * tan(-command.front_right_steering_angle) + 1));
            command.rear_left_steering_angle = -command.front_left_steering_angle;
            command.rear_right_steering_angle = -command.front_right_steering_angle;

            command.front_left_wheel_velocity = vx + TREAD * omega / 2.0;
            command.front_left_wheel_velocity /= -WHEEL_RADIUS;
            command.front_right_wheel_velocity = vx - TREAD * omega / 2.0;
            command.front_right_wheel_velocity /= -WHEEL_RADIUS;
            command.rear_left_wheel_velocity = command.front_left_wheel_velocity;
            command.rear_right_wheel_velocity = command.front_right_wheel_velocity;
          }
        }
        command_pub.publish(command);

      }else{
        //平行移動
        command.front_right_steering_angle = atan2(velocity.linear.y, velocity.linear.x);
        bool inverce_flag = false;
        if(command.front_right_steering_angle > M_PI/1.5){
          command.front_right_steering_angle -= M_PI;
          inverce_flag = true;
        }else if(command.front_right_steering_angle < -M_PI/1.5){
          command.front_right_steering_angle += M_PI;
          inverce_flag = true;
        }
        command.front_left_steering_angle = command.front_right_steering_angle;
        command.rear_right_steering_angle = command.front_right_steering_angle;
        command.rear_left_steering_angle = command.front_right_steering_angle;

        command.front_right_wheel_velocity = sqrt(velocity.linear.x * velocity.linear.x + velocity.linear.y * velocity.linear.y) / WHEEL_RADIUS;
        if(inverce_flag){
          command.front_right_wheel_velocity = -command.front_right_wheel_velocity;
        }
        command.front_left_wheel_velocity = command.front_right_wheel_velocity;
        command.rear_right_wheel_velocity = command.front_right_wheel_velocity;
        command.rear_left_wheel_velocity = command.front_right_wheel_velocity;

        command_pub.publish(command);
      }
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
