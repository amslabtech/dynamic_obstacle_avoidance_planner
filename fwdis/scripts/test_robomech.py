#!/usr/bin/env python
#! coding:utf-8

import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Empty
import tf

import math as m
import numpy as np
from pyquaternion import Quaternion as q

velocity = Twist()

HZ = 20.0

ROBOT = "DD"
#ROBOT = "FWDIS"

def process():
  ROBOT_FRAME = rospy.get_param("/dynamic_avoidance/ROBOT_FRAME")
  WORLD_FRAME = rospy.get_param("/dynamic_avoidance/WORLD_FRAME")
  VELOCITY_TOPIC_NAME = rospy.get_param("/dynamic_avoidance/VELOCITY_TOPIC_NAME")

  vel_pub = rospy.Publisher(VELOCITY_TOPIC_NAME, Twist)

  print "=== test robomech ==="

  velocity.linear.x = 0
  velocity.linear.y = 0
  velocity.angular.z = 0

  r = rospy.Rate(HZ)

  current_pose = np.empty(3)
  current_orientation = np.empty(4)
  current_quaternion = q(current_orientation)
  last_pose = np.empty(3)
  last_orientation = np.empty(4)
  last_quaternion = q(last_orientation)
  first_flag = True
  listener =  tf.TransformListener()

  current_time = rospy.get_time()

  v_robot = 0
  omega = 0
  vel_flag = False
  flag_time = 0
  w = 0
  theta_n = 0
  theta_n_max = m.pi / 2.0

  while not rospy.is_shutdown():
    try:
      last_pose = current_pose
      last_orientation = current_orientation
      last_quaternion = current_quaternion
      (current_pose, current_orientation) = listener.lookupTransform(WORLD_FRAME, ROBOT_FRAME, rospy.Time(0))
      current_pose = np.reshape(np.array(current_pose), (3))
      current_quaternion = q(np.array([current_orientation[3], current_orientation[0], current_orientation[1], current_orientation[2]]))
      current_quaternion = current_quaternion.normalised
      last_time = current_time
      current_time = rospy.get_time()
      dt = current_time - last_time

      if not first_flag:
        print "process"
        _, _, current_yaw = tf.transformations.euler_from_quaternion(np.reshape(current_orientation, (4)))
        _, _, last_yaw = tf.transformations.euler_from_quaternion(np.reshape(last_orientation, (4)))
        d_base = current_pose - last_pose

        #print current_orientation, _last_orientation
        #print current_quaternion
        d_quaternion = current_quaternion.inverse * last_quaternion
        d_quaternion = d_quaternion.normalised
        v_base = d_base / dt
        #print v_base
        v_robot = last_quaternion.inverse.rotate(v_base)
        dyaw = current_yaw - last_yaw
        omega = dyaw / dt
        print v_robot
        print omega
        #print d_quaternion.radians / dt
        V_MAX = 0.5
        if ROBOT == "DD":
          velocity.linear.x = V_MAX
          if (v_robot[0] > 0.5) and (not vel_flag):
            vel_flag = True
            flag_time = rospy.get_time()
          elif vel_flag:
            time = rospy.get_time() - flag_time
            if time > 1.0:
              print time, "[s]"
              W_MAX = 0.8
              DW = 1.0
              w += DW / HZ
              if w < -W_MAX:
                w = -W_MAX
              elif w > W_MAX:
                w = W_MAX
              velocity.angular.z = w
          vel_pub.publish(velocity)
          print velocity
          print "DD"
        elif ROBOT == "FWDIS":
          W_MAX = 3
          current_v = m.sqrt(v_robot[0] * v_robot[0] + v_robot[1] * v_robot[1])
          velocity.linear.x = V_MAX
          if (current_v > 0.5) and (not vel_flag):
            vel_flag = True
            flag_time = rospy.get_time()
          elif vel_flag:
            time = rospy.get_time() - flag_time
            if time > 1.0:
              print time, "[s]"
              theta_n += W_MAX / HZ
              if theta_n < -theta_n_max:
                theta_n = -theta_n_max
              elif theta_n > theta_n_max:
                theta_n = theta_n_max
              velocity.linear.x = V_MAX * m.cos(theta_n)
              velocity.linear.y = V_MAX * m.sin(theta_n)
          vel_pub.publish(velocity)
          print velocity
          print theta_n
          print "FWDIS"
      else:
        first_flag = False
        print "first"

    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
      #print "exception"
      continue

    r.sleep()

if __name__=='__main__':
  rospy.init_node('test_robomech')
  try:
    process()
  except rospy.ROSInterruptException: pass
