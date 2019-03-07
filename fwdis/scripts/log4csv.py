#!/usr/bin/env python
#! coding:utf-8

import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Empty
import tf

import math as m
import numpy as np
import os

HZ = 20.0

FILE_NAME = 'ex5.csv'
ROBOT_FRAME = '/vicon/base_link/base_link'
WORLD_FRAME = '/world'
VELOCITY_TOPIC_NAME = '/fwdis/velocity'

class Log4csv:
  def __init__(self):
    self.vel = Twist()
    self.path = os.getcwd() + '/' + FILE_NAME
    self.first_flag = True
    self.p_0 = PoseStamped()
    self.theta_0 = 0

  def vel_callback(self, data):
    self.vel = data
    if self.first_flag:
      self.first_time = rospy.get_time()
      self.first_flag = False
      (trans, rot) = self.listener.lookupTransform(WORLD_FRAME, ROBOT_FRAME, rospy.Time(0))
      self.p_0.pose.position.x = trans[0]
      self.p_0.pose.position.y = trans[1]
      self.p_0.pose.orientation.x = rot[0]
      self.p_0.pose.orientation.y = rot[1]
      self.p_0.pose.orientation.z = rot[2]
      self.p_0.pose.orientation.w = rot[3]
      _, _, self.theta_0 = tf.transformations.euler_from_quaternion(rot)
    else:
      try:
        (trans, rot) = self.listener.lookupTransform(WORLD_FRAME, ROBOT_FRAME, rospy.Time(0))
        with open(self.path, 'a') as f:
          x = trans[0] * m.cos(-self.theta_0) - trans[1] * m.sin(-self.theta_0)
          y = trans[0] * m.sin(-self.theta_0) + trans[1] * m.cos(-self.theta_0)
          sentence = (str(rospy.get_time() - self.first_time) + ','
                    + str(x) + ','
                    + str(y) + ','
                    + str(self.vel.linear.x) + ','
                    + str(self.vel.linear.y) + ','
                    + str(self.vel.angular.z)
                    + '\n'
                    )
          f.write(sentence)
          print sentence
      except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        print 'exception'

  def process(self):
    rospy.init_node('log4csv')
    self.vel_sub = rospy.Subscriber(VELOCITY_TOPIC_NAME, Twist, self.vel_callback, queue_size=1)
    self.listener = tf.TransformListener()
    self.first_time = rospy.get_time()
    print '=== log4csv ==='
    rospy.spin()

if __name__=='__main__':
  log4csv = Log4csv()
  try:
    log4csv.process()
  except rospy.ROSInterruptException: pass
