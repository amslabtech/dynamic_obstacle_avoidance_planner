#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
import tf

import math as m

pose = PoseStamped()

HZ = 10.0


def process():
  print "=== evaluator ==="

  listener = tf.TransformListener()

  r = rospy.Rate(HZ)

  ave_velocity = 0
  min_distance = 2.0

  last_r_x = 0
  last_r_y = 0

  count = 0.0

  first = True

  while not rospy.is_shutdown():
    try:
      (trans, rot)= listener.lookupTransform('map', 'base_link', rospy.Time(0))
      robot_x = trans[0]
      robot_y = trans[1]
      (trans, rot) = listener.lookupTransform('map', 'obs0', rospy.Time(0))
      obs0_x = trans[0]
      obs0_y = trans[1]
      if(not first):
        distance = m.sqrt((robot_x-obs0_x)**2 + (robot_y-obs0_y)**2)
        velocity = m.sqrt((robot_x-last_r_x)**2 + (robot_y-last_r_y)**2) / 0.1
        ave_velocity = (count * ave_velocity + velocity) / (count + 1.0)
        count += 1.0
        if(distance < min_distance):
          min_distance = distance

      last_r_x = robot_x
      last_r_y = robot_y
      first = False
    except(tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
      continue
    print "min_distance=" + str(min_distance)
    print "ave_velocity=" + str(ave_velocity)
    r.sleep()

if __name__=='__main__':
  rospy.init_node('evaluator')
  try:
    process()
  except rospy.ROSInterruptException: pass
