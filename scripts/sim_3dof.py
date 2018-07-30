#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
import tf

import math as m

pose = PoseStamped()

def velocity_callback(data):
  q_list = [pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w]
  (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(q_list)
  q = tf.transformations.quaternion_from_euler(roll, pitch, yaw + data.angular.z * 0.1)
  pose.pose.orientation.x = q[0]
  pose.pose.orientation.y = q[1]
  pose.pose.orientation.z = q[2]
  pose.pose.orientation.w = q[3]
  pose.pose.position.x += data.linear.x * m.cos(yaw) * 0.1 - data.linear.y * m.sin(yaw) * 0.1
  pose.pose.position.y += data.linear.x * m.sin(yaw) * 0.1 + data.linear.y * m.cos(yaw) * 0.1

def process():
  rospy.Subscriber('/velocity', Twist, velocity_callback)

  br = tf.TransformBroadcaster()

  pose.pose.orientation.w = 1

  r = rospy.Rate(10)

  while not rospy.is_shutdown():
    br.sendTransform((pose.pose.position.x, pose.pose.position.y, 0), (pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w), rospy.Time.now(), "base_link", "odom")
    r.sleep()

if __name__=='__main__':
  rospy.init_node('sim_3dof')
  try:
    process()
  except rospy.ROSInterruptException: pass
