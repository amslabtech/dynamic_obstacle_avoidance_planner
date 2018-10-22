#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan

HZ = 10.0

laser = LaserScan()

def laser_callback(data):
  global laser
  laser = data

def process():
  rospy.Subscriber('/scan', LaserScan, laser_callback)

  print "=== sub laser ==="

  r = rospy.Rate(HZ)

  while not rospy.is_shutdown():
    print len(laser.ranges)
    r.sleep()

if __name__=='__main__':
  rospy.init_node('sub_laser')
  try:
    process()
  except rospy.ROSInterruptException: pass
