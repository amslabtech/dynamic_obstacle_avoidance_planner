#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan

HZ = 10.0

laser = LaserScan()
min_pooling_laser = LaserScan()

def laser_callback(data):
  global laser
  laser = data
  min_pooling_laser = laser

def process():
  laser_pub = rospy.Publisher("/min_pooling_laser", LaserScan, queue_size=10)
  rospy.Subscriber('/scan', LaserScan, laser_callback)

  print "=== sub laser ==="

  r = rospy.Rate(HZ)

  while not rospy.is_shutdown():
    min_pooling_laser.ranges = []
    for i in (len(laser.ranges) / 81 - 1):
      l = min(laser.ranges[i*81:(i+1)*81])
      min_pooling_laser.ranges.append(l)
    laser_pub.publish(min_pooling_laser)

    r.sleep()

if __name__=='__main__':
  rospy.init_node('sub_laser')
  try:
    process()
  except rospy.ROSInterruptException: pass
