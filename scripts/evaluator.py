#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseArray
from nav_msgs.msg import Path
import tf

import math as m

obs_paths = PoseArray()
obs_poses = PoseArray()
PREDICTION_TIME = 0
VELOCITY_TOPIC_NAME = ""
last_velocity = Twist()
start_flag = False
started_flag = False
intermediate_path = Path()
INTERMEDIATE_PATH_TOPIC_NAME = ""
goal = PoseStamped()
transformed_goal = PoseStamped()
goal_subscribed = False

HZ = 10.0

def obs_callback(data):
  global obs_paths
  global obs_poses
  obs_paths = PoseArray()
  obs_poses = PoseArray()
  obs_poses.header = data.header
  PREDICTION_STEP = int(PREDICTION_TIME * HZ + 1)
  obs_paths = data
  _size = len(obs_paths.poses)
  obs_num = int(_size / PREDICTION_STEP)
  for i in range(obs_num):
    obs_poses.poses.append(obs_paths.poses[i * PREDICTION_STEP])
  print PREDICTION_TIME
  print PREDICTION_STEP
  print _size
  print obs_num

def velocity_callback(data):
  global last_velocity
  global start_flag
  if(last_velocity.linear.x == 0.0 and data.linear.x > 0.0):
    start_flag = True
  last_velocity = data

def path_callback(data):
  #print "=== path_callback ==="
  global intermediate_path
  intermediate_path = data
  _size = len(intermediate_path.poses)
  print _size
  if(_size > 0):
    global goal
    goal = intermediate_path.poses[_size - 1]
    global goal_subscribed
    goal_subscribed = True

def is_goal(x, y):
  dx = x - transformed_goal.pose.position.x
  dy = y - transformed_goal.pose.position.y
  dist = m.sqrt(dx**2 + dy**2)
  if(dist < 0.3):
    return True
  else:
    return False

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
  ROBOT_FRAME = rospy.get_param("/dynamic_avoidance/ROBOT_FRAME")
  OBS_FRAME = rospy.get_param("/dynamic_avoidance/OBSTACLES_FRAME")
  WORLD_FRAME = rospy.get_param("/dynamic_avoidance/WORLD_FRAME")
  global VELOCITY_TOPIC_NAME
  VELOCITY_TOPIC_NAME = rospy.get_param("/dynamic_avoidance/VELOCITY_TOPIC_NAME")
  global PREDICTION_TIME
  PREDICTION_TIME = rospy.get_param("/dynamic_avoidance/PREDICTION_TIME")
  global INTERMEDIATE_PATH_TOPIC_NAME
  INTERMEDIATE_PATH_TOPIC_NAME = rospy.get_param("/dynamic_avoidance/INTERMEDIATE_PATH_TOPIC_NAME")

  rospy.Subscriber("/predicted_paths", PoseArray, obs_callback)
  rospy.Subscriber(VELOCITY_TOPIC_NAME, Twist, velocity_callback)
  rospy.Subscriber(INTERMEDIATE_PATH_TOPIC_NAME, Path, path_callback)
  obs_pub = rospy.Publisher("/evalation/obs", PoseArray, queue_size=1)

  time_to_goal = rospy.get_time()

  transformed = False


  while not rospy.is_shutdown():
    try:
      (trans, rot)= listener.lookupTransform(WORLD_FRAME, ROBOT_FRAME, rospy.Time(0))
      #(_trans, _rot)= listener.lookupTransform(WORLD_FRAME, "local_costmap", rospy.Time(0))
      transformed = True
    except(tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
      continue
    print transformed
    print goal_subscribed
    if(transformed and goal_subscribed):
      robot_x = trans[0]
      robot_y = trans[1]
      global transformed_goal
      transformed_goal = listener.transformPose(WORLD_FRAME, goal)

      global started_flag
      if(start_flag and not started_flag):
        time_to_goal = rospy.get_time()
        started_flag = True
      current_time = rospy.get_time()
      if(not first):
        temp_distance = 2
        temp_min_distance = 2
        for obs in obs_poses.poses:
          temp_distance = m.sqrt((robot_x-obs.position.x)**2 + (robot_y-obs.position.y)**2)
          if(temp_min_distance > temp_distance):
            temp_min_distance = temp_distance
        interval = current_time - last_time
        velocity = m.sqrt((robot_x-last_r_x)**2 + (robot_y-last_r_y)**2) / interval
        ave_velocity = (count * ave_velocity + velocity) / (count + 1.0)
        count += 1.0
        if(temp_min_distance < min_distance):
          min_distance = temp_min_distance

      last_r_x = robot_x
      last_r_y = robot_y
      first = False
      last_time = current_time
      obs_pub.publish(obs_poses)
      if(is_goal(robot_x, robot_y)):
        time_to_goal = rospy.get_time() - time_to_goal
        print "time_to_goal=" + str(time_to_goal)
        return
    print "closest_distance=" + str(min_distance)
    print "average_velocity=" + str(ave_velocity)
    r.sleep()

if __name__=='__main__':
  rospy.init_node('evaluator')
  try:
    process()
  except rospy.ROSInterruptException: pass
