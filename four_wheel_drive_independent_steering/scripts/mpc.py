#!/usr/bin/env python
# -*- coding:utf-8 -*-

# mpc
import time
from cvxpy import *
import numpy as np
import matplotlib.pyplot as plt

# ros
import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from four_wheel_drive_independent_steering.msg import FourWheelDriveIndependentSteering

#other
import math

global_path = []

def path_callback(data):
    global_path = data

def get_matrix(dt, theta_t):
  A = np.array([
    [dt * math.cos(theta_t), -dt * math.sin(theta_t), 0],
    [dt * math.sin(theta_t),  dt * math.cos(theta_t), 0],
    [0, 0, 1]
    ])
  return A

def process():
  rospy.init_node('mpc', anonymous=True)

  command_pub = rospy.Publisher("/fwdis/command", FourWheelDriveIndependentSteering, queue_size=100)
  rospy.Subscriber("/path", Path, path_callback)

  r = rospy.Rate(10)

  n_state = 3#6   # 状態の数(x, y, theta, vx, vy, omega)
  m_state = 3#8   # 制御入力の数(w1w, w1s, w2w, w2s, w3w, w3s, w4w, w4s)
  T = 20  # 何ステップ先まで予測するかを決める

  #simulation parameter
  delta_t = 0.1

  WHEEL_RADIUS = 0.100#[m]
  WHEEL_BASE = 0.040#[m]
  TREAD = 0.040#[m]
  RADIUS = math.sqrt(WHEEL_BASE**2 + TREAD**2) / 2.0
  THETA = math.atan(TREAD / WHEEL_BASE)
  MAX_OMEGA = 30#[rad/s]
  MAX_STEERING_ANGLE = math.pi / 1.5#[rad]

  target_theta = 0
  target_v = 2.0

  # Model Parameter
  forward_matrix = np.array([
    [1.0, 0.0,  RADIUS * math.sin(THETA)],
    [0.0, 1.0,  RADIUS * math.cos(THETA)],
    [1.0, 0.0, -RADIUS * math.sin(THETA)],
    [0.0, 1.0,  RADIUS * math.cos(THETA)],
    [1.0, 0.0, -RADIUS * math.sin(THETA)],
    [0.0, 1.0, -RADIUS * math.cos(THETA)],
    [1.0, 0.0,  RADIUS * math.sin(THETA)],
    [0.0, 1.0, -RADIUS * math.cos(THETA)],
    ])
  inversed_matrix = np.linalg.pinv(forward_matrix)

  x = Variable(n_state, T+1)
  u = Variable(m_state, T)

  cost_arr = np.array([
    [1.0, 0.0, 0.0, 0.0],
    [0.0, 1.0, 0.0, 0.0],
    [0.0, 0.0, 0.1, 0.0],
    [0.0, 0.0, 0.0, 0.1]
    ])

  states = []

  while not rospy.is_shutdown():
    states = []
    for t in range(T):
      cost = sum_squares(cost_arr*x[:,t+1]) + sum_squares(0.1*u[:,t])
      constr = [x[:,t+1] == x[:,t] + A*u[:,t]]
      states.append( Problem(Minimize(cost), constr) )
    # sums problem objectives and concatenates constraints.
    prob = sum(states)
    prob.constraints += [x[:,T] == 0, x[:,0] == x_0]

    start = time.time()
    result=prob.solve(verbose=False)
    elapsed_time = time.time() - start

    if result == float("inf"):
      print("Cannot optimize")
      import sys
      sys.exit()

    # optimized value array
    ox = np.array(x.value[0, :]).flatten()
    oy = np.array(x.value[1, :]).flatten()
    oyaw = np.array(x.value[2, :]).flatten()
    ovx = np.array(u.value[0, :]).flatten()
    ovy = np.array(u.value[1, :]).flatten()
    ow = np.array(u.value[2, :]).flatten()

    # 最適とされる制御入力の配列を取得
    good_u_arr = np.array(u[0,:].value[0,:])[0]

    # 制御入力を入れて、次の状態を得る
    x_next = np.dot(A, x_0) + B * good_u_arr[0]
    x_0 = x_next

    rospy.spin()
    r.sleep()

if __name__ == '__main__':
  try:
    process()
  except rospy.ROSInterruptException: pass
