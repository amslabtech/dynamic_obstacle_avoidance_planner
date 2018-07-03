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

def process():
  rospy.init_node('mpc', anonymous=True)

  command_pub = rospy.Publisher("/fwdis/command", FourWheelDriveIndependentSteering, queue_size=100)
  rospy.Subscriber("/path", Path, path_callback)

  r = rospy.Rate(10)

  n_state = 6   # 状態の数(x, y, theta, vx, vy, omega)
  m_state = 8   # 制御入力の数(w1w, w1s, w2w, w2s, w3w, w3s, w4w, w4s)
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
  for t in range(T):
    # コスト関数の値が小さくなるような配列uを求める
    #cost = sum_squares(cost_arr*x[:,t+1]) + sum_squares(0.1*u[:,t])
    # 制約式（線形方程式と制御入力の限界値）を与える
    #constr = [x[:,t+1] == A*x[:,t] + B*u[:,t], norm(u[:,t], 'inf') <= 20.0]
    constr = [x[:,t+1] == A*x[:,t] + B*u[:,t], norm(u[:,t], 'inf') <= 20.0]
    states.append( Problem(Minimize(cost), constr) )
  # sums problem objectives and concatenates constraints.
  prob = sum(states)

  prob.constraints += [x[:,0] == x_0]

  start = time.time()
  result=prob.solve(verbose=True)
  elapsed_time = time.time() - start
  print("calc time:{0} [sec]".format(elapsed_time))

  # 発散した場合は制御不能として終了
  if result == float("inf"):
    print("Cannot optimize")
    import sys
    sys.exit()

  while not rospy.is_shutdown():
    cnt = 0
    # 制御1000回分行う
    while (cnt < 1000) and (not rospy.is_shutdown()):
      states = []
      for t in range(T):
        cost = sum_squares(cost_arr*x[:,t+1]) + sum_squares(0.1*u[:,t])
        constr = [x[:,t+1] == A*x[:,t] + B*u[:,t], norm(u[:,t], 'inf') <= 20.0]
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

      # 最適とされる制御入力の配列を取得
      good_u_arr = np.array(u[0,:].value[0,:])[0]

      # 制御入力を入れて、次の状態を得る
      x_next = np.dot(A, x_0) + B * good_u_arr[0]
      x_0 = x_next

      cnt += 1

      print(x_next.reshape(-1))
    rospy.spin()
    r.sleep()

if __name__ == '__main__':
  try:
    process()
  except rospy.ROSInterruptException: pass
