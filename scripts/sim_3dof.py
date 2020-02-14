#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty
import tf

import math as m

# global variables
pose = PoseStamped()
velocity = Twist()
stop_flag = False
MAX_ACCELERATION = 0
MAX_YAWRATE = 0
MAX_D_YAWRATE = 0

HZ = 20.0

def velocity_callback(data):
    # print "velocity callback"
    global velocity
    velocity = data
    # print velocity

def stop_callback(data):
    global stop_flag
    stop_flag = True

def process():
    global MAX_ACCELERATION
    global MAX_YAWRATE
    global MAX_D_YAWRATE
    ROBOT_FRAME = rospy.get_param("/dynamic_avoidance/ROBOT_FRAME")
    VELOCITY_TOPIC_NAME = rospy.get_param("/dynamic_avoidance/VELOCITY_TOPIC_NAME")
    MAX_ACCELERATION = rospy.get_param("MAX_ACCELERATION", 1.0)
    MAX_YAWRATE = rospy.get_param("MAX_ANGULAR_VELOCITY", 1.0)
    MAX_D_YAWRATE = rospy.get_param("MAX_ANGULAR_ACCCELERATION", 3.14)

    # print MAX_ACCELERATION
    # print MAX_D_YAWRATE

    odom_pub = rospy.Publisher("/odom", Odometry, queue_size=1)
    pose_pub = rospy.Publisher("pose", PoseStamped, queue_size=1)
    rospy.Subscriber(VELOCITY_TOPIC_NAME, Twist, velocity_callback)
    rospy.Subscriber("/stop", Empty, stop_callback)

    print "=== sim 3dof ==="

    br = tf.TransformBroadcaster()

    pose.pose.position.x = -10
    pose.pose.orientation.w = 1
    pose.header.frame_id = ROBOT_FRAME

    global velocity
    velocity.linear.x = 0
    velocity.linear.y = 0
    velocity.angular.z = 0

    r = rospy.Rate(HZ)

    cmd = Twist()
    while not rospy.is_shutdown():
        if stop_flag:
            velocity.linear.x = 0
            velocity.linear.y = 0

        print "velocity"
        # print velocity
        dt = 1. / HZ
        acc_x = (velocity.linear.x - cmd.linear.x) / dt
        # print acc_x
        acc_x = max(-MAX_ACCELERATION, min(MAX_ACCELERATION, acc_x))
        # print acc_x
        acc_y = (velocity.linear.y - cmd.linear.y) / dt
        acc_y = max(-MAX_ACCELERATION, min(MAX_ACCELERATION, acc_y))
        cmd.linear.x += acc_x * dt
        cmd.linear.y += acc_y * dt
        ang_acc_z = (velocity.angular.z - cmd.angular.z) / dt
        # print ang_acc_z
        ang_acc_z = max(-MAX_D_YAWRATE, min(MAX_D_YAWRATE, ang_acc_z))
        # print ang_acc_z
        cmd.angular.z += ang_acc_z * dt
        print cmd

        q_list = [pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w]
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(q_list)
        q = tf.transformations.quaternion_from_euler(roll, pitch, yaw + cmd.angular.z / HZ)
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]
        pose.pose.position.x += cmd.linear.x * m.cos(yaw) / HZ - cmd.linear.y * m.sin(yaw) / HZ
        pose.pose.position.y += cmd.linear.x * m.sin(yaw) / HZ + cmd.linear.y * m.cos(yaw) / HZ
        br.sendTransform((pose.pose.position.x, pose.pose.position.y, 0), (pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w), rospy.Time.now(), ROBOT_FRAME, "odom")
        odom = Odometry()
        odom.header.frame_id = "odom"
        odom.header.stamp = rospy.Time.now()
        odom.child_frame_id = ROBOT_FRAME
        odom.pose.pose = pose.pose
        odom.twist.twist = cmd
        odom_pub.publish(odom)
        pose.header = odom.header
        print pose
        pose_pub.publish(pose)
        r.sleep()

if __name__=='__main__':
    rospy.init_node('sim_3dof')
    try:
        process()
    except rospy.ROSInterruptException: pass
