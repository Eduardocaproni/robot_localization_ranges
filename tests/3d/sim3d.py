#!/usr/bin/env python3

from geometry_msgs.msg import Twist
from anchor_msgs.msg import RangeWithCovariance
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from scipy.spatial.transform import Rotation

import numpy as np

import rclpy
from rclpy.node import Node

rclpy.init(args=None)
node = Node('sim3d')
br = TransformBroadcaster(node)

anchors = [[0,0,0],
           [0,0,1]]

dt = 0.02


class Pose:
    def __init__(self):
        self.t = np.matrix(np.zeros(3)).T
        self.R = np.matrix(np.eye(3))

    def toTF(self):
        msg = TransformStamped()
        msg.header.frame_id = 'world'
        msg.child_frame_id = 'robot'

        msg.transform.translation.x = self.t[0,0]
        msg.transform.translation.y = self.t[1,0]
        msg.transform.translation.z = self.t[2,0]

        q = Rotation.from_matrix(pose.R).as_quat()
        msg.transform.rotation.x = q[0]
        msg.transform.rotation.y = q[1]
        msg.transform.rotation.z = q[2]
        msg.transform.rotation.w = q[3]

        return msg


pose = Pose()
cmd = Twist()

def toMatrix(v):
    return np.matrix((v.x,v.y,v.z)).T

def skew(v):
    return np.matrix([[0, -v.z,v.y],[v.z,0,-v.x],[-v.y,v.x,0]])

def cmd_callback(msg):
    global cmd
    cmd = msg

cmd_sub = node.create_subscription(Twist, 'cmd_vel', cmd_callback, 1)

#range_pub = ...

def refresh():
    pose.t += pose.R*toMatrix(cmd.linear)*dt
    print(pose.t.T)

    pose.R += pose.R*skew(cmd.angular)*dt

    br.sendTransform(pose.toTF())

    #range_pub.publish


timer = node.create_timer(dt, refresh)



rclpy.spin(node)

node.destroy_node()
rclpy.shutdown()
