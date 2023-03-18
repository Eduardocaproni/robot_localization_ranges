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
MAX_RANGE = 1e3
MIN_RANGE = 0.0
COVARIANCE = 1e-2

class Pose:
    def __init__(self, origin_frame='world', child_frame='robot', t=np.matrix(np.zeros(3)), R=np.matrix(np.eye(3))):
        self.t = t.T
        self.R = R
        self.origin_frame = origin_frame
        self.child_frame = child_frame

    def toTF(self):
        msg = TransformStamped()
        msg.header.frame_id = self.origin_frame
        msg.child_frame_id = self.child_frame
        
        msg.transform.translation.x = self.t[0,0]
        msg.transform.translation.y = self.t[1,0]
        msg.transform.translation.z = self.t[2,0]

        q = Rotation.from_matrix(pose.R).as_quat()
        msg.transform.rotation.x = q[0]
        msg.transform.rotation.y = q[1]
        msg.transform.rotation.z = q[2]
        msg.transform.rotation.w = q[3]

        return msg

anchors = [[0,0,0],
           [0,0,1],
           [1,0,1],
           [1,1,1]]
for idx, anchor in enumerate(anchors):
    pose = Pose(child_frame=f'anchor_{idx}', t=np.matrix(np.array(anchor), dtype=float))
    br.sendTransform(pose.toTF())
    print(f'anchor_{idx}')

dt = 0.02

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

range_pub = node.create_publisher(RangeWithCovariance, '/r2d2/ranges', 10)

def refresh():
    pose.t += pose.R*toMatrix(cmd.linear)*dt
    print(pose.t.T)

    pose.R += pose.R*skew(cmd.angular)*dt

    br.sendTransform(pose.toTF())

    for idx, anchor in enumerate(anchors):
        range = np.linalg.norm(np.array(pose.t) - np.matrix(np.array(anchor)).T)
        frame_id = f'anchor_{idx}'

        range_msg = RangeWithCovariance()
        range_msg.range_min, range_msg.range_max, range_msg.covariance = MIN_RANGE, MAX_RANGE, COVARIANCE
        range_msg.range = range
        range_msg.header.frame_id = frame_id
        range_msg.moving_frame = 'robot'

        range_pub.publish(range_msg)


timer = node.create_timer(dt, refresh)



rclpy.spin(node)

node.destroy_node()
rclpy.shutdown()
