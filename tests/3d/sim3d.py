#!/usr/bin/env python3

from geometry_msgs.msg import Twist
from anchor_msgs.msg import RangeWithCovariance
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from scipy.spatial.transform import Rotation

import numpy as np

import rclpy
from rclpy.node import Node

rclpy.init(args=None)
node = Node('sim3d')
br = TransformBroadcaster(node)
br_static = StaticTransformBroadcaster(node)
MAX_RANGE = 1e3
MIN_RANGE = 0.0
COVARIANCE = 1e-2
linear_noise = 0
angular_noise = 0
robot_namespace = 'r2d2/'
link_prefix = ''
base_link = 'base_link'
publish_gt = False

class Pose:
    def __init__(self, origin_frame='map', child_frame='robot', t=np.matrix(np.zeros(3)), R=np.matrix(np.eye(3))):
        self.t = t.T
        self.R = R
        self.origin_frame = origin_frame
        self.child_frame = child_frame

    def updateFrom(self, linear_twist, angular_twist):
        # import pdb; pdb.set_trace()
        self.t += self.R*toMatrix(linear_twist)*dt
        print(self.t.T)

        self.R += self.R*skew(angular_twist)*dt

    def sendTransform(self):
        br.sendTransform(self.toTF())

    def toTF(self):
        msg = TransformStamped()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.header.frame_id = self.origin_frame
        msg.child_frame_id = self.child_frame
        
        msg.transform.translation.x = self.t[0,0]
        msg.transform.translation.y = self.t[1,0]
        msg.transform.translation.z = self.t[2,0]

        q = Rotation.from_matrix(self.R).as_quat()
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
    anchor_pose = Pose(child_frame=f'anchor{idx+1}', t=np.matrix(np.array(anchor), dtype=float))
    br_static.sendTransform(anchor_pose.toTF())
    print(f'anchor_{idx}')

odom = Odometry()
odom_pub = node.create_publisher(Odometry, robot_namespace + "odom", 10)
odomTransform = TransformStamped() # ?
odom.header.frame_id = odomTransform.header.frame_id = link_prefix + "odom";
# odom.header.frame_id = odomTransform.header.frame_id = 'map'
odom.child_frame_id = odomTransform.child_frame_id = link_prefix + base_link

dt = 0.02

pose = Pose()
cmd = Twist()

odom2map = TransformStamped()
odom2map.header.frame_id = "map";
odom2map.child_frame_id = odom.header.frame_id
odom2map.transform.translation.x = pose.t[0, 0]
odom2map.transform.translation.y = pose.t[1, 0]
odom2map.transform.translation.y = pose.t[2, 0]
qx, qy, qz, qw = Rotation.from_matrix(pose.R).as_quat()
odom2map.transform.rotation.x = qx
odom2map.transform.rotation.y = qy
odom2map.transform.rotation.z = qz
odom2map.transform.rotation.w = qw
br_static.sendTransform(odom2map)

def toMatrix(v):
    return np.matrix((v.x,v.y,v.z)).T

def skew(v):
    return np.matrix([[0, -v.z,v.y],[v.z,0,-v.x],[-v.y,v.x,0]])

def cmd_callback(msg):
    odom.twist.twist.linear.x = msg.linear.x
    odom.twist.twist.linear.y = msg.linear.y
    odom.twist.twist.linear.z = msg.linear.z
    odom.twist.twist.angular.x = msg.angular.x
    odom.twist.twist.angular.y = msg.angular.y
    odom.twist.twist.angular.z = msg.angular.z

# const rclcpp::Time &
def refresh():
    move(dt)
    publish_odom()
    publish_ranges()

cmd_sub = node.create_subscription(Twist, 'cmd_vel', cmd_callback, 1)

range_pub = node.create_publisher(RangeWithCovariance, '/r2d2/ranges', 10)

timer = node.create_timer(dt, refresh) 

def move(dt):
    pose.updateFrom(odom.twist.twist.linear, odom.twist.twist.angular)
    pose.sendTransform()
    # vx = odom.twist.twist.linear.x
    # vy = odom.twist.twist.linear.y
    # vz = odom.twist.twist.linear.z
    # wx = odom.twist.twist.angular.x
    # wy = odom.twist.twist.angular.y
    # wz = odom.twist.twist.angular.z

    # write actual covariance, proportional to velocity
    # odom.twist.covariance[0] = max(0.0001, np.abs(vx)*linear_noise*linear_noise)
    # odom.twist.covariance[7] = max(0.0001, np.abs(vy)*linear_noise*linear_noise)
    # odom.twist.covariance[35] = max(0.0001, np.abs(wz)*angular_noise*angular_noise)

    # add noise: command velocity to measured (odometry) one
    # vx *= (1+linear_noise*unit_noise(random_engine))
    # vy *= (1+linear_noise*unit_noise(random_engine))
    # vz *= (1+linear_noise*unit_noise(random_engine))
    # wx *= (1+linear_noise*unit_noise(random_engine))
    # wy *= (1+linear_noise*unit_noise(random_engine))
    # wz *= (1+angular_noise*unit_noise(random_engine))

    #  update noised odometry
    rel_pose = Pose()
    rel_pose.t = toMatrix(odom.pose.pose.position)
    odom_quat = np.matrix(np.array([odom.pose.pose.orientation.x,
                                     odom.pose.pose.orientation.y,
                                     odom.pose.pose.orientation.z,
                                     odom.pose.pose.orientation.w]))
    rel_pose.R = Rotation.from_quat(odom_quat).as_matrix()[0]
    rel_pose.updateFrom(odom.twist.twist.linear, odom.twist.twist.angular)
    qx, qy, qz, qw = Rotation.from_matrix(rel_pose.R).as_quat()

    odom.pose.pose.position.x = rel_pose.t[0, 0]
    odom.pose.pose.position.y = rel_pose.t[1, 0]
    odom.pose.pose.position.z = rel_pose.t[2, 0]
    odom.pose.pose.orientation.x = qx
    odom.pose.pose.orientation.y = qy
    odom.pose.pose.orientation.z = qz
    odom.pose.pose.orientation.w = qw

def publish_odom():
    # odom.header.stamp = transform.header.stamp = stamp;
    # build odom angle & publish as msg + tf
    odom_pub.publish(odom)
    #  build transform odom . base link
    odomTransform.transform.translation.x = odom.pose.pose.position.x
    odomTransform.transform.translation.y = odom.pose.pose.position.y
    odomTransform.transform.translation.z = odom.pose.pose.position.z
    odomTransform.transform.rotation = odom.pose.pose.orientation
    br.sendTransform(odomTransform)

def publish_ranges():
    for idx, anchor in enumerate(anchors):
        range = np.linalg.norm(np.array(pose.t) - np.matrix(np.array(anchor)).T)
        frame_id = f'anchor{idx+1}'

        range_msg = RangeWithCovariance()
        range_msg.range_min, range_msg.range_max, range_msg.covariance = MIN_RANGE, MAX_RANGE, COVARIANCE
        range_msg.range = range
        range_msg.header.frame_id = frame_id
        range_msg.moving_frame = 'robot'
        print(frame_id)
        range_pub.publish(range_msg)

rclpy.spin(node)

node.destroy_node()
rclpy.shutdown()
