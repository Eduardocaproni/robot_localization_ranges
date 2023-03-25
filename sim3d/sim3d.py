#!/usr/bin/env python3

from geometry_msgs.msg import Twist
from anchor_msgs.msg import RangeWithCovariance
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from scipy.spatial.transform import Rotation
from time import sleep

import numpy as np
import yaml
from yaml.loader import SafeLoader

import rclpy
from rclpy.node import Node

rclpy.init(args=None)
node = Node('sim3d')

node.declare_parameter('config_file')
yaml_file = node.get_parameter('config_file').value
with open(yaml_file) as f:
    data = yaml.load(f, Loader=SafeLoader)

br = TransformBroadcaster(node)
br_static = StaticTransformBroadcaster(node)
MAX_RANGE = 1e3
MIN_RANGE = 0.0
COVARIANCE = 1e-2
linear_noise = 1
angular_noise = 1

robot_namespace = data['/**']['ros__parameters']['base_link_frame'].split('/')[0] + '/'
link_prefix = robot_namespace
base_link = data['/**']['ros__parameters']['base_link_frame'].split('/')[1]
map_frame = data['/**']['ros__parameters']['map_frame']
odom_frame = data['/**']['ros__parameters']['odom_frame']
odom_topic = robot_namespace + data['/**']['ros__parameters']['odom0']

publish_gt = False

class Pose:
    def __init__(self, origin_frame=map_frame, child_frame=f'{base_link}base_footprint_gt', t=np.matrix(np.zeros(3)), R=np.matrix(np.eye(3))):
        self.t = t.T
        self.R = R
        self.origin_frame = origin_frame
        self.child_frame = child_frame

    def updateFrom(self, linear_twist, angular_twist):
        self.t += self.R*toMatrix(linear_twist)*dt
        # print(self.t.T)

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

i = 1
anchors = []
anchor_msgs = []
while f'range0_anchor{i}' in data['/**']['ros__parameters'].keys():
    anchor_data = data['/**']['ros__parameters'][f'range0_anchor{i}']['pose']
    anchors.append([anchor_data['x'], anchor_data['y'], anchor_data['z']])    
    
    anchor_pose_array = np.array([anchor_data['x'], anchor_data['y'], anchor_data['z']])
    anchor_pose = Pose(child_frame=f'anchor{i}', t=np.matrix(anchor_pose_array, dtype=float))
    anchor_msgs.append(anchor_pose.toTF())
    i+=1

br_static.sendTransform(anchor_msgs)


odom = Odometry()
odom_pub = node.create_publisher(Odometry, odom_topic, 10)
odomTransform = TransformStamped() # ?
odom.header.frame_id = odomTransform.header.frame_id = odom_frame
odom.child_frame_id = odomTransform.child_frame_id = link_prefix + base_link

dt = 0.02

pose = Pose()
cmd = Twist()

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

range_pub = node.create_publisher(RangeWithCovariance, f'/{robot_namespace}ranges', 10)

timer = node.create_timer(dt, refresh) 

def move(dt):
    pose.updateFrom(odom.twist.twist.linear, odom.twist.twist.angular)
    pose.sendTransform()
    vx = odom.twist.twist.linear.x
    vy = odom.twist.twist.linear.y
    vz = odom.twist.twist.linear.z
    wx = odom.twist.twist.angular.x
    wy = odom.twist.twist.angular.y
    wz = odom.twist.twist.angular.z
    
    # write actual covariance, proportional to velocity
    odom.twist.covariance[0] = max(0.0001, np.abs(vx)*linear_noise*linear_noise)
    odom.twist.covariance[7] = max(0.0001, np.abs(vy)*linear_noise*linear_noise)
    odom.twist.covariance[14] = max(0.0001, np.abs(vz)*linear_noise*linear_noise)
    odom.twist.covariance[21] = max(0.0001, np.abs(wx)*linear_noise*linear_noise)
    odom.twist.covariance[28] = max(0.0001, np.abs(wy)*linear_noise*linear_noise)
    odom.twist.covariance[35] = max(0.0001, np.abs(wz)*angular_noise*angular_noise)

    # add noise: command velocity to measured (odometry) one
    vx *= (1+linear_noise*np.random.normal())
    vy *= (1+linear_noise*np.random.normal())
    vz *= (1+linear_noise*np.random.normal())
    wx *= (1+linear_noise*np.random.normal())
    wy *= (1+linear_noise*np.random.normal())
    wz *= (1+angular_noise*np.random.normal())

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
    odom.header.stamp = node.get_clock().now().to_msg()
    odom_pub.publish(odom)
    odomTransform.transform.translation.x = odom.pose.pose.position.x
    odomTransform.transform.translation.y = odom.pose.pose.position.y
    odomTransform.transform.translation.z = odom.pose.pose.position.z
    odomTransform.transform.rotation = odom.pose.pose.orientation
    odomTransform.header.stamp = node.get_clock().now().to_msg()
    br.sendTransform(odomTransform)

def publish_ranges():
    for idx, anchor in enumerate(anchors):
        range = np.linalg.norm(np.array(pose.t) - np.matrix(np.array(anchor)).T)
        frame_id = f'anchor{idx+1}'

        range_msg = RangeWithCovariance()
        range_msg.range_min, range_msg.range_max, range_msg.covariance = MIN_RANGE, MAX_RANGE, COVARIANCE
        range_msg.range = range
        range_msg.header.frame_id = frame_id
        range_msg.moving_frame = robot_namespace + base_link
        # print(frame_id)
        range_pub.publish(range_msg)

rclpy.spin(node)

node.destroy_node()
rclpy.shutdown()
