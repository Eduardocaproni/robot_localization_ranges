#!/usr/bin/env python3

from geometry_msgs.msg import Twist
from anchor_msgs.msg import RangeWithCovariance
from nav_msgs.msg import Odometry
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
linear_noise = 0
angular_noise = 0
robot_namespace = 'r2d2/'
link_prefix = ''
base_link = 'base_link'
publish_gt = False

class Pose:
    def __init__(self, origin_frame='world', child_frame='robot', t=np.matrix(np.zeros(3)), R=np.matrix(np.eye(3))):
        self.t = t.T
        self.R = R
        self.origin_frame = origin_frame
        self.child_frame = child_frame

    def updateFrom(self):
        pose.t += pose.R*toMatrix(cmd.linear)*dt
        print(pose.t.T)

        pose.R += pose.R*skew(cmd.angular)*dt

        br.sendTransform(pose.toTF())

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

odom = Odometry()
odom_pub = node.create_publisher(Odometry, robot_namespace + "odom", 10)
odomTransform = TransformStamped() # ?
odom.header.frame_id = odomTransform.header.frame_id = link_prefix + "odom";
# odom.header.frame_id = odomTransform.header.frame_id = 'world'
odom.child_frame_id = odomTransform.child_frame_id = link_prefix + base_link;

def cmb_sub_function(msg):
    odom.twist.twist.linear.x = msg.linear.x
    odom.twist.twist.linear.y = msg.linear.y
    odom.twist.twist.angular.z = msg.angular.z
# revisar assinatura
cmd_sub = node.create_subscription(Twist, robot_namespace + "cmd_vel", cmb_sub_function, 10)

# if(static_tf)
# {
# odom2map = TransformStamped();
# odom2map.header.stamp = sim_node.now();
# odom2map.header.frame_id = "map";
# odom2map.child_frame_id = odom.header.frame_id;
# odom2map.transform.translation.x = pose.x;
# odom2map.transform.translation.y = pose.y;
# odom2map.transform.rotation.z = sin(pose.theta/2);
# odom2map.transform.rotation.w = cos(pose.theta/2);
# publishStaticTF(odom2map);
# }
# else
# {
#     pose_gt.header.frame_id = "map";
# }
# publish_gt = !static_tf;    

def move(dt):
    # wx = odom.twist.twist.angular.x
    # wy = odom.twist.twist.angular.y
    # wz = odom.twist.twist.angular.z

    # write actual covariance, proportional to velocity
    # odom.twist.covariance[0] = max(0.0001, np.abs(vx)*linear_noise*linear_noise)
    # odom.twist.covariance[7] = max(0.0001, np.abs(vy)*linear_noise*linear_noise)
    # odom.twist.covariance[35] = max(0.0001, np.abs(wz)*angular_noise*angular_noise)

    pose.updateFrom()
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
    rel_pose.R = Rotation.from_quat(odom_quat).as_matrix()
    # possivel fonte de erro
    rel_pose.t += (rel_pose.R*toMatrix(odom.twist.twist.linear)*dt).T
    rel_pose.R += rel_pose.R*skew(odom.twist.twist.angular)*dt
    qx, qy, qz, qw = Rotation.from_matrix(rel_pose.R).as_quat()[0]

    odom.pose.pose.position.x = pose.t[0, 0]
    odom.pose.pose.position.y = pose.t[1, 0]
    odom.pose.pose.position.z = pose.t[2, 0]
    odom.pose.pose.orientation.x = qx
    odom.pose.pose.orientation.y = qy
    odom.pose.pose.orientation.z = qz
    odom.pose.pose.orientation.w = qw

    # q = Rotation.from_matrix(pose.R).as_quat()
    # msg.transform.rotation.x = q[0]
    # msg.transform.rotation.y = q[1]
    # msg.transform.rotation.z = q[2]
    # msg.transform.rotation.w = q[3]
    # odom.pose.pose.orientation.z = sin(rel_pose.theta/2)
    # odom.pose.pose.orientation.w = cos(rel_pose.theta/2)

def publish_odom():
    # odom.header.stamp = transform.header.stamp = stamp;

    # build odom angle & publish as msg + tf
    odom_pub.publish(odom);

    #  build transform odom . base link
    odomTransform.transform.translation.x = odom.pose.pose.position.x
    odomTransform.transform.translation.y = odom.pose.pose.position.y
    odomTransform.transform.translation.z = odom.pose.pose.position.z
    odomTransform.transform.rotation = odom.pose.pose.orientation
    br.sendTransform(odomTransform)

    odom2map = TransformStamped()
    odom2map.header.frame_id = "world";
    odom2map.child_frame_id = odom.header.frame_id
    odom2map.transform.translation.x = pose.t[0, 0]
    odom2map.transform.translation.y = pose.t[1, 0]
    odom2map.transform.translation.y = pose.t[2, 0]
    qx, qy, qz, qw = Rotation.from_matrix(pose.R).as_quat()
    odom2map.transform.rotation.x = qx
    odom2map.transform.rotation.y = qy
    odom2map.transform.rotation.z = qz
    odom2map.transform.rotation.w = qw
    br.sendTransform(odom2map)

    if(publish_gt):
        pose_gt = TransformStamped()
        pose_gt.child_frame_id = odom.child_frame_id + "_gt"
        # pose_gt.header.stamp = stamp
        pose_gt.transform.translation.x = pose.t[0, 0]
        pose_gt.transform.translation.y = pose.t[1, 0]
        pose_gt.transform.translation.y = pose.t[2, 0]
        qx, qy, qz, qw = Rotation.from_matrix(pose.R).as_quat()
        pose_gt.transform.rotation.x = qx
        pose_gt.transform.rotation.y = qy
        pose_gt.transform.rotation.z = qz
        pose_gt.transform.rotation.w = qw

        br.sendTransform(pose_gt)

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

# const rclcpp::Time &
def refresh():
    move(dt)
    publish_odom()
    publish_ranges()

cmd_sub = node.create_subscription(Twist, 'cmd_vel', cmd_callback, 1)

range_pub = node.create_publisher(RangeWithCovariance, '/r2d2/ranges', 10)

def publish_ranges():
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
