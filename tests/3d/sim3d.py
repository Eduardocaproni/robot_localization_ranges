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
base_link = ''

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

odom = Odometry()
odom_pub = node.create_publisher(Odometry, robot_namespace + "odom", 10)
transform = TransformStamped() # ?
odom.header.frame_id = transform.header.frame_id = link_prefix + "odom";
odom.child_frame_id = transform.child_frame_id = link_prefix + base_link;

def cmb_sub_function(msg):
    odom.twist.twist.linear.x = msg.linear.x
    odom.twist.twist.linear.y = msg.linear.y
    odom.twist.twist.angular.z = msg.angular.z
# revisar assinatura
cmd_sub = node.create_subscription(Twist, robot_namespace + "cmd_vel", cmb_sub_function, 10)

# if(static_tf)
# {
# odom2map = TransformStamped();
# odom2map.header.stamp = sim_node->now();
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
    vx = odom.twist.twist.linear.x
    vy = odom.twist.twist.linear.y
    vz = odom.twist.twist.linear.z
    wx = odom.twist.twist.angular.x
    wy = odom.twist.twist.angular.y
    wz = odom.twist.twist.angular.z

    # write actual covariance, proportional to velocity
    odom.twist.covariance[0] = std::max(0.0001, std::abs(vx)*linear_noise*linear_noise)
    odom.twist.covariance[7] = std::max(0.0001, std::abs(vy)*linear_noise*linear_noise)
    odom.twist.covariance[35] = std::max(0.0001, std::abs(wz)*angular_noise*angular_noise)

    pose.updateFrom(vx, vy, wz, dt)

    # add noise: command velocity to measured (odometry) one
    # vx *= (1+linear_noise*unit_noise(random_engine))
    # vy *= (1+linear_noise*unit_noise(random_engine))
    # vz *= (1+linear_noise*unit_noise(random_engine))
    # wx *= (1+linear_noise*unit_noise(random_engine))
    # wy *= (1+linear_noise*unit_noise(random_engine))
    # wz *= (1+angular_noise*unit_noise(random_engine))

    #  update noised odometry
    rel_pose = Pose()
    # lidar com quaternions
    rel_pose =  {odom.pose.pose.position.x, odom.pose.pose.position.y, 2*atan2(odom.pose.pose.orientation.z, odom.pose.pose.orientation.w)}
    rel_pose.updateFrom(vx, vy, wz, dt)

    odom.pose.pose.position.x = rel_pose.x
    odom.pose.pose.position.y = rel_pose.y
    odom.pose.pose.position.y = rel_pose.y

    q = Rotation.from_matrix(pose.R).as_quat()
    msg.transform.rotation.x = q[0]
    msg.transform.rotation.y = q[1]
    msg.transform.rotation.z = q[2]
    msg.transform.rotation.w = q[3]
    odom.pose.pose.orientation.z = sin(rel_pose.theta/2)
    odom.pose.pose.orientation.w = cos(rel_pose.theta/2)

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

def publish_ranges():
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
# const rclcpp::Time &
def refresh(now):
    # Robot::refreshStamp();
    # for robot in robots:
        # if(robot.connected()):
    move(dt)
    publish_ranges()

    # grid.computeLaserScans(robots);
    # if(last_tf != now.nanoseconds()):
    #     for robot in robots:
    #         if(robot.connected()):
    #             robot.publish(br);
    #     last_tf = now.nanoseconds();

refresh_timer = create_wall_timer(milliseconds(static_cast<long>(1000*dt)), [&](){refresh(now());})


rclpy.spin(node)

node.destroy_node()
rclpy.shutdown()
