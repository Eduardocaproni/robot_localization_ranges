#!/usr/bin/env python3
'''
    Internal simulation of steering-wheel robots

'''
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster, TransformListener, Buffer
from anchor_msgs.msg import RangeWithCovariance
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
import numpy as np

dt = 0.1

class Anchor:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.range = 0.
                
class Trilat(Node):
    def __init__(self):
        super().__init__('trilateration')
        
        self.mMr = [0.,0.,0.]
        
        # get anchor names
        anchors = self.declare_parameter('range0_anchors', ['']).value
        
        # init actual anchors
        self.anchors = {}
        for anchor in anchors:
            x = self.declare_parameter(f'range0_{anchor}.pose.x', 0.).value
            y = self.declare_parameter(f'range0_{anchor}.pose.y', 0.).value
            self.anchors[anchor] = Anchor(x, y)

        self.get_logger().info(f'Found {len(anchors)} anchors')
        
        self.range_sub = self.create_subscription(RangeWithCovariance, 'ranges', self.range_callback, max(1, len(anchors)))
        
        self.twist = Twist()
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 2)
        

        self.br = TransformBroadcaster(self)        
        self.pose = TransformStamped()
        self.pose.header.frame_id = 'map'
        
        self.buff = Buffer()
        self.tf = TransformListener(self.buff, self)
        self.robot = None
                            
        self.timer = self.create_timer(dt, self.timer_callback)

    def timer_callback(self):
                
        # get all anchors with non-0 range
        valid = [anchor for name,anchor in self.anchors.items() if anchor.range != 0.]
        n = len(valid)
        
        if n < 3:
            return

        now = rclpy.time.Time()
        
        # useless if no odom yet, cannot correct
        if self.robot is None or not self.buff.can_transform(self.robot, self.pose.child_frame_id, now):
            self.get_logger().info('No odom yet')
            return
        
        # build pose estimate map -> robot
        A = np.matrix(np.zeros((n-1,2)))
        B = np.matrix(np.zeros((n-1,1)))
        
        r0 = valid[0].range
        x0 = valid[0].x
        y0 = valid[0].y

        for row,anchor in enumerate(valid[1:]):
            xi,yi,r = anchor.x, anchor.y, anchor.range
            #self.get_logger().info(f' @ {r} from ({xi},{yi})')
            
            A[row,0] = xi-x0
            A[row,1] = yi-y0
            B[row,0] = xi**2-x0**2 + yi**2-y0**2 - (r**2-r0**2)
        #self.get_logger().info(f'A: {A}')
        #self.get_logger().info(f'b: {B}')
        xr,yr = (0.5*np.linalg.pinv(A)*B).T.tolist()[0]

        # estimate angle if v is available
        if self.twist.linear.x or self.twist.linear.y:
            vx = self.twist.linear.x
            vy = self.twist.linear.y
            
            dx = (xr - self.mMr[0])/dt
            dy = (yr - self.mMr[1])/dt
            
            cst = dx*vx + dy*vy
            det = np.sqrt(dx**2*vx**2 + dx**2*vy**2 + dy**2*vx**2 + dy**2*vy**2)
            denom = dx*vy - dy*vx
            
            self.mMr[2] = min((2*np.arctan2(cst+det,denom), 2*np.arctan2(cst-det,denom)),
                              key = lambda t: (-dx + vx*np.cos(t) - vy*np.sin(t))**2 + (-dy + vx*np.sin(t) + vy*np.cos(t))**2)            
        else:
            self.mMr[2] += self.twist.angular.z*dt
            
        # update absolute position
        
        xr = self.mMr[0] = 0.5*(self.mMr[0]+xr)
        yr = self.mMr[1] = 0.5*(self.mMr[1]+yr)
        tr = self.mMr[2]
                        
        # correct odometry
        rMo = self.buff.lookup_transform(self.robot, self.pose.child_frame_id, now)
        xo = rMo.transform.translation.x
        yo = rMo.transform.translation.y
        to = 2*np.arctan2(rMo.transform.rotation.z, rMo.transform.rotation.w)
        
        self.pose.transform.translation.x = xo*np.cos(tr) + xr - yo*np.sin(tr)
        self.pose.transform.translation.y = xo*np.sin(tr) + yo*np.cos(tr) + yr
        self.pose.transform.rotation.z = np.sin((to+tr)/2.)
        self.pose.transform.rotation.w = np.cos((to+tr)/2.)        
                
        # publish mMo        
        self.pose.header.stamp = self.get_clock().now().to_msg()
        self.br.sendTransform(self.pose)                
        
    def odom_callback(self, msg):
        self.twist = msg.twist.twist
                
    def range_callback(self, msg):
        frame = msg.header.frame_id
        if frame not in self.anchors:
            return
        
        if self.pose.child_frame_id == '':
            
            self.robot = msg.moving_frame
            
            if '/' in self.robot:
                slash = self.robot.index('/')
                self.pose.child_frame_id = self.robot[:slash] + '/odom'
            else:
                self.pose.child_frame_id = 'odom'

        if msg.range_min <= msg.range <= msg.range_max:
            self.anchors[frame].range = msg.range
        else:
            self.anchors[frame].range = 0.

rclpy.init()

trilat = Trilat()

rclpy.spin(trilat)

robot.destroy_node()
