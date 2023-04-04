#!/usr/bin/env python3

from geometry_msgs.msg import TransformStamped
from scipy.spatial.transform import Rotation

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

import rclpy
from rclpy.node import Node

rclpy.init(args=None)
node = Node('transform_listener') #tirar isso

class FrameListener(Node):
    def __init__(self, head_frame='r2d2/base_footprint_gt', child_frame='r2d2/base_footprint'):
        super().__init__('FrameListener')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.head_frame = head_frame
        self.child_frame = child_frame

        self.timer = self.create_timer(dt, self.ListentoTrans)

        self.t = []
        self.error = []
    
    def ListentoTrans(self):
        try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform(self.child_frame, 
                self.head_frame, 
                now)
        except TransformException as ex:
            # self.get_logger().info(
            #     f'Could not transform {self.child_frame} to {self.head_frame}: {ex}')
            return
        
        error = np.sqrt(trans.transform.translation.x**2 + trans.transform.translation.y**2 + trans.transform.translation.z**2)
        self.t.append(trans.header.stamp.sec+trans.header.stamp.nanosec/(10**9))
        self.error.append(error)

        if(len(self.t)==300):
            self.t = np.array(self.t)
            self.t = self.t - self.t[0]
            # np_data = np.array([self.t,self.error])
            # df = pd.DataFrame(np_data, columns = ['t','error'])
            # df.to_csv('error.csv', index=False)

            fig, ax = plt.subplots()
            ax.plot(self.t, self.error, marker='.')
            ax.set_xlabel('Time [s]')
            ax.set_ylabel('error [m]')
            ax.grid(True)
            fig.savefig("Error_vitesse_cv_e-3")
            print("figure created")
            #self.timer.cancel()





dt = 0.02

node = FrameListener()


rclpy.spin(node)

node.destroy_node()
rclpy.shutdown()
