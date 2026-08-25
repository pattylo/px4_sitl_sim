#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
import numpy as np
import math
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from geometry_msgs.msg import PointStamped
from std_msgs.msg import ColorRGBA
from scipy.spatial.transform import Rotation 
from .model.kf import kf
from .model.spring import spring

from collections import deque

from geometry_msgs.msg import Twist
import csv
from datetime import datetime
import os
from std_msgs.msg import Bool
# from scout_msgs.msg import AcousticEst

def q2rpy(q):
    r = Rotation.from_quat(q)
    roll, pitch, yaw = r.as_euler('xyz', degrees=False)
    return roll, pitch, yaw

def wrap_to_pi(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi

class acousticKF(Node):
    """acousticKF"""
    def __init__(self):
        super().__init__("acousticKF")
        
        qos = QoSProfile(depth=10)
        qos.reliability = QoSReliabilityPolicy.BEST_EFFORT
        
        self.spring = spring()
        
        self.follower_pos_sub = self.create_subscription(
            PoseStamped, 
            f'/follower/mavros/local_position/pose', 
            self.follower_pose_callback, 
            qos
        )
        self.follower_pos = np.array([])
        self.follower_pose_msg = PoseStamped()
        
        # RELTAED TO ACOUSTIC SHIT
        self.acoustic_inference_sub = self.create_subscription(
            Point, 
            f'/acoustic/inference', 
            self.acoustic_inference_callback, 
            qos
        )
        
        self.acoustic_inference_pub = self.create_publisher(
            PointStamped,
            '/acoustic/guess',
            10
        )
        self.acoustic_kf_inference_pub = self.create_publisher(
            PointStamped,
            '/acoustic/kf_guess',
            10
        )
        
        self.traj_reset_pub = self.create_publisher(
            Bool,
            '/acoustic/traj_reset',
            10
        )

        self.acoustic_guess = PointStamped()
        self.acoustic_guess_kf = PointStamped()

        self.acoustic_start = False
        self.kf = kf()
        self.lala_i = 0                
                        
        self.leader_pos_start = False
        self.follower_pos_start = False
        self.follower_pos_gazebo_start = False
        
        
        # self.acoustic_msg_pub = self.create_publisher(
        #     AcousticEst,
        #     '/acoustic/kf_all',
        #     10
        # )
        # self.acoustic_msg = AcousticEst()
        
        # for viz convenience
        for i in range(10):
            msg_temp = Bool()
            msg_temp.data = True
            self.traj_reset_pub.publish(msg_temp)
        
    def acoustic_inference_callback(self, msg: Point):      
          
        if not self.follower_pos_start:
            return       
        
        bearing = msg.x - 5 # best param for new model
        range = msg.y + 0.2 # best param for new model
        # bearing = msg.x * 1.1 # best param for 0408
        # range = msg.y - 0.17 # best param for 0408
        # Clip bearing (degrees) to [-90, 90]
        clipped_bearing = float(np.clip(bearing, -90.0, 90.0))
        # clipped_bearing = float(np.clip(bearing, -180.0, 180.0))
        
        r_rel_B = np.array([
            # range * np.cos(bearing / 180.0 * np.pi), 
            # range * np.sin(bearing / 180.0 * np.pi),
            range * np.cos(clipped_bearing / 180.0 * np.pi), 
            range * np.sin(clipped_bearing / 180.0 * np.pi),
            0
        ])
        
        _, _, ego_yaw = q2rpy([
            self.follower_pose_msg.pose.orientation.x,
            self.follower_pose_msg.pose.orientation.y,
            self.follower_pose_msg.pose.orientation.z,
            self.follower_pose_msg.pose.orientation.w
        ])            

        r_rel_W = self.spring.rot3D(0,0,ego_yaw) @ r_rel_B + self.follower_pos
                
        self.acoustic_guess.point.x = r_rel_W[0].copy()
        self.acoustic_guess.point.y = r_rel_W[1].copy()
        
        
        if not self.acoustic_start:
            self.acoustic_start = True
            self.kf.kf_init(
                x0=r_rel_W[:2].copy(),
                dt=0.3
            )
            self.origin = r_rel_W[:2].copy()

            return
        
        self.kf.predict()
        self.kf.update(
            # z=np.array([bearing/180*np.pi, range]), 
            z=np.array([clipped_bearing/180*np.pi, range]), 
            x_ego=np.array([self.follower_pos[0], self.follower_pos[1]]),
            yaw_ego=ego_yaw
        )            
        
        # 1207 here
        self.get_logger().info("ha")
        self.pointstamped_pub(
            self.acoustic_inference_pub,
            [float(r_rel_W[0]), float(r_rel_W[1]), self.follower_pos[2]]
        )
        # self.pointstamped_pub(
        #     self.acoustic_inference_pub,
        #     [self.kf.xk[0], self.kf.xk[1], self.follower_pos[2]]
        # )
        self.get_logger().info("gan")
        
        
        # self.get_logger().info(f"d2: {self.kf.d2}")
        
        # self.acoustic_msg.kf_est_pos = [self.kf.xk[0], self.kf.xk[1], self.follower_pos[2]]
        # self.acoustic_msg.kf_est_vel = self.kf.xk[2:4].copy()
        # self.acoustic_msg.raw_obs = r_rel_W[:2].copy()
        # self.acoustic_msg.d2 = self.kf.d2
        # self.acoustic_msg_pub.publish(self.acoustic_msg)
                
    
    def follower_pose_callback(self, msg):
        self.follower_pose_msg = msg
        self.follower_pos_start = True
        self.follower_pos = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z
        ])
        
    def pointstamped_pub(self, publisher, pt):
        pt_obj = PointStamped()
        pt_obj.header.frame_id = 'map'
        pt_obj.point.x = pt[0]
        pt_obj.point.y = pt[1]
        pt_obj.point.z = pt[2]
        
        publisher.publish(pt_obj)
        return

def main(args=None):
    rclpy.init(args=args)
    dummy = acousticKF()
    try:
        rclpy.spin(dummy)
    finally:
        if hasattr(dummy, 'csv_file') and dummy.csv_file:
            dummy.csv_file.close()
            dummy.get_logger().info("CSV file closed")
        dummy.destroy_node()
        rclpy.shutdown()
    
if __name__ == '__main__':
    main()