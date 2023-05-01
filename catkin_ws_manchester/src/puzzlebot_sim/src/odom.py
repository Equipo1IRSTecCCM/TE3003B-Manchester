#!/usr/bin/env python3

import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion
from std_msgs.msg import Float32
from math import cos, sin
import numpy as np
from puzzlebot_info import *


#!/usr/bin/env python3

import rospy
import tf
import tf2_ros
from tf.transformations import quaternion_from_euler
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
import numpy as np


class k_model:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.v = 0.0
        self.w = 0.0
        self.wr = 0.0
        self.wl = 0.0

        rospy.init_node('puzzlebot_deadReckoning')
        self.pub_odom = rospy.Publisher('/odom', Odometry, queue_size=10)
        self.wl_sub = rospy.Subscriber('/wl', Float32, self.wr_cb)
        self.wr_sub = rospy.Subscriber('/wr', Float32, self.wl_cb)

    
    
    def wr_cb(self, msg):
        self.wr = msg.data

    def wl_cb(self, msg):
        self.wl = msg.data
    
    def run(self):
        dt = 0.1
        rate = rospy.Rate(1/dt)
        while not rospy.is_shutdown():
            self.w = R * (self.wr - self.wl) / L
            self.v = R * (self.wr + self.wl) * 0.5
            
            self.th += self.w * dt
            self.x += self.v * np.cos(self.th) * dt
            self.y += self.v * np.sin(self.th) * dt

            present_time = rospy.Time.now()
            o = Odometry()
            o.header.frame_id = "map"
            o.child_frame_id = "base_link"
            o.header.stamp = present_time
            o.pose.pose.position.x = self.x
            o.pose.pose.position.y = self.y

            quat = Quaternion(*quaternion_from_euler(0,0,self.th))
            o.pose.pose.orientation = quat

            o.twist.twist.linear.x = self.v
            o.twist.twist.angular.z = self.w
            self.pub_odom.publish(o)
            rate.sleep()

if __name__ == "__main__":
    model = k_model()
    model.run()


