#!/usr/bin/env python3

import rospy
import tf
import tf2_ros
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import PoseStamped, Twist, TransformStamped, Quaternion
import numpy as np


class k_model:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0

        self.v = 0.0
        self.w = 0.0

        rospy.init_node('puzzlebot_sim')
        self.pub_pose = rospy.Publisher('/pose_sim', PoseStamped, queue_size=10)
        self.sub = rospy.Subscriber('/cmd_vel', Twist, self.cmd_cb)
        #self.publish_static()
        self.tf_broadcaster = tf.TransformBroadcaster()
    
    
    def cmd_cb(self, msg):
        self.v = msg.linear.x
        
        self.w = msg.angular.z
    
    def run(self):
        dt = 0.1
        rate = rospy.Rate(1/dt)
        while not rospy.is_shutdown():
            self.th += self.w * dt

            self.x += self.v * np.cos(self.th) * dt
            self.y += self.v * np.sin(self.th) * dt
            present_time = rospy.Time.now()
            p = PoseStamped()
            p.header.frame_id = "map"
            p.header.stamp = present_time
            p.pose.position.x = self.x
            p.pose.position.y = self.y

            quat = Quaternion(*quaternion_from_euler(0,0,self.th))
            p.pose.orientation = quat
            self.pub_pose.publish(p)

            # first, we'll publish the transform over tf
            pose_trans = TransformStamped()
            pose_trans.header.stamp = present_time
            pose_trans.header.frame_id = "map"
            pose_trans.child_frame_id = "base_link"

            pose_trans.transform.translation.x = self.x
            pose_trans.transform.translation.y = self.y
            pose_trans.transform.rotation = quat
            self.tf_broadcaster.sendTransformMessage(pose_trans)
            rate.sleep()

if __name__ == "__main__":
    model = k_model()
    model.run()