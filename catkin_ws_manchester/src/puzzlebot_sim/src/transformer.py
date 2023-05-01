#!/usr/bin/env python3

import rospy
import tf
import tf2_ros
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import PoseStamped, Twist, TransformStamped, Quaternion
import numpy as np


class tf_model:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.quat = Quaternion(*quaternion_from_euler(0,0,0))
        self.R = 0.05
        rospy.init_node('puzzlebot_tf')
        self.sub_pose = rospy.Subscriber('/pose_sim', PoseStamped, self.pose_cb)
        #self.publish_static()
        self.tf_broadcaster = tf.TransformBroadcaster()
    
    
    def pose_cb(self, msg):
        self.x = msg.pose.position.x 
        self.y = msg.pose.position.y 
        self.quat = msg.pose.orientation
    
    def run(self):
        dt = 0.1
        rate = rospy.Rate(1/dt)
        while not rospy.is_shutdown():
        
            pose_trans = TransformStamped()
            pose_trans.header.stamp = rospy.Time.now()
            pose_trans.header.frame_id = "map"
            pose_trans.child_frame_id = "base_link"

            pose_trans.transform.translation.x = self.x
            pose_trans.transform.translation.y = self.y
            pose_trans.transform.translation.z = self.R
            pose_trans.transform.rotation = self.quat
            self.tf_broadcaster.sendTransformMessage(pose_trans)
            rate.sleep()

if __name__ == "__main__":
    model = tf_model()
    model.run()