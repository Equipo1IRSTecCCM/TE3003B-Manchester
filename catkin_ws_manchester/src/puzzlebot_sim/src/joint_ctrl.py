#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64

def talker():
    pub = rospy.Publisher('/base_to_right_w_velocity_controller/command', Float64, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        velocity = 0.5 # rad/s
        pub.publish(velocity)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass