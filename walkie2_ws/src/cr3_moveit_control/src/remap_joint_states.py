#!/usr/bin/env python

"""
subscribe /walkie/joint_states
publish /joint_states
"""

# ros library
import rospy
from sensor_msgs.msg import JointState

# other library
import time
import math
import random

def joint_state_cb(data) :
    global joint_state
    joint_state = data

if __name__ == '__main__' :

    rospy.init_node('remap_joint_states')

    global joint_state

    joint_state = JointState()

    pub = rospy.Publisher("/joint_states", JointState, queue_size=10)
    rospy.Subscriber("/walkie2/joint_states", JointState, joint_state_cb, queue_size=1)

    rate = rospy.Rate(50) # 10hz
    while not rospy.is_shutdown():
        try:
            rospy.loginfo("joint_state")
            pub.publish(joint_state)
            rate.sleep()

        except KeyboardInterrupt: break
