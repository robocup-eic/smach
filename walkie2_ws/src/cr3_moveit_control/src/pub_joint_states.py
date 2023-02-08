#!/usr/bin/env python

# ros library
from ast import Sub
from pickle import TRUE
import rospy
from std_msgs.msg import Bool, Int16, Float32
from sensor_msgs.msg import JointState

# other library
import time
import math
import random

def to_rad(deg):
    return deg / 180.0 * math.pi
    
def realsense_pitch_cb(data):
    global realsense_pitch_angle
    realsense_pitch_angle = to_rad(data.data)

def realsense_yaw_cb(data):
    global realsense_yaw_angle
    realsense_yaw_angle = to_rad(data.data)

def done_cb(data):
    global done
    done = data.data

def lift_cb(data):
    global lift_state
    lift_state = data.data

if __name__ == '__main__' :

    rospy.init_node('pub_joint_states')
    global realsense_pitch_angle
    realsense_pitch_angle = 0

    global realsense_yaw_angle
    realsense_yaw_angle = 0

    global lift_state
    lift_state = 0.0

    pub = rospy.Publisher("/realsense/joint_states", JointState, queue_size=100)
    rospy.Subscriber("/realsense_pitch_angle", Int16, realsense_pitch_cb, queue_size=100)
    rospy.Subscriber("/realsense_yaw_angle", Int16, realsense_yaw_cb, queue_size=100)
    rospy.Subscriber("/lift_state", Float32, lift_cb, queue_size=100)

    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        try:
            msg = JointState()
            msg.name = ["realsense_yaw_joint", "realsense_pitch_joint", "base_cr3_joint"]
            # Initialize the time of publishing
            msg.header.stamp = rospy.Time.now()
            # Joint angle values
            msg.position = [realsense_yaw_angle, -realsense_pitch_angle, lift_state]
            # rospy.loginfo(msg.position)
            pub.publish(msg)
            rate.sleep()

        except KeyboardInterrupt: break
