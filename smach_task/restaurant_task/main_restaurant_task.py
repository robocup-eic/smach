#!/usr/bin/env python

"""
$ source walkie2_ws/devel/setup.bash
$ roslaunch realsense2_camera rs_rgbd.launch align_depth:=true color_width:=1280 color_height:=720 color_fps:=30 depth_width:=1280 depth_height:=720 depth_fps:=30 filters:=pointcloud
kill flask in background
$ kill -9 $(lsof -t -i:5000)
"""

import rospy
import smach
import smach_ros

# navigation
import tf2_ros
from nav_msgs.msg import Odometry
from math import pi
import tf
import tf2_msgs
import tf2_geometry_msgs
from geometry_msgs.msg import Twist
from actionlib_msgs.msg import GoalStatus

# ros pub sub
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import Bool
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo

# SimpleActionClient
import actionlib

# import for speed-to-text
from flask import Flask, request
import threading

# realsense and computer vision
import requests
import numpy as np
import cv2
from geometry_msgs.msg import PoseStamped, Twist ,Vector3, TransformStamped, Pose
from std_msgs.msg import Bool,Int64
import socket
from util.custom_socket import CustomSocket
from util.realsense import Realsense

# import for text-to-speech
import requests
import json
from util.nlp_server import SpeechToText, speak
import time

# import yaml reader
from util.guest_name_manager import GuestNameManager
from util.environment_descriptor import EnvironmentDescriptor

class go_to_Navigation():
    def __init__(self):
        self.move_base_client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    
    def move(self,location):
        global ed
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now() - rospy.Duration.from_sec(1)
        goal.target_pose.pose = ed.get_robot_pose(location)
        self.move_base_client.send_goal(goal)
        self.move_base_client.wait_for_result()
        while True:
            result = self.move_base_client.get_state()
            rospy.loginfo("status {}".format(result))
            if result == GoalStatus.SUCCEEDED :
                return True
            else:
                return False
#---------------------------------------------------------------
class Walkie_Rotate(smach.State) :
    def __init__(self):
        rospy.loginfo('Initiating Walkie_Rotate state')
        smach.State.__init__(self,outcomes=['continue_Walkie_Speak'])
        self.rotate_pub = rospy.Publisher("/walkie2/cmd_vel", Twist, queue_size=10)
    
    def execute(self,userdata):
        rospy.loginfo('Executing Walkie_Rotate state')
        # find people raising hand
        rotate_msg = Twist()
        rotate_msg.angular.z = 0.1

        is_found = False
        while not is_found:
            self.rotate_pub.publish(rotate_msg)
            detections = HandRaising(rs.get_image())
            for people in detections:

                x_relative = people["x"] + (people["w"] / 2)

                if (people["hand_raised"] == True) and (400 < x_relative < 800) :
                    cancel = Twist()
                    cancel.linear.x = 0
                    cancel.linear.y = 0
                    cancel.angular.z = 0
                    is_found = True

        self.rotate_pub.publish(cancel)
        rospy.sleep(1)

        return 'continue_Walkie_Speak'
            
class Walkie_Speak(smach.State) :
    def __init__(self):
        rospy.loginfo('Initiating Walkie_Speak state')
        smach.State.__init__(self,outcomes=['continue_succeed'])
    
    def execute(self,userdata):
        rospy.loginfo('Executing Walkie_Speak state')

        speak("Okey I found")
        speak("I will follow on you")

        return 'continue_succeed'
#---------------------------------------------------------------

if __name__ == '__main__':
    rospy.init_node('restaurant_task')

    tf_Buffer = tf2_ros.Buffer()

    ed = EnvironmentDescriptor("../config/fur_data_onsite.yaml")
    ed.visual_robotpoint()

    # connect to server
    host = "0.0.0.0"
    # host = socket.gethostname()
    # hand raising detection
    port_HandRaising = 10011
    HandRaising = CustomSocket(host, port_HandRaising)
    HandRaising.clientConnect()

    rs = Realsense()
    rs.wait() # wait for camera intrinsics

    # Flask nlp server
    stt = SpeechToText("nlp")
    t = threading.Thread(target = stt.run ,name="nlp")
    t.start()

    # Create a SMACH state machine
    sm_top = smach.StateMachine(outcomes=['SUCCEEDED'])

    # smach userdata

    # Open the container
    with sm_top:
        # Add states to the container
        # smach.StateMachine.add('Turn_Around_Walkie', Turn_Around_Walkie(),
        #                         transitions={'continue_Find_Hand_raising':'Find_Hand_raising'})
        smach.StateMachine.add('Turn_Around_Walkie', Walkie_Rotate(),
                                transitions={'continue_Walkie_Speak':'Walkie_Speak'})

        smach.StateMachine.add('Walkie_Speak', Walkie_Speak(),
                                transitions={'continue_succeed':'SUCCEEDED'})

    sis = smach_ros.IntrospectionServer('Server_name', sm_top, '/Restaurant_task')
    sis.start()
    # Execute SMACH plan
    outcome = sm_top.execute()
    rospy.spin()
    sis.stop()