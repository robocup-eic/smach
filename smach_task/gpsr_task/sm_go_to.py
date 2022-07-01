#!/usr/bin/env python

# smach
import roslib
import rospy
import smach
import smach_ros

# navigation
import tf2_ros
from nav_msgs.msg import Odometry
from math import pi
import tf
import tf2_msgs

# for furniture marker
import yaml
from visualization_msgs.msg import Marker , MarkerArray
from geometry_msgs.msg import Point, Pose

# SimpleActionClient
import actionlib

# realsense and computer vision
import requests
import cv2
import numpy as np
import pyrealsense2.pyrealsense2 as rs2
from geometry_msgs.msg import PoseStamped, Twist ,Vector3, TransformStamped
from std_msgs.msg import Bool,Int64
import socket

# import for text-to-speech
import requests
import json
import time

# ros pub sub
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import Bool
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo
from actionlib_msgs.msg import GoalStatus

# other utility
from util.custom_socket import CustomSocket
from util.nlp_server import SpeechToText, speak
from util.environment_descriptor import EnvironmentDescriptor
import time
import threading

class sm_go_to_Check_Object_Location(smach.State):
    def __init__(self, stt):
        rospy.loginfo('Initiating Check_object_location state')
        smach.State.__init__(self, outcomes = ['continue_sm_go_to_Navigation'],output_keys=['sm_go_to_Check_Object_Location_out'],)
        self.stt = stt
    def execute(self, userdata):
        rospy.loginfo('Executing Check_object_location state')
        while True:
            #print(self.stt.body)
            if self.stt.body is not None:
                # TODO check again with nlp
                print(self.stt.body)
                location = self.stt.body["intent"]
                userdata.sm_go_to_Check_Object_Location_out = location
                print(location)
                return 'continue_sm_go_to_Navigation'

class sm_go_to_Navigation(smach.State):
    def __init__(self, ed):
        rospy.loginfo('Initiating Navigation state')
        smach.State.__init__(self, outcomes = ['continue_sm_go_to_Announce'], input_keys=['sm_go_to_Navigation_in'])
        self.move_base_client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        self.ed = ed
    def execute(self, userdata):
        rospy.loginfo('Executing Navigation state')
        
        location = userdata.sm_go_to_Navigation_in

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now() - rospy.Duration.from_sec(1)
        goal.target_pose.pose = self.ed.get_robot_pose(location)
        self.move_base_client.send_goal(goal)
        self.move_base_client.wait_for_result()
        result = self.move_base_client.get_result()
        rospy.loginfo("result {}".format(result))
        if result.status == GoalStatus.SUCCESS :
            return 'continue_sm_go_to_Announce'
        else:
            # return abort TODO check http://docs.ros.org/en/fuerte/api/actionlib_msgs/html/msg/GoalStatus.html
            return 'continue_sm_go_to_Announce'

class sm_go_to_Announce(smach.State):
    def __init__(self):
        rospy.loginfo('Initiating Announce state')
        smach.State.__init__(self, outcomes = ['continue_succeeded', 'continue_aborted'])
        self.check = True
    def execute(self, userdata):
        rospy.loginfo("Executing Announce state")
        if self.check == True:
            speak("I'm arrive at the destination")
            return 'continue_succeeded'
        else:
            return 'continue_aborted'
