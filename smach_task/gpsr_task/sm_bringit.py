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
#from actionlib_msgs import GoalStatus

# other utility
from util.custom_socket import CustomSocket
from util.nlp_server import SpeechToText, speak
from util.environment_descriptor import EnvironmentDescriptor
import time
import threading
# import smach 
class  sm_bringit_Check_Object_Room_2(smach.State):
    def __init__(self):
        rospy.loginfo('initiating check object room 2 state')
        smach.State.__init__(self, outcomes = ['continue_sm_bringit_Save_Location'])
    def execute(self, userdata):
        rospy.loginfo('execute the check object room 2 state')
        return 'continue_sm_bringit_Save_Location'


class sm_bringit_Save_Location(smach.State):
    def __init__(self):
        rospy.loginfo('initiating save location state')
        smach.State.__init__(self, outcomes = ['continue_sm_bringit_Find_Object'])
    def execute(self, userdata):
        rospy.loginfo('execute save location state')
        return 'continue_sm_bringit_Find_Object'


class sm_bringit_Find_Object(smach.State):
    def __init__(self):
        rospy.loginfo('initiating find object state')
        smach.State.__init__(self, outcomes = ['continue_sm_bringit_Pick_Up','continue_aborted'])
        self.x = 1
    def execute(self, userdata):
        rospy.loginfo('execute find object state')
        if self.x == 1:
            return 'continue_sm_bringit_Pick_Up'
        else:
            return 'continue_aborted'


class sm_bringit_Pick_Up(smach.State):
    def __init__(self):
        rospy.loginfo('initiating pick up state')
        smach.State.__init__(self, outcomes = ['continue_sm_bringit_Back_To_Start'])
    def execute(self, userdata):
        rospy.loginfo('execute pick up state')
        return 'continue_sm_bringit_Back_To_Start'


class sm_bringit_Back_To_Start(smach.State):
    def __init__(self):
        rospy.loginfo('initiating back to start state')
        smach.State.__init__(self, outcomes = ['continue_sm_bringit_Place_Object'])
    def execute(self, userdata):
        rospy.loginfo('executing back to start state')
        return 'continue_sm_bringit_Place_Object'


class sm_bringit_Place_Object(smach.State):
    def __init__(self):
        rospy.loginfo('initiating place object state')
        smach.State.__init__(self, outcomes = ['continue_succeeded'])
    def execute(self, userdata):
        rospy.loginfo('executing place object state')
        return 'continue_succeeded'