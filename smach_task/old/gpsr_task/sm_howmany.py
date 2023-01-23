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
# declare the class in python
class sm_howmany_Check_Location(smach.State):
    def __init__(self):
        rospy.loginfo('initiating check location state')
        smach.State.__init__(self, outcomes = ['continue_sm_howmany_Navigation'])
    def execute(self, userdata):
        rospy.loginfo('executing check location state')
        return 'continue_sm_howmany_Navigation'

class sm_howmany_Navigation(smach.State):
    def __init__(self):
        rospy.loginfo('initiating navigation state')
        smach.State.__init__(self, outcomes = ['continue_sm_howmany_Object_Detection'])
    def execute(self, userdata):
        rospy.loginfo('executing navigation state')
        return 'continue_sm_howmany_Object_Detection'

class sm_howmany_Object_Detection(smach.State):
    def __init__(self):
        rospy.loginfo('initiating object detection state')
        smach.State.__init__(self, outcomes = ['continue_aborted', 'continue_succeeded'])
        self.x = 1
    def execute(self, userdata):
        rospy.loginfo('executing navigation state')
        if self.x == 1:
            return 'continue_aborted'
        else:
            return 'continue_succeeded'