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
class sm_find_person_Save_Location(smach.State):
    def __init__(self):
        rospy.loginfo('initiating save location state')
        smach.State.__init__(self, outcomes = ['continue_sm_find_person_Ask_Which_Room'])
    def execute(self, userdata):
        rospy.loginfo("Executing Save location state")
        return 'continue_sm_find_person_Ask_Which_Room'


class sm_find_person_Ask_Which_Room(smach.State):
    def __init__(self):
        rospy.loginfo('initiating ask which room state')
        smach.State.__init__(self, outcomes = ['continue_sm_find_person_Go_To_Room_1','continue_sm_find_person_Go_To_Specified_Room'])
        self.x = 1
    def execute(self, userdata):
        rospy.loginfo('Executing person ask which room')
        if self.x == 1:
            return 'continue_sm_find_person_Go_To_Room_1'
        else:
            return 'continue_sm_find_person_Go_To_Specified_Room'


class sm_find_person_Go_To_Room_1(smach.State):
    def __init__(self):
        rospy.loginfo('initiating go to room state')
        smach.State.__init__(self, outcomes = ['continue_sm_find_person_Find_Item'])
    def execute(self, userdata):
        rospy.loginfo('Executing go to room 1')
        return 'continue_sm_find_person_Find_Item'


class sm_find_person_Find_Item(smach.State):
    def __init__(self):
        rospy.loginfo('initiating find item')
        smach.State.__init__(self, outcomes = ['continue_sm_find_person_Go_To_Room_2', 'continue_aborted', 'continue_sm_find_person_Back_To_Start'])
        self.x = 1
    def execute(self, userdata):
        rospy.loginfo('Executing find item')
        if self.x == 1:
            return 'continue_sm_find_person_Go_To_Room_2'
        elif self.x == 2:
            return 'continue_aborted'
        else:
            return 'continue_sm_find_person_Back_To_Start'

class sm_find_person_Go_To_Room_2(smach.State):
    def __init__(self):
        rospy.loginfo('initiating go to rooom 2 state')
        smach.State.__init__(self, outcomes = ['continue_sm_find_person_Find_Item'])
    def execute(self, userdata):
        rospy.loginfo('Executing go to room 2')
        return 'continue_sm_find_person_Find_Item'

class sm_find_person_Back_To_Start(smach.State):
    def __init__(self):
        rospy.loginfo('initiating back to start state')
        smach.State.__init__(self, outcomes = ['continue_sm_find_person_Tell_Where_Is_Person'])
    def execute(self, userdata):
        rospy.loginfo('Executing back to start')
        return 'continue_sm_find_person_Tell_Where_Is_Person'

class sm_find_person_Go_To_Specified_Room(smach.State):
    def __init__(self):
        rospy.loginfo('initiating go to specified room')
        smach.State.__init__(self, outcomes = ['continue_sm_find_person_Find_Item'])
    def execute(self, userdata):
        rospy.loginfo('Executing Go to speciffied room')
        return 'continue_sm_find_person_Find_Item'

class sm_find_person_Tell_Where_Is_Person(smach.State):
    def __init__(self):
        rospy.loginfo('initiating tell where is person state')
        smach.State.__init__(self, outcomes = ['continue_succeeded'])
    def execute(self, userdata):
        rospy.loginfo('Executing tell where is the person')
        return 'continue_succeeded'