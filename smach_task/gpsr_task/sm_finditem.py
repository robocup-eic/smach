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
from http.client import responses

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
class sm_finditem_Check_Object_Room(smach.State):
    def __init__(self):
        rospy.loginfo('initiating check object room state')
        smach.State.__init__(self, outcomes = ['continue_sm_finditem_Navigate_To_Room'])
        self.x = True
    def execute(self, userdata):
        rospy.loginfo('exetuting check object room')
        return 'continue_sm_finditem_Navigate_To_Room'


class sm_finditem_Navigate_To_Room(smach.State):
    def __init__(self):
        rospy.loginfo('initiating navigate to room state')
        smach.State.__init__(self, outcomes = ['continue_sm_finditem_Find_Object', 'continue_aborted'])
        self.x = True
    def execute(self, userdata):
        rospy.loginfo('executing navigate to room')
        if self.x == True:
            return 'continue_sm_finditem_Find_Object'
        else:
            return 'continue_aborted'


class sm_finditem_Find_Object(smach.State):
    def __init__(self, host, port):
        rospy.loginfo('initiating find object state')
        smach.State.__init__(self, outcomes = ['continue_sm_finditem_Announce'])
        self.host = host
        self.port = port
        
    def execute(self, userdata):
        self.c = CustomSocket(host, port):
        self.clientConnect()
        #cap = cv2.VideoCapture(2)
        self.cap.set(4,720)
        self.cap.set(3,1280)
        self.cap = cv2.VideoCapture(2)

        while self.cap.isOpened():
	
            self.ret, self.frame = self.cap.read()
            if not self.ret:
                print("Ignoring empty camera frame.")
                continue
    
            self.cv2.imshow('test', self.frame)

            print("Send")

            # WIT yolov5 person-tracker, object-tracker
            self.msg = self.c.req(frame)
            print(self.msg)


            # Face-recog
            # image = cv2.imread("elon.jpeg")
            # c.register(image, "elon") #hide this
            # c.detect(frame)

            if self.cv2.waitKey(1) == ord("q"):
                self.cap.release()
        
        self.cv2.destroyAllWindows()

        rospy.loginfo('executing find object')
        return 'continue_sm_finditem_Announce'


class sm_finditem_Announce(smach.State):
    def __init__(self):
        rospy.loginfo('initiating announce state')
        smach.State.__init__(self, outcomes = ['continue_succeeded'])
    def execute(self, userdata):
        rospy.loginfo('executing announce state')
        return 'continue_succeeded'
