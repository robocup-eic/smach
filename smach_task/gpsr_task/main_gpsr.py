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
from util.realsense import Realsense
import time
import threading

# import class for smach
from sm_bringit import sm_bringit_Check_Object_Room_2, sm_bringit_Save_Location, sm_bringit_Find_Object
from sm_bringit import sm_bringit_Pick_Up, sm_bringit_Back_To_Start, sm_bringit_Place_Object
from sm_finditem import sm_finditem_Check_Object_Room, sm_finditem_Navigate_To_Room, sm_finditem_Find_Object, sm_finditem_Announce
from sm_find_person import sm_find_person_Save_Location, sm_find_person_Ask_Which_Room, sm_find_person_Go_To_Room_1
from sm_find_person import sm_find_person_Go_To_Room_2, sm_find_person_Go_To_Specified_Room
from sm_find_person import sm_find_person_Find_Item, sm_find_person_Back_To_Start, sm_find_person_Tell_Where_Is_Person
from sm_go_to import sm_go_to_Check_Object_Location, sm_go_to_Navigation, sm_go_to_Announce
from sm_howmany import sm_howmany_Check_Location, sm_howmany_Navigation, sm_howmany_Object_Detection
# LAN local network
# import smach 
class Stand_by(smach.State):
    def __init__(self, stt):
        rospy.loginfo('Initiating Standby state')
        smach.State.__init__(self, outcomes =['continue_SM_BRINGIT','continue_SM_FINDITEM','continue_SM_FIND_PERSON', 'continue_SM_GO_TO', 'continue_SM_HOWMANY'])
        self.x = 4
        self.stt = stt
        self.stt.body = None

        self.FRAME_COUNT_LIMIT = 5
        self.close_distance = 1 # meter

        self.moving_pub = rospy.Publisher("/walkie2/cmd_vel", Twist, queue_size=10)
        self.moving_msg = Twist()
        self.moving_msg.linear.x = 0.2

    def execute(self, userdata):
        rospy.loginfo("Executing Standby state")
        # if self.x == 4:
        #     return "continue_SM_FINDITEM"

        global rs

        #Detect door open from the depth at the center of the frame
        x_pixel, y_pixel = 1280/2, 720/2
        frame_count = 0

        while True:
            rospy.sleep(0.5)
            distance = rs.get_coordinate(x_pixel, y_pixel)[2]
            rospy.loginfo(distance)
            # filter lower distance
            if distance < 0.4:
                continue
            # check if have available frame consecutively
            if frame_count >= self.FRAME_COUNT_LIMIT:
                speak("door open")

                #Moving through entrance door
                start_time = time.time()
                while time.time() - start_time < 4:
                    rospy.loginfo("Moving Forward...")
                    self.moving_pub.publish(self.moving_msg)
                    rospy.sleep(0.1)
                
                self.moving_msg.linear.x = 0
                self.moving_pub.publish(self.moving_msg)
                break
            if distance > self.close_distance:
                frame_count += 1
            else:
                frame_count = 0

        

        speak("Please state new command")
        self.stt.listen()


        while True:
            if self.stt.body is not None:
                if self.stt.body["intent"] == "bring_desc_to_someone":
                    return 'continue_SM_BRINGIT'
                #if self.stt.body["intent"] == "find_object":
                #    return 'continue_SM_FINDITEM'
                if self.stt.body["intent"] == "find_people":
                    return 'continue_SM_FIND_PERSON'
                if self.stt.body["intent"] == "move_to":
                    return 'continue_SM_GO_TO'
                if self.stt.body["intent"] == "counting":
                    return 'continue_SM_HOWMANY'
                else:
                    speak("Please state new command")
                    self.stt.clear()
                    self.stt.listen()
# ------- sm_bringit ----------
# ------- sm_finditem ---------
# ------- sm_find_person ------
#-------- sm_go_to ------------
#-------- sm_howmany ----------

def main():
    rospy.init_node('rospy_GPSR_state_machine')
    #delcare the globiable
    ed = EnvironmentDescriptor("../config/fur_data.yaml")
    ed.read_yaml()
    print(ed.data_yaml)
    target_lost = False
    is_stop = False
    stop_rotate = False
    person_id = -1
    last_pose = None

    # connect to server
    host = "0.0.0.0"
    # an important port
    WIT_lolov5 = 10002
    Face_recog = 10006
    Person_tracker = 10000
    Object_Tracker = 10008
    
    # Flask nlp server
    stt = SpeechToText("nlp")
    
    t = threading.Thread(target = stt.run ,name="nlp")
    t.start()

    global rs
    rs = Realsense()
    rs.wait()

    sm_top = smach.StateMachine(outcomes = ['succeeded','aborted'])
    with sm_top:
        smach.StateMachine.add('STAND_BY', Stand_by(stt),
                               transitions = {'continue_SM_BRINGIT':'SM_BRINGIT',
                                              'continue_SM_GO_TO':'SM_GO_TO',
                                              'continue_SM_FIND_PERSON':'SM_FIND_PERSON',
                                              'continue_SM_FINDITEM':'SM_FINDITEM',
                                              'continue_SM_HOWMANY':'SM_HOWMANY'})
        
        # sub state machine
        sm_bringit = smach.StateMachine(outcomes = ['succeeded', 'aborted'])
        with sm_bringit:
            smach.StateMachine.add('sm_bringit_CHECK_OBJECT_ROOM_2',sm_bringit_Check_Object_Room_2(),
                                   transitions = {'continue_sm_bringit_Save_Location':'sm_bringit_SAVE_LOCATION'})
            smach.StateMachine.add('sm_bringit_SAVE_LOCATION',sm_bringit_Save_Location(),
                                   transitions = {'continue_sm_bringit_Find_Object':'sm_bringit_FIND_OBJECT'})
            smach.StateMachine.add('sm_bringit_FIND_OBJECT', sm_bringit_Find_Object(),
                                   transitions = {'continue_sm_bringit_Pick_Up':'sm_bringit_PICK_UP',
                                                  'continue_aborted':'aborted'})
            smach.StateMachine.add('sm_bringit_PICK_UP',sm_bringit_Pick_Up(),
                                   transitions = {'continue_sm_bringit_Back_To_Start':'sm_bringit_BACK_TO_START'})
            smach.StateMachine.add('sm_bringit_BACK_TO_START',sm_bringit_Back_To_Start(),
                                   transitions = {'continue_sm_bringit_Place_Object':'sm_bringit_PLACE_OBJECT'})
            smach.StateMachine.add('sm_bringit_PLACE_OBJECT',sm_bringit_Place_Object(),
                                   transitions = {'continue_succeeded':'succeeded'})
        smach.StateMachine.add('SM_BRINGIT', sm_bringit,
                               transitions = {'succeeded':'STAND_BY'})
        
        #sub state machine
        sm_finditem = smach.StateMachine(outcomes = ['succeeded', 'aborted'])
        with sm_finditem:
            smach.StateMachine.add('sm_finditem_CHECK_OBJECT_ROOM', sm_finditem_Check_Object_Room(),
                                   transitions = {'continue_sm_finditem_Navigate_To_Room':'sm_finditem_NAVIGATE_TO_ROOM'})
            smach.StateMachine.add('sm_finditem_NAVIGATE_TO_ROOM', sm_finditem_Navigate_To_Room(),
                                   transitions = {'continue_sm_finditem_Find_Object':'sm_finditem_FIND_OBJECT',
                                                  'continue_aborted':'aborted'})
            smach.StateMachine.add('sm_finditem_FIND_OBJECT', sm_finditem_Find_Object(host, Object_Tracker),
                                   transitions = {'continue_sm_finditem_Announce':'sm_finditem_ANNOUNCE'})
            smach.StateMachine.add('sm_finditem_ANNOUNCE', sm_finditem_Announce(),
                                   transitions = {'continue_succeeded':'succeeded'})
        smach.StateMachine.add('SM_FINDITEM', sm_finditem,
                               transitions = {'succeeded':'STAND_BY'})
        
        # sub state machien
        sm_find_person = smach.StateMachine(outcomes = ['succeeded', 'aborted'])
        with sm_find_person:
            smach.StateMachine.add('sm_find_person_SAVE_LOCATION', sm_find_person_Save_Location(),
                                   transitions = {'continue_sm_find_person_Ask_Which_Room':'sm_find_person_ASK_WHICH_ROOM'})
            smach.StateMachine.add('sm_find_person_ASK_WHICH_ROOM', sm_find_person_Ask_Which_Room(),
                                   transitions = {'continue_sm_find_person_Go_To_Room_1':'sm_find_person_GO_TO_ROOM_1',
                                                  'continue_sm_find_person_Go_To_Specified_Room':'sm_find_person_GO_TO_SPECIFIED_ROOM'})
            smach.StateMachine.add('sm_find_person_GO_TO_ROOM_1', sm_find_person_Go_To_Room_1(),
                                   transitions = {'continue_sm_find_person_Find_Item':'sm_find_person_FIND_ITEM'})
            smach.StateMachine.add('sm_find_person_GO_TO_SPECIFIED_ROOM', sm_find_person_Go_To_Specified_Room(),
                                   transitions = {'continue_sm_find_person_Find_Item':'sm_find_person_FIND_ITEM'})
            smach.StateMachine.add('sm_find_person_FIND_ITEM', sm_find_person_Find_Item(),
                                   transitions = {'continue_sm_find_person_Go_To_Room_2':'sm_find_person_GO_TO_ROOM_2',
                                                  'continue_aborted':'aborted',
                                                  'continue_sm_find_person_Back_To_Start':'sm_find_person_BACK_TO_START'})
            #smach.StateMachine.add('sm_find_person_FIND_ITEM_2', sm_find_person_Find_Item_2(),
            #                       transitions = {'continue_sm_find_person_Back_To_Start_2':'sm_find_person_BACK_TO_START_2'})
            smach.StateMachine.add('sm_find_person_GO_TO_ROOM_2', sm_find_person_Go_To_Room_2(),
                                   transitions = {'continue_sm_find_person_Find_Item':'sm_find_person_FIND_ITEM'})
            smach.StateMachine.add('sm_find_person_BACK_TO_START', sm_find_person_Back_To_Start(),
                                   transitions = {'continue_sm_find_person_Tell_Where_Is_Person':'sm_find_person_TELL_WHERE_IS_PERSON'})
            #smach.StateMachine.add('sm_find_person_BACK_TO_START_2', sm_find_person_Back_To_Start_2(),
            #                       transitions = {'continue_sm_find_person_Tell_Where_Is_Person_2':'sm_find_person_TELL_WHERE_IS_PERSON'})
            smach.StateMachine.add('sm_find_person_TELL_WHERE_IS_PERSON', sm_find_person_Tell_Where_Is_Person(),
                                   transitions = {'continue_succeeded':'succeeded'})
        smach.StateMachine.add('SM_FIND_PERSON', sm_find_person,
                               transitions = {'succeeded':'STAND_BY'})
        
        # sub state machine
        sm_go_to = smach.StateMachine(outcomes = ['succeeded', 'aborted'])
        sm_go_to.userdata.sm_go_to_location = ''
        #sm_go_to.userdata.sm_go_to_coordinate = []
        with sm_go_to:
            smach.StateMachine.add('sm_go_to_CHECK_OBJECT_LOCATION', sm_go_to_Check_Object_Location(stt),
                                   transitions = {'continue_sm_go_to_Navigation':'sm_go_to_NAVIGATION'},
                                   remapping = {'sm_go_to_Check_Object_Location_out':'sm_go_to_location'})
            smach.StateMachine.add('sm_go_to_NAVIGATION', sm_go_to_Navigation(ed),
                                   transitions = {'continue_sm_go_to_Announce':'sm_go_to_ANNOUNCE'},
                                   remapping = {'sm_go_to_Navigation_in':'sm_go_to_location'})
            smach.StateMachine.add('sm_go_to_ANNOUNCE', sm_go_to_Announce(),
                                   transitions = {'continue_aborted':'aborted',
                                                  'continue_succeeded':'succeeded'})
        smach.StateMachine.add('SM_GO_TO', sm_go_to,
                               transitions = {'succeeded':'STAND_BY'})
        sm_howmany = smach.StateMachine(outcomes = ['succeeded', 'aborted'])
        with sm_howmany:
            smach.StateMachine.add('sm_howmany_CHECK_LOCATION', sm_howmany_Check_Location(),
                                   transitions = {'continue_sm_howmany_Navigation':'sm_howmany_NAVIGATION'})
            smach.StateMachine.add('sm_howmany_NAVIGATION', sm_howmany_Navigation(),
                                   transitions = {'continue_sm_howmany_Object_Detection':'sm_howmany_OBJECT_DETECTION'})
            smach.StateMachine.add('sm_howmany_OBJECT_DETECTION', sm_howmany_Object_Detection(),
                                   transitions = {'continue_aborted':'aborted',
                                                  'continue_succeeded':'succeeded'})
        smach.StateMachine.add('SM_HOWMANY', sm_howmany,
                               transitions = {'succeeded':'STAND_BY'})

    # create and start the introspection server
    sis = smach_ros.IntrospectionServer('GPSR_server', sm_top, '/SM_GPSR')
    sis.start()
    # execute the state machine
    outcome = sm_top.execute()
    # wait for the state machine until stop
    rospy.spin()
    sis.stop()
if __name__ == '__main__':
    main()
