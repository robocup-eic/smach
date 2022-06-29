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
from actionlib_msgs import GoalStatus

# other utility
from util.custom_socket import CustomSocket
from util.nlp_server import SpeechToText, speak
from util.environment_descriptor import EnvironmentDescriptor
import time
import threading

class Stand_by(smach.State):
    def __init__(self):
        rospy.loginfo('initiating stand by state')
        smach.State.__init__(self, outcomes =['continue_SM_BRINGIT','continue_SM_FINDITEM','continue_SM_FIND_PERSON', 'continue_SM_GO_TO', 'continue_SM_HOWMANY'])
        self.x = 1
    def execute(self, userdata):
        if self.x == 1:
            return 'continue_SM_BRINGIT'
        elif self.x == 2:
            return 'continue_SM_FINDITEM'
        elif self.x == 3:
            return 'continue_SM_FIND_PERSON'
        elif self.x == 4:
            return 'continue_SM_GO_TO'
        else:
            return 'continue_SM_HOWMANY'


# ------- sm_bringit ----------------------------
class  sm_bringit_Check_Object_Room_2(smach.State):
    def __init__(self):
        rospy.loginfo('initiating check object room 2 state')
        smach.State.__init__(self, outcomes = ['continue_sm_bringit_Save_Location'])
    def execute(self, userdata):
        return 'continue_sm_bringit_Save_Location'


class sm_bringit_Save_Location(smach.State):
    def __init__(self):
        rospy.loginfo('initiating save location state')
        smach.State.__init__(self, outcomes = ['continue_sm_bringit_Find_Object'])
    def execute(self, userdata):
        return 'continue_sm_bringit_Find_Object'


class sm_bringit_Find_Object(smach.State):
    def __init__(self):
        rospy.loginfo('initiating find object state')
        smach.State.__init__(self, outcomes = ['continue_sm_bringit_Pick_Up','continue_aborted'])
        self.x = 1
    def execute(self, userdata):
        if self.x == 1:
            return 'continue_sm_bringit_Pick_Up'
        else:
            return 'continue_aborted'


class sm_bringit_Pick_Up(smach.State):
    def __init__(self):
        rospy.loginfo('initiating pick up state')
        smach.State.__init__(self, outcomes = ['continue_sm_bringit_Back_To_Start'])
    def execute(self, userdata):
        return 'continue_sm_bringit_Back_To_Start'


class sm_bringit_Back_To_Start(smach.State):
    def __init__(self):
        rospy.loginfo('initiating back to start state')
        smach.State.__init__(self, outcomes = ['continue_sm_bringit_Place_Object'])
    def execute(self, userdata):
        return 'continue_sm_bringit_Place_Object'


class sm_bringit_Place_Object(smach.State):
    def __init__(self):
        rospy.loginfo('initiating place object state')
        smach.State.__init__(self, outcomes = ['continue_succeeded'])
    def execute(self, userdata):
        return 'continue_succeeded'


# ---------------------------  sm_finditem ---------------------------------
class sm_finditem_Check_Object_Room(smach.State):
    def __init__(self):
        rospy.loginfo('initiating check object room state')
        smach.State.__init__(self, outcomes = ['continue_sm_finditem_Navigate_To_Room'])
        self.x = True
    def execute(self, userdata):
        return 'continue_sm_finditem_Navigate_To_Room'


class sm_finditem_Navigate_To_Room(smach.State):
    def __init__(self):
        rospy.loginfo('initiating navigate to room state')
        smach.State.__init__(self, outcomes = ['continue_sm_finditem_Find_Object', 'continue_aborted'])
        self.x = True
    def execute(self, userdata):
        if self.x == True:
            return 'continue_sm_finditem_Find_Object'
        else:
            return 'continue_aborted'


class sm_finditem_Find_Object(smach.State):
    def __init__(self):
        rospy.loginfo('initiating find object state')
        smach.State.__init__(self, outcomes = ['continue_sm_finditem_Announce'])
    def execute(self, userdata):
        return 'continue_sm_finditem_Announce'


class sm_finditem_Announce(smach.State):
    def __init__(self):
        rospy.loginfo('initiating announce state')
        smach.State.__init__(self, outcomes = ['continue_succeeded'])
    def execute(self, userdata):
        return 'continue_succeeded'

# ----------------------- sm find person --------------------------------
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
        if self.x == 1:
            return 'continue_sm_find_person_Go_To_Room_1'
        else:
            return 'continue_sm_find_person_Go_To_Specified_Room'


class sm_find_person_Go_To_Room_1(smach.State):
    def __init__(self):
        rospy.loginfo('initiating go to room state')
        smach.State.__init__(self, outcomes = ['continue_sm_find_person_Find_Item'])
    def execute(self, userdata):
        return 'continue_sm_find_person_Find_Item'


class sm_find_person_Find_Item_1(smach.State):
    def __init__(self):
        rospy.loginfo('initiating find item')
        smach.State.__init__(self, outcomes = ['continue_sm_find_person_Go_To_Room_2', 'continue_aborted', 'continue_sm_find_person_Back_To_Start_1'])
        self.x = 1
    def execute(self, userdata):
        if self.x == 1:
            return 'continue_sm_find_person_Go_To_Room_2'
        elif self.x == 2:
            return 'continue_aborted'
        else:
            return 'continue_sm_find_person_Back_To_Start_1'

class sm_find_person_Go_To_Room_2(smach.State):
    def __init__(self):
        rospy.loginfo('initiating go to rooom 2 state')
        smach.State.__init__(self, outcomes = ['continue_sm_find_person_Find_Item_1'])
    def execute(self, userdata):
        return 'continue_sm_find_person_Find_Item_1'

class sm_find_person_Back_To_Start_1(smach.State):
    def __init__(self):
        rospy.loginfo('initiating back to start state')
        smach.State.__init__(self, outcomes = ['continue_sm_find_person_Tell_Where_Is_Person_1'])
    def execute(self, userdata):
        return 'continue_sm_find_person_Tell_Where_Is_Person_1'

class sm_find_person_Go_To_Specified_Room(smach.State):
    def __init__(self):
        rospy.loginfo('initiating go to specified room')
        smach.State.__init__(self, outcomes = ['continue_sm_find_person_Find_Item_2'])
    def execute(self, userdata):
        return 'continue_sm_find_person_Find_Item_2'

class sm_find_person_Find_Item_2(smach.State):
    def __init__(self):
        rospy.loginfo('initiating find item2 state')
        smach.State.__init__(self, outcomes = ['continue_sm_find_person_Back_To_Start_2'])
    def execute(self, userdata):
        return 'continue_sm_find_person_Back_To_Start_2'

class sm_find_person_Back_To_Start_2(smach.State):
    def __init__(self):
        rospy.loginfo('initiating back to start 2 state')
        smach.State.__init__(self, outcomes = ['continue_sm_find_person_Tell_Where_Is_Person_2'])
    def execute(self, userdata):
        return 'continue_sm_find_person_Tell_Where_Is_Person_2'

class sm_find_person_Tell_Where_Is_Person(smach.State):
    def __init__(self):
        rospy.loginfo('initiating tell where is person state')
        smach.State.__init__(self, outcomes = ['continue_succeeded'])
    def execute(self, userdata):
        return 'continue_succeeded'

#--------------------------------  sm_go_to --------------------------------------
class sm_go_to_Check_Object_Location(smach.State):
    def __init__(self):
        rospy.loginfo('Initiating Check_object_location state')
        smach.State.__init__(self, outcomes = ['continue_sm_go_to_Navigation'],output_keys=['sm_go_to_Check_Object_Location_out'],)

    def execute(self, userdata):
        rospy.loginfo('Executing Check_object_location state')
        global stt
        while True:
            if stt.body is not None:
                # TODO check again with nlp
                location = stt.body["intent"]
                userdata.sm_go_to_Check_Object_Location_out = location
                return 'continue_sm_go_to_Navigation'

class sm_go_to_Navigation(smach.State):
    def __init__(self):
        rospy.loginfo('Initiating Navigation state')
        smach.State.__init__(self, outcomes = ['continue_sm_go_to_Announce'], input_keys=['sm_go_to_Navigation_in'])
        self.move_base_client = actionlib.SimpleActionClient('move_base',MoveBaseAction)

    def execute(self, userdata):
        rospy.loginfo('Executing Navigation state')
        global ed
        location = userdata.sm_go_to_Navigation_in

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now() - rospy.Duration.from_sec(1)
        goal.target_pose.pose = ed.get_robot_pose(location)
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


#------------------------- sm_howmany --------------------------------
class sm_howmany_Check_Location(smach.State):
    def __init__(self):
        rospy.loginfo('initiating check location state')
        smach.State.__init__(self, outcomes = ['continue_sm_howmany_Navigation'])
    def execute(self, userdata):
        return 'continue_sm_howmany_Navigation'

class sm_howmany_Navigation(smach.State):
    def __init__(self):
        rospy.loginfo('initiating navigation state')
        smach.State.__init__(self, outcomes = ['continue_sm_howmany_Object_Detection'])
    def execute(self, userdata):
        return 'continue_sm_howmany_Object_Detection'

class sm_howmany_Object_Detection(smach.State):
    def __init__(self):
        rospy.loginfo('initiating object detection state')
        smach.State.__init__(self, outcomes = ['continue_aborted', 'continue_succeeded'])
        self.x = 1
    def execute(self, userdata):
        if self.x == 1:
            return 'continue_aborted'
        else:
            return 'continue_succeeded'
def main():
    rospy.init_node('rospy_GPSR_state_machine')
    #delcare the global variable
    ed = EnvironmentDescriptor("../config/fur_data.yaml")
    target_lost = False
    is_stop = False
    stop_rotate = False
    person_id = -1
    last_pose = None

    # connect to server
    host = socket.gethostname()
    port = 11000
    c = CustomSocket(host,port)
    c.clientConnect()

    pose_port = 10002
    pose_c = CustomSocket(host,pose_port)
    pose_c.clientConnect()
    
    
    # Flask nlp server
    stt = SpeechToText("nlp")
    stt.clear()
    t = threading.Thread(target = stt.run ,name="flask")
    t.start()

    sm_top = smach.StateMachine(outcomes = ['succeeded','aborted'])
    with sm_top:
        smach.StateMachine.add('STAND_BY', Stand_by(),
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
            smach.StateMachine.add('sm_finditem_FIND_OBJECT', sm_finditem_Find_Object(),
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
                                   transitions = {'continue_sm_find_person_Find_Item':'sm_find_person_FIND_ITEM_1'})
            smach.StateMachine.add('sm_find_person_GO_TO_SPECIFIED_ROOM', sm_find_person_Go_To_Specified_Room(),
                                   transitions = {'continue_sm_find_person_Find_Item_2':'sm_find_person_FIND_ITEM_2'})
            smach.StateMachine.add('sm_find_person_FIND_ITEM_1', sm_find_person_Find_Item_1(),
                                   transitions = {'continue_sm_find_person_Go_To_Room_2':'sm_find_person_GO_TO_ROOM_2',
                                                  'continue_aborted':'aborted',
                                                  'continue_sm_find_person_Back_To_Start_1':'sm_find_person_BACK_TO_START_1'})
            smach.StateMachine.add('sm_find_person_FIND_ITEM_2', sm_find_person_Find_Item_2(),
                                   transitions = {'continue_sm_find_person_Back_To_Start_2':'sm_find_person_BACK_TO_START_2'})
            smach.StateMachine.add('sm_find_person_GO_TO_ROOM_2', sm_find_person_Go_To_Room_2(),
                                   transitions = {'continue_sm_find_person_Find_Item_1':'sm_find_person_FIND_ITEM_1'})
            smach.StateMachine.add('sm_find_person_BACK_TO_START_1', sm_find_person_Back_To_Start_1(),
                                   transitions = {'continue_sm_find_person_Tell_Where_Is_Person_1':'sm_find_person_TELL_WHERE_IS_PERSON'})
            smach.StateMachine.add('sm_find_person_BACK_TO_START_2', sm_find_person_Back_To_Start_2(),
                                   transitions = {'continue_sm_find_person_Tell_Where_Is_Person_2':'sm_find_person_TELL_WHERE_IS_PERSON'})
            smach.StateMachine.add('sm_find_person_TELL_WHERE_IS_PERSON', sm_find_person_Tell_Where_Is_Person(),
                                   transitions = {'continue_succeeded':'succeeded'})
        smach.StateMachine.add('SM_FIND_PERSON', sm_find_person,
                               transitions = {'succeeded':'STAND_BY'})
        
        # sub state machine
        sm_go_to = smach.StateMachine(outcomes = ['succeeded', 'aborted'])
        sm_go_to.userdata.sm_go_to_location = ''
        #sm_go_to.userdata.sm_go_to_coordinate = []
        with sm_go_to:
            smach.StateMachine.add('sm_go_to_CHECK_OBJECT_LOCATION', sm_go_to_Check_Object_Location(),
                                   transitions = {'continue_sm_go_to_Navigation':'sm_go_to_NAVIGATION'},
                                   remapping = {'sm_go_to_Check_Object_Location_out':'sm_go_to_location'})
            smach.StateMachine.add('sm_go_to_NAVIGATION', sm_go_to_Navigation(),
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
