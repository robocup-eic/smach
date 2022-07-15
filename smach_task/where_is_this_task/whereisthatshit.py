"""
kill flask in background
kill -9 $(lsof -t -i:5000)
"""

# ros libraries
import roslib
import rospy
import smach
import smach_ros
import time
import tf
import tf2_ros
import tf2_msgs
from geometry_msgs.msg import PoseStamped, Twist ,Vector3, TransformStamped, Point, Pose
from std_msgs.msg import String

# import computer vision and realsense
import cv2
import numpy as np
from util.realsense import Realsense

# import for speech-to-text
from flask import Flask, request
import threading

# import for text-to-speech
import requests
import json
from util.nlp_server import SpeechToText, speak
import time

# Bring in the simple action client
import actionlib

#Bring in the .action file and messages used by the move based action
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from util.custom_socket import CustomSocket
from cv_bridge import CvBridge, CvBridgeError 
from sensor_msgs.msg import Image, CameraInfo 
from actionlib_msgs.msg import GoalStatus

# Utils function
from math import atan, pi, radians
from util.environment_descriptor import EnvironmentDescriptor
from util.realsense import Realsense

import math
from tf.transformations import euler_from_quaternion, quaternion_from_euler

def listen_for_intent(stt):
    while True:
        if stt.body is not None:
            if (stt.body["intent"] == "find_placement") and ("placement" in stt.body.keys()):
                print(stt.body["find_placement"])
                location = stt.body["find_placement"]
                stt.clear()
                start_time = time.time()
                break
            else:

                stt.clear()
                stt.listen()
                start_time = time.time()
        if time.time()-start_time > 8:

            stt.listen()
            start_time = time.time()

    return location

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
    
    def move_no_block(self, location):
        global ed
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now() - rospy.Duration.from_sec(1)
        goal.target_pose.pose = ed.get_robot_pose(location)
        self.move_base_client.send_goal(goal)


    def move_position(self, position):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now() - rospy.Duration.from_sec(1)

        rospy.loginfo('{}'.format(position))

        if position:
            goal.target_pose.pose.position = position.position
            goal.target_pose.pose.position.z = 0
            goal.target_pose.pose.orientation = position.orientation
            

            self.move_base_client.send_goal(goal)

    def move_position_no_block(self, position):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now() - rospy.Duration.from_sec(1)

        goal.target_pose.pose.position = position.position
        goal.target_pose.pose.position.z = 0
        goal.target_pose.pose.orientation = position.orientation

        self.move_base_client.send_goal(goal)


class Standby(smach.State):

    def __init(self):
        rospy.loginfo('Initiating Standby state')
        smach.State.__init__(self, outcomes = ['continue_DIRECTION'])
    
    def execute(self, userdata):
        global location, stt

        rospy.loginfo('Executing Standby state')

        navigation.move("instruction_point") #TODO should be information point
        speak('if you need any information, please say Hey walkie, wait for the beep, then ask question')

        start_time = time.time()
        WAIT_NLP = 8
        while True:
            if stt.body is not None:
                if (stt.body["intent"] == "find_placement") and ("placement" in stt.body.keys()):
                    print(stt.body["find_placement"])
                    location = stt.body["find_placement"]
                    stt.clear()
                    start_time = time.time()
                    break
                else:
                    speak("Pardon?")
                    stt.clear()
                    stt.listen()
                    start_time = time.time()
            if time.time()-start_time > WAIT_NLP:
                speak("Where do you want to go?")
                stt.listen()
                start_time = time.time()
        return 'continue_DIRECTION'

class Direction(smach.State):

    def __init(self):
        rospy.loginfo('Initiating Direction state')
        smach.State.__init__(self, outcomes = ['continue_FOLLOW'])
    
    def execute(self, userdata):

        global navigation, location, ed, stt, path
        rospy.loginfo('Executing Direction state')
        
        while not (location in [d['name'] for d in ed.data_yaml]):
            #location not in fur_data
            speak("I don't recognize that location. Where do you want to go again?")
            stt.clear()
            stt.listen()
            location = listen_for_intent(stt)

        speak("All right, i will show you the way, come")
        return 'continue_FOLLOW'
        
class Planning(smach.State):  
    def __init__(self):
        smach.State.__init__(self, outcomes=['continue_stop','continue_arrived'])
        rospy.loginfo('Initiating state Planning')
        self.move_base_client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    
    def execute(self, userdata):
        rospy.loginfo('Executing state Planning')
        global target_lost, is_stop, last_pose, robot_inside, location, path, is_arrived
        go_to_Navigation.move_no_block(location)
        while True:
            
            result = self.move_base_client.get_state()
            rospy.loginfo("status {}".format(result))

            if result == GoalStatus.SUCCEEDED :
                is_arrived = True
                return 'continue_arrived'

            if target_lost == True:
                return 'comtinue_stop'
            
class Check_location(smach.State):
    def __init__(self):
        rospy.loginfo('Initiating Check_location state')
        smach.State.__init__(self, outcomes=['continue_stop', 'continue_arrived'])
    
    def execute(self, userdata):
        rospy.loginfo('Executing Check_location state')
        global target_lost, is_arrived, is_stop
        
        def left_or_right (robot_pose= Pose(), furniture_name= ''):
            robot_position = robot_pose.position
            fur_position = ed.get_center_point(furniture_name)
            orientation_list = (robot_pose.orientation.x, robot_pose.orientation.y, robot_pose.orientation.z, robot_pose.orientation.w)
            yaw = math.degrees(euler_from_quaternion (orientation_list)[2])
            fx = fur_position.x
            fy = fur_position.y
            rx = robot_position.x
            ry = robot_position.y

            if -45 < yaw <= 45:
                # <-
                if (fx >= rx) and (math.atan(abs(fy-ry)/(fx-rx)) < 0.3925):
                    direction = 'front'
                else:
                    if ry < fy:
                        direction = 'left'
                    elif ry > fy:
                        direction = 'right'
                    else:
                        print('weird')

            elif -135 < yaw <= -45:
                #^|
                if (fy <= ry) and (math.atan(abs(fx-rx)/(ry-fy)) < 0.3925):
                    direction = 'front'
                else:
                    if rx < fx:
                        direction = 'left'
                    elif rx > fx:
                        direction = 'right'
                    else:
                        print('weird')
            elif 45 < yaw <= 135:
                # v|
                if (fy >= ry) and (math.atan(abs(fx-rx)/(fy-ry)) < 0.3925):
                    direction = 'front'
                else:
                    if rx > fx:
                        direction = 'left'
                    elif rx < fx:
                        direction = 'right'
                    else:
                        print('weird')
            else:
                if (fx <= rx) and (math.atan(abs(fy-ry)/(rx-fx)) < 0.3925):
                    direction = 'front'
                else:
                    if ry > fy:
                        direction = 'left'
                    elif ry < fy:
                        direction = 'right'
                    else:
                        print('weird')
            
            return direction
            
        furniture_list = ed.furlist("all")
        while True:
            # when we just enter the new room check if we are in the same location as the furniture
            # if no, speak("you are now in the ____ romm") speak("Please go to the next room") only once
            # if yes, speak("you are now in the ____ room") speak("the bed is next to you")
            robot_pos = self.tfBuffer.lookup_transform('map','base_footprint',rospy.Time.now()-rospy.Duration.from_sec(1.0)).pose
            closest_fur = ed.get_closest_fur(robot_pos,room=None)
            side = left_or_right(robot_pos,closest_fur)
            speak("there is {} on our {}side".format(closest_fur,side))
            furniture_list.remove(closest_fur)
            if target_lost == True or is_stop == True:
                return 'continue_stop'
            if is_arrived == True:
                return 'continue_arrived'


        
if __name__ == '__main__':
    # initiate ROS node
    rospy.init_node('where_is_this')
    # initiate the global variable
    target_lost = False
    is_stop = False
    robot_inside = True
    person_id = None
    stop_rotate = False
    last_pose = None
    is_arrived = False
    location = None
    path = None

    ed = EnvironmentDescriptor('/home/eic/ros/smach/smach_task/config/fur_data_onsite.yaml')
    ed.read_yaml()

    # person tracker
    host = "0.0.0.0"
    port_personTrack = 11000
    personTrack = CustomSocket(host,port_personTrack)
    personTrack.clientConnect()

    rs = Realsense()
    rs.wait() # wait for camera intrinsics

    #Navigation manager
    navigation = go_to_Navigation()
    
    #image publisher
    image_pub = rospy.Publisher("/blob/image_blob", Image, queue_size=1)

    # Flask nlp server
    stt = SpeechToText("nlp")
    t = threading.Thread(target = stt.run ,name="nlp")
    t.start()

    # start state machine
    sm = smach.StateMachine(outcomes=['Succeeded','Aborted'])

    with sm:
        # smach.StateMachine.add('Start_signal',Start_signal(),
        #                         transitions={'continue_Navigate_information_point':'Standby'})
        smach.StateMachine.add('Standby',Standby(),
                                transitions={'continue_DIRECTION':'Direction'}) 

        # Create sub smach state machine
        sm_follow = smach.Concurrence(outcomes=['STOP', 'ARRIVED'], default_outcome = 'ARRIVED', outcome_map = {'STOP':{'Stop_command':'continue_stop','Follow_person':'continue_stop','Get_bounding_box':'continue_stop','Check_location':'continue_stop'}})      
        with sm_follow:
            # smach.Concurrence.add('Stop_command',Stop_command())
            smach.Concurrence.add('Follow_person',Planning()) #navigate
            # smach.Concurrence.add('Check_location',Check_location()) #announce location
        smach.StateMachine.add('FOLLOW', sm_follow, transitions={'STOP':'Lost','ARRIVED':'Navigate_information_point'})
        #------------------------------------
        smach.StateMachine.add('Lost',Lost(),
                                transitions={'continue_FOLLOW':'FOLLOW','continue_Navigate_information_point':'Navigate_information_point'})
        # Set up                                                    
        sis = smach_ros.IntrospectionServer('Server_name',sm,'/PatterRoot')
        sis.start()
        
        outcome = sm.execute()

        rospy.spin()
        sis.stop()