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
from geometry_msgs.msg import PoseStamped, Twist ,Vector3, TransformStamped
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

class Start_signal(smach.State):
    def __init__(self):
        rospy.loginfo('Initiating Start_signal state')
        smach.State.__init__(self,outcomes=['continue_Standby'])
        self.FRAME_COUNT_LIMIT = 5
        self.close_distance = 1 # meter
        
    def execute(self,userdata):
        rospy.loginfo('Executing Start_signal state')
        global rs
        # Detect door opening
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
                break

            if distance > self.close_distance:
                frame_count += 1
            else:
                frame_count = 0
        return 'continue_Standby'

    
class Standby(smach.State):
    def __init__(self):
        rospy.loginfo('Initiating Standby state')
        smach.State.__init__(self,outcomes=['continue_Ask'])

        # image
        self.x_pixel = None
        self.y_pixel = None
        self.bridge = CvBridge()
        self.person_id = -1

    def execute(self,userdata):
        global image_pub, personTrack, rs

        def detect(frame):
            # scale image incase image size donot match cv server
            frame = rs.check_image_size_for_cv(frame)
            # send frame to server and recieve the result
            result = personTrack.req(frame)
            # rescale pixel incase pixel donot match
            frame = rs.check_image_size_for_ros(frame)

            # if there is no person just skip
            if len(result["result"]) == 0:
                image_pub.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))
                return

            # not found person yet
            if self.person_id == -1:
                center_pixel_list = []
                for track in result["result"]:
                    self.x_pixel = int((track[2][0]+track[2][2])/2)
                    self.y_pixel = int((track[2][1]+track[2][3])/2)
                    depth = rs.get_coordinate(self.x_pixel, self.y_pixel, ref=(1280,720))[2] # numpy array
                    center_pixel_list.append((self.x_pixel, self.y_pixel, depth, track[0])) # (x, y, depth, perons_id)
                
                self.person_id = min(center_pixel_list, key=lambda x: x[2])[3] # get person id with min depth
            
            for track in result["result"]:
                # track : [id, class_name, [x1,y1,x2,y2]]
                # rospy.loginfo("Track ID: {} at {}".format(track[0],track[2]))
                if track[0] == self.person_id:
                    self.x_pixel = int((track[2][0]+track[2][2])/2)
                    self.y_pixel = int((track[2][1]+track[2][3])/2)
                    self.x_pixel, self.y_pixel = rs.rescale_pixel(self.x_pixel, self.y_pixel)
                    # visualize purpose
                    frame = cv2.circle(frame, (self.x_pixel, self.y_pixel), 5, (0, 255, 0), 2)
                    frame = cv2.rectangle(frame, rs.rescale_pixel(track[2][0], track[2][1]), rs.rescale_pixel(track[2][2], track[2][3]), (0, 255, 0), 2)
                    frame = cv2.putText(frame, str(self.person_id), rs.rescale_pixel(track[2][0], track[2][1] + 10), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)

            image_pub.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))
            # 3d pose

            # rescale pixel incase pixel donot match
            x_coord, y_coord, z_coord = rs.get_coordinate(self.x_pixel, self.y_pixel, ref=(frame.shape[1], frame.shape[0]))
            # data from comuter vision realsense x is to the right, y is to the bottom, z is toward.

            if 0.75 < z_coord < 1.5:
                rospy.sleep(0.1)
                return True
            return False
        
        # -------------------------------------------------------------------------------------
        rospy.loginfo('Executing Standby state')
        # run person detection constantly
        # wait untill the robot finds a person then continue to the next state
        # before continue to the next state count the number of person
        global person_count

        # start person tracker
        rospy.sleep(0.5)
        while True:
            if detect(rs.get_image()) == True:
                self.x_pixel = None
                self.y_pixel = None
                self.person_id = -1
                person_count += 1
                break
        return 'continue_Ask'


class Ask(smach.State):
    def __init__(self):
        rospy.loginfo('Initiating Ask state')
        smach.State.__init__(self,outcomes=['continue_Navigation'])
        
    def execute(self,userdata):
            
        rospy.loginfo('Executing Ask state')
        # ask the guest to register's his/her face to the robot
        # ask name and favorite drink
        # save name and favorite drink in dictionary
        global person_count, faceRec, stt, rs, gm

        # listening to the person and save his/her name to the file
        speak("What is your name?")
        person_name = ""
        while True:
            if stt.body is not None:
                if stt.body["intent"] == "my_name" and "people" in stt.body.keys():
                    print(stt.body["people"])
                    person_name = stt.body["people"]
                    gm.add_guest_name("guest_{}".format(person_count), person_name)
                    # add guest name to database accordingly to the person_count
                    stt.clear()
                    break

        # register face
        speak("Please show your face to the robot's camera")
        while True:

            # count down
            for number in ["three", "two", "one"]:
                speak(number)
                time.sleep(1.0)
            speak("capture!")

            # scale image incase image size donot match cv server
            frame = rs.check_image_size_for_cv(rs.get_image)
            msg = faceRec.register(frame, person_name)

            if msg["isOk"]:
                print(msg)
                break
            else:
                print("No face detect.")
        
        # listening to the person and save his his/her fav_drink to the file
        speak("What is your favorite drink?")
        object_name = ""
        while True:
            if stt.body is not None:
                if stt.body["intent"] == "favorite" and "object" in stt.body.keys():
                    print(stt.body["object"])
                    object_name = stt.body["people"]
                    gm.add_guest_name("guest_{}".format(person_count), object_name)
                    stt.clear()
                    break
        return 'continue_Navigation'

class Navigation(smach.State):
    def __init__(self):
        rospy.loginfo('Initiating Navigation state')
        smach.State.__init__(self,outcomes=['continue_No_seat','continue_Seat'])
        self.case = 1
    def execute(self,userdata):
        rospy.loginfo('Executing Navigation state')
        # navigate to seat
        # detect available
        if self.case == 0:
            return 'continue_No_seat'
        else:
            return 'continue_Seat'


class No_seat(smach.State):
    def __init__(self):
        rospy.loginfo('Initiating No_seat state')
        smach.State.__init__(self,outcomes=['continue_Introduce_guest'])
    def execute(self,userdata):
        rospy.loginfo('Executing No_seat state')
        # announce that there is no seat available
        speak("Sorry, there is no available seat")
        return 'continue_Introduce_guest'


class Seat(smach.State):
    def __init__(self):
        rospy.loginfo('Initiating Seat state')
        smach.State.__init__(self,outcomes=['continue_Introduce_guest'])
    def execute(self,userdata):
        rospy.loginfo('Executing Seat state')
        # announce that there is available seat
        # point to the furniture
        speak("There is an available seat here")
        return 'continue_Introduce_guest'


class Introduce_guest(smach.State):
    def __init__(self):
        rospy.loginfo('Initiating Introduce_guest state')
        smach.State.__init__(self,outcomes=['continue_Introduce_host'])
        
    def execute(self,userdata):
        rospy.loginfo('Executing Introduce_guest state')
        global person_count
        global gm
        # find the host and face the robot to the host
        # clearly identify the person being introduced and state their name and favorite drink
        if person_count == 1:
            speak("Hello {host_name}, the guest who is on the {furniture} is {guest_1}".format(host_name = gm.get_guest_name("host"), furniture = "Couch", guest_1 = gm.get_guest_name("guest_1")))
            speak("His favorite drink is {fav_drink1}".format(fav_drink1 = gm.get_guest_fav_drink("guest_1")))
        if person_count == 2:
            speak("Hello {host_name}, the new guest is {guest_2}".format(host_name = gm.get_guest_name("host"), guest_2 = gm.get_guest_name("guest_2")))
            speak("His favorite drink is {fav_drink2}".format(fav_drink2 = gm.get_guest_fav_drink("guest_2")))
        return 'continue_Introduce_host'

    
class Introduce_host(smach.State):
    def __init__(self):
        rospy.loginfo('Initiating Introduce_host state')
        smach.State.__init__(self,outcomes=['continue_Navigate_to_start'])
        
    def execute(self,userdata):
        rospy.loginfo('Executing Introduce_host state')
        # find the guest and face the robot to the guest
        # clearly identify the person being introduced annd state their name and favorite drink
        global person_count
        if person_count == 1:
            # find the guest1 and face the robot to the guest1
            speak("Hello {guest_1}, the host's name is {host_name}".format(guest_1 = gm.get_guest_name("guest_1"), host_name = gm.get_guest_name("host")))
            speak("His favorite drink is {fav_drink_host}".format(fav_drink_host = gm.get_guest_fav_drink("host")))
        if person_count == 2:
            # find the guest_2 and face the robot the the guest_2
            speak("Hello {guest_2}, the host's name is {host_name}".format(guest_2 = gm.get_guest_name("guest_2"), host_name = gm.get_guest_name("host")))
            speak("His favorite drink is {fav_drink_host}".format(fav_drink_host= gm.get_guest_fav_drink("host")))
        return 'continue_Navigate_to_start'


class Navigate_to_start(smach.State):
    def __init__(self):
        rospy.loginfo('Initiating Navigate_to_start state')
        smach.State.__init__(self,outcomes=['continue_Standby', 'continue_SUCCEEDED'])
        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

    def execute(self,userdata):
        rospy.loginfo('Executing Navigate_to_start state')
        # navigate back to the door to wait for the next guest
        """
        global ed

        pose = ed.get_robot_pose("door_inside_living_room") # dont forget to find the right position on the setup day
            
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()-rospy.Duration.from_sec(1)
        goal.target_pose.pose.position.x = pose.position.x
        goal.target_pose.pose.position.y = pose.position.y
        goal.target_pose.pose.orientation = pose.orientation

        self.client.send_goal(goal)
        self.client.wait_for_result()
        result = self.client.get_result()
        
        if result.status == Goalstatus.SUCCEEDED:
            if person_count == 2:
                speak("I have finished my task")
                return 'continue_SUCCEEDED'
            else:
                # navigate back to the door
                return 'continue_Standby'

        if result.status == Goalstatus.ABORTED:
            rospy.loginfo("---------------------- ERROR ----------------")
            return 'continue_SUCCEEDED'
        """

        if person_count == 2:
            speak("I have finished my task")
            return 'continue_SUCCEEDED'
        else:
            # navigate back to the door
            return 'continue_Standby'


if __name__ == '__main__':
    rospy.init_node('receptionist_task')

    ed = EnvironmentDescriptor("../config/fur_data.yaml")
    gm = GuestNameManager("../config/receptionist_database.yaml")
    gm.reset()
    person_count = 0

    image_pub = rospy.Publisher("/blob/image_blob", Image, queue_size=1)

    # connect to server
    host = "0.0.0.0"
    # host = socket.gethostname()
    # face recognition model
    port_faceRec = 10006
    faceRec = CustomSocket(host,port_faceRec)
    faceRec.clientConnect()
    # person tracker model
    port_personTrack = 11000
    personTrack = CustomSocket(host,port_personTrack)
    personTrack.clientConnect()

    rs = Realsense()
    rs.wait() # wait for camera intrinsics

    # Flask nlp server
    stt = SpeechToText("nlp")
    t = threading.Thread(target = stt.run ,name="nlp")
    t.start()

    # Create a SMACH state machine
    sm_top = smach.StateMachine(outcomes=['SUCCEEDED'])

    # Open the container
    with sm_top:
        # Add states to the container
        smach.StateMachine.add('Start_signal', Start_signal(),
                               transitions={'continue_Standby':'Standby'})
        smach.StateMachine.add('Standby', Standby(),
                               transitions={'continue_Ask':'Ask'})
        smach.StateMachine.add('Ask', Ask(),
                               transitions={'continue_Navigation':'Navigation'})
        smach.StateMachine.add('Navigation', Navigation(),
                               transitions={'continue_No_seat':'No_seat',
                                            'continue_Seat':'Seat'})
        smach.StateMachine.add('No_seat', No_seat(),
                               transitions={'continue_Introduce_guest':'Introduce_guest'})
        smach.StateMachine.add('Seat', Seat(),
                               transitions={'continue_Introduce_guest':'Introduce_guest'})
        smach.StateMachine.add('Introduce_guest', Introduce_guest(),
                               transitions={'continue_Introduce_host':'Introduce_host'})
        smach.StateMachine.add('Introduce_host', Introduce_host(),
                               transitions={'continue_Navigate_to_start':'Navigate_to_start'})
        smach.StateMachine.add('Navigate_to_start', Navigate_to_start(),
                               transitions={'continue_Standby':'Standby',
                                            'continue_SUCCEEDED':'SUCCEEDED'})

    sis = smach_ros.IntrospectionServer('Server_name', sm_top, '/Receptionist')
    sis.start()
    # Execute SMACH plan
    outcome = sm_top.execute()
    rospy.spin()
    sis.stop()
    
    

