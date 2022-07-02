#!/usr/bin/env python

import roslib
import rospy
import smach
import smach_ros

from client.custom_socket import CustomSocket

# ros pub sub
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import Bool
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo

# import for speed-to-text
from flask import Flask, request
import threading

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
from client.nlp_server import SpeechToText, speak
import time

# import yaml reader
from client.guest_name_manager import GuestNameManager

class Start_signal(smach.State):
    def __init__(self):
        rospy.loginfo('Initiating Start_signal state')
        smach.State.__init__(self,outcomes=['continue_Standby'])
    def execute(self,userdata):
        rospy.loginfo('Executing Start_signal state')
        # Detect door opening
        return 'continue_Standby'


class Standby(smach.State):
    def __init__(self):
        rospy.loginfo('Initiating Standby state')
        smach.State.__init__(self,outcomes=['continue_Ask'])

        # computer vision socker
        global c
        self.c = c

        # image
        self.frame = None
        self.depth_image = None
        self.x_pixel = None
        self.y_pixel = None
        self.intrinsics = None
        self.bridge = CvBridge()

    def execute(self,userdata):
        def check_image_size_for_cv(frame):
            if frame.shape[0] != 720 and frame.shape[1] != 1280:
                frame = cv2.resize(frame, (1280, 720))
            return frame

        def check_image_size_for_ros(frame):
            if frame.shape[0] != self.intrinsics.height and frame.shape[1] != self.intrinsics.width:
                frame = cv2.resize(frame, (self.intrinsics.width, self.intrinsics.height))
            return frame

        def rescale_pixel(x, y):
            x = int(x*self.intrinsics.width/1280)
            y = int(y*self.intrinsics.height/720)
            return (x, y)
        
        def find_closest_person():
            pass

        def detect():
            global target_lost, person_id, last_pose, person_id
            if self.frame is None:
                return
            # scale image incase image size donot match cv server
            self.frame = check_image_size_for_cv(self.frame)
            # send frame to server and recieve the result
            result = self.c.req(self.frame)
            # rescale pixel incase pixel donot match
            self.frame = check_image_size_for_ros(self.frame)

            lost = True

            # not found person yet
            if person_id == -1:
                center_pixel_list = []
                for track in result["result"]:
                    self.depth_image = check_image_size_for_ros(self.depth_image)
                    x_pixel = int((track[2][0]+track[2][2])/2)
                    y_pixel = int((track[2][1]+track[2][3])/2)
                    depth = self.depth_image[y_pixel, x_pixel] # numpy array
                    center_pixel_list.append((x_pixel, y_pixel, depth, track[0])) # (x, y, depth, perons_id)
                
                person_id = min(center_pixel_list, key=lambda x: x[2])[3] # get person id with min depth

            for track in result["result"]:
                # track : [id, class_name, [x1,y1,x2,y2]]
                # rospy.loginfo("Track ID: {} at {}".format(track[0],track[2]))
                if track[0] == person_id:
                    self.x_pixel = int((track[2][0]+track[2][2])/2)
                    self.y_pixel = int((track[2][1]+track[2][3])/2)

                    lost = False
                    self.lost_frame = 0
                    # rospy.loginfo("Found Target")
            
                    # visualize purpose
                    self.frame = cv2.circle(self.frame, (self.x_pixel, self.y_pixel), 5, (0, 255, 0), 2)
                    self.frame = cv2.rectangle(self.frame, rescale_pixel(track[2][0], track[2][1]), rescale_pixel(track[2][2], track[2][3]), (0, 255, 0), 2)
                    self.frame = cv2.putText(self.frame, str(person_id), (track[2][0], track[2][1] + 10), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
            
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(self.frame, "bgr8"))

            # check if person tracker can find any person
            if  lost==True:
                self.lost_frame += 1
                if last_pose is not None:
                    br = tf.TransformBroadcaster()
                    br.sendTransform(last_pose, tf.transformations.quaternion_from_euler(0, 0, 0),rospy.Time.now(),'human_frame', 'map')
            
            else:
                # check if it lost forever
                if self.lost_frame >= 100:
                    target_lost = True

                # 3d pose
                if not self.intrinsics:
                    rospy.logerr("no camera intrinsics")
                    return
                # rescale pixel incase pixel donot match
                self.depth_image = check_image_size_for_ros(self.depth_image)
                pix = (self.x_pixel, self.y_pixel)
                depth = self.depth_image[pix[1], pix[0]] # [y, x] for numpy array
                result = rs2.rs2_deproject_pixel_to_point(self.intrinsics, [pix[0], pix[1]], depth)  # [x, y] for realsense lib
                
                # data from comuter vision realsense x is to the right, y is to the bottom, z is toward.
                x_coord, y_coord, z_coord = result[0]/1000, result[1]/1000, result[2]/1000

                if 1 < z_coord < 8:
                    # rospy.loginfo("Target is at: ({}, {})".format(self.x_pixel, self.y_pixel))
                    # rospy.loginfo("Depth is {}".format(depth))
                    # rospy.loginfo("Detected at (x,y,z): {}".format([r/1000 for r in result]))
                    rospy.sleep(0.1)
                    
                    # camera frame for tf x is point toward and y is point left.
                    x = z_coord - 1 # set the goal point to be 1 meter away from person
                    y = -x_coord
                    z = -y_coord
                    br = tf.TransformBroadcaster()
                    br.sendTransform((x, y, z),tf.transformations.quaternion_from_euler(0, 0, 0),rospy.Time.now(),'human_frame','realsense_mount_1')
                    return True
            return False
        
        rospy.loginfo('Executing Standby state')
        # run person detection constantly
        # wait untill the robot finds a person then continue to the next state
        # before continue to the next state count the number of person
        global person_count
        person_count += 1
        rospy.loginfo("Guest count : ", person_count)

        # start person tracker
        image_sub = rospy.Subscriber("/camera/color/image_raw", Image , self.yolo_callback)
        depth_sub = rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image , self.depth_callback)
        depth_info_sub = rospy.Subscriber("/camera/aligned_depth_to_color/camera_info", CameraInfo , self.info_callback)
        self.image_pub = rospy.Publisher("/blob/image_blob", Image, queue_size=1)
        
        while True:
            if detect() == True:
                break
            return 'continue_Ask'

    def yolo_callback(self, data):
        try:
            # change subscribed data to numpy.array and save it as "frame"
            self.frame = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        except CvBridgeError as e:
            print(e)

    def depth_callback(self, frame):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(frame, frame.encoding)
        except CvBridgeError as e:
            print(e)
            return
        except ValueError as e:
            return

    def info_callback(self, cameraInfo):
        try:
            if self.intrinsics:
                return
            self.intrinsics = rs2.intrinsics()
            self.intrinsics.width = cameraInfo.width
            self.intrinsics.height = cameraInfo.height
            self.intrinsics.ppx = cameraInfo.K[2]
            self.intrinsics.ppy = cameraInfo.K[5]
            self.intrinsics.fx = cameraInfo.K[0]
            self.intrinsics.fy = cameraInfo.K[4]
            if cameraInfo.distortion_model == 'plumb_bob':
                self.intrinsics.model = rs2.distortion.brown_conrady
            elif cameraInfo.distortion_model == 'equidistant':
                self.intrinsics.model = rs2.distortion.kannala_brandt4
            self.intrinsics.coeffs = [i for i in cameraInfo.D]
        except CvBridgeError as e:
            print(e)
            return       


class Ask(smach.State):
    def __init__(self):
        rospy.loginfo('Initiating Ask state')
        smach.State.__init__(self,outcomes=['continue_Navigation'])
        
    def execute(self,userdata):
        rospy.loginfo('Executing Ask state')
        # ask the guest to register's his/her face to the robot
        # ask name and favorite drink
        # save name and favorite drink in dictionary
        global stt
        # register face
        speak("Please show your face to the robot's camera")
        # listening to the person and save his/her name to the file
        speak("What is your name?")
        while True:
            if stt.body is not None:
                print(stt.body)
                if stt.body["intent"] == "name":
                    # add guest name to database accordingly to the person_count
                    stt.clear()
                    break
        # listening to the person and save his his/her fav_drink to the file
        speak("What is your favorite drink?")
        

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
        
    def execute(self,userdata):
        rospy.loginfo('Executing Navigate_to_start state')
        # navigate back to the door to wait for the next guest
        if person_count == 2:
            speak("I have finished my task")
            return 'continue_SUCCEEDED'
        else:
            # navigate back to the door
            return 'continue_Standby'


if __name__ == '__main__':
    rospy.init_node('receptionist_task')

    gm = GuestNameManager("../../config/receptionist_database.yaml")

    person_count = 0

    # connect to server
    host = socket.gethostname()
    # face recognition model
    port_faceRec = 10006
    faceRec = CustomSocket(host,port_faceRec)
    faceRec.clientConnect()
    # person tracker model
    port_personTrack = 11000
    personTrack = CustomSocket(host,port_personTrack)
    personTrack.clientConnect()


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
    
    

