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
import tf2_geometry_msgs
from geometry_msgs.msg import Twist
from actionlib_msgs.msg import GoalStatus

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
from geometry_msgs.msg import PoseStamped, Twist ,Vector3, TransformStamped, Pose
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

class Start_signal(smach.State):
    def __init__(self):
        rospy.loginfo('Initiating Start_signal state')
        smach.State.__init__(self,outcomes=['continue_Standby'])
        self.FRAME_COUNT_LIMIT = 5
        self.close_distance = 1 # meter

        self.moving_pub = rospy.Publisher("/walkie2/cmd_vel", Twist, queue_size=10)
        
    def execute(self,userdata):
        rospy.loginfo('Executing Start_signal state')

        global rs, navigation
        # Detect door opening
        x_pixel, y_pixel = 1280/2, 720/2
        frame_count = 0

        self.moving_msg = Twist()
        self.moving_msg.linear.x = 0.2

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

                # move forward
                #Moving through entrance door
                start_time = time.time()
                while time.time() - start_time < 4:
                    rospy.loginfo("Moving Forward...")
                    self.moving_pub.publish(self.moving_msg)
                    rospy.sleep(0.1)

                rospy.loginfo("Stop Moving Forward")
                self.moving_msg.linear.x = 0
                self.moving_pub.publish(self.moving_msg)

                break

            if distance > self.close_distance:
                frame_count += 1
            else:
                frame_count = 0
        
        # navigate to standby position
       

   

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
        rospy.loginfo('Executing Standby state')

        standby = navigation.move('receptionist_standby')

        if standby:
            rospy.loginfo('Walky stand by, Ready for order')
        else:
            rospy.logerr('Navigation failed')

        global image_pub, personTrack, rs

        def go_to_Navigation(location):
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = "map"
            goal.target_pose.header.stamp = rospy.Time.now() - rospy.Duration.from_sec(1)
            goal.target_pose.pose = self.ed.get_robot_pose(location)
            self.move_base_client.send_goal(goal)
            self.move_base_client.wait_for_result()
            result = self.move_base_client.get_result()
            rospy.loginfo("result {}".format(result))
            if result.status == GoalStatus.SUCCEEDED :
                return True
            else:
                return False

        def detect(frame):
            # scale image incase image size donot match cv server
            frame = rs.check_image_size_for_cv(frame)
            # send frame to server and recieve the result
            result = personTrack.req(frame)
            # rescale pixel incase pixel donot match
            frame = rs.check_image_size_for_ros(frame)

            # if there is no person just skip
            if len(result["result"]) == 0:
                # rospy.loginfo("guest not found yet")
                image_pub.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))
                return

            # not found person yet
            center_pixel_list = []
            for track in result["result"]:
                self.x_pixel = int((track[2][0]+track[2][2])/2)
                self.y_pixel = int((track[2][1]+track[2][3])/2)
                depth = rs.get_coordinate(self.x_pixel, self.y_pixel, ref=(1280,720))[2] # numpy array
                center_pixel_list.append((self.x_pixel, self.y_pixel, depth, track[0])) # (x, y, depth, perons_id)
            
            self.x_pixel = min(center_pixel_list, key=lambda x: x[2])[0]
            self.y_pixel = min(center_pixel_list, key=lambda x: x[2])[1]

            # filter x, y pixel at the edge
            if not (300 < self.x_pixel < 900):
                rospy.loginfo("X_pixel: {}, Y_pixel: {}".format(self.x_pixel, self.y_pixel))
                rospy.loginfo("Human not in the middle")
                return False

            self.x_pixel, self.y_pixel = rs.rescale_pixel(self.x_pixel, self.y_pixel)
            # visualize purpose
            frame = cv2.circle(frame, (self.x_pixel, self.y_pixel), 5, (0, 255, 0), 2)
            frame = cv2.rectangle(frame, rs.rescale_pixel(track[2][0], track[2][1]), rs.rescale_pixel(track[2][2], track[2][3]), (0, 255, 0), 2)
            frame = cv2.putText(frame, str(self.person_id), rs.rescale_pixel(track[2][0], track[2][1] + 10), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)

            image_pub.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))
            # 3d pose

            rospy.loginfo("X_pixel: {}, Y_pixel: {}".format(self.x_pixel, self.y_pixel))
            # rescale pixel incase pixel donot match

            x_coord, y_coord, z_coord = rs.get_coordinate(self.x_pixel, self.y_pixel, ref=(frame.shape[1], frame.shape[0]))
            rospy.loginfo("Target person is at coordinate: {}".format((x_coord, y_coord, z_coord)))
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
        global person_count, faceRec, stt, rs, gm, personDescription, PERSON1_DES

        # capture person description
        if person_count == 1:
            frame = rs.get_image()
            PERSON1_DES = personDescription.req(frame) #string
    

        # listening to the person and save his/her name to the file
        speak("What is your name?")
        stt.listen()
        person_name = ""
        while True:
            if stt.body is not None:
                if (stt.body["intent"] == "my_name") and ("people" in stt.body.keys()):
                    print(stt.body["people"])
                    person_name = stt.body["people"]
                    gm.add_guest_name("guest_{}".format(person_count), person_name)
                    # add guest name to database accordingly to the person_count
                    stt.clear()
                    break
                else:
                    speak("Pardon?")
                    stt.clear()
                    stt.listen()

        # register face
        speak("Please show your face to the robot's camera")
        while True:

            # count down
            for number in ["three", "two", "one"]:
                speak(number)
                time.sleep(0.2)
            speak("capture!")

            # scale image incase image size donot match cv server
            frame = rs.check_image_size_for_cv(rs.get_image())
            msg = faceRec.register(frame, person_name)
            print(msg)
            if msg["isOk"] or msg["message"] == "name taken":
                break
            else:
                print("No face detect.")
        
        # listening to the person and save his his/her fav_drink to the file
        speak("What is your favorite drink?")
        stt.listen()
        object_name = ""
        while True:
            if stt.body is not None:
                # print(stt.body.keys())
                if stt.body["intent"] == "favorite" and ("object" in stt.body.keys()):
                    print(stt.body["object"])
                    object_name = stt.body["object"]
                    speak('Ok sir')
                    gm.add_guest_fav_drink("guest_{}".format(person_count), object_name)
                    stt.clear()
                    break
                else:
                    speak("Pardon?")
                    stt.clear()
                    stt.listen()

        navigation.move('livingroom')

        return 'continue_Navigation'

class Navigation(smach.State):
    def __init__(self):
        rospy.loginfo('Initiating Navigation state')
        smach.State.__init__(self,outcomes=['continue_No_seat','continue_Seat'], output_keys=['avaliable_seat_out'])
        global ed

        # init list of seat
        self.chair_list = ed.get_chair_poses()
        self.person_list = []
        self.bridge = CvBridge()
        self.tf_buffer =  tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)

    def execute(self,userdata):
        rospy.loginfo('Executing Navigation state')
        global tf_Buffer, ed

        self.person_list = []

        def detect(frame):
            # scale image incase image size donot match cv server
            frame = rs.check_image_size_for_cv(frame)
            # send frame to server and recieve the result
            result = personTrack.req(frame)
            # rescale pixel incase pixel donot match
            frame = rs.check_image_size_for_ros(frame)
            # init person_list
            person_list = []

            # if there is no person just skip
            if len(result["result"]) == 0:
                image_pub.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))
                return

            for person in result["result"]:
                person_id = person[0]
                self.x_pixel = int((person[2][0]+person[2][2])/2)
                self.y_pixel = int((person[2][1]+person[2][3])/2)
                self.x_pixel, self.y_pixel = rs.rescale_pixel(self.x_pixel, self.y_pixel)

                # visualize purpose
                frame = cv2.circle(frame, (self.x_pixel, self.y_pixel), 5, (0, 255, 0), 2)
                frame = cv2.rectangle(frame, rs.rescale_pixel(person[2][0], person[2][1]), rs.rescale_pixel(person[2][2], person[2][3]), (0, 255, 0), 2)
                frame = cv2.putText(frame, str(person_id), rs.rescale_pixel(person[2][0], person[2][1] + 10), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
                
                # get 3d person point
                person_pose = Pose()
                person_pose.position.x, person_pose.position.y, person_pose.position.z = rs.get_coordinate(self.x_pixel, self.y_pixel, ref=(frame.shape[1], frame.shape[0]))
                person_pose.orientation.x, person_pose.orientation.y, person_pose.orientation.z, person_pose.orientation.w = 0,0,0,1
                person_list.append((person_id, person_pose))

            image_pub.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))
            # data from comuter vision realsense x is to the right, y is to the bottom, z is toward.
            return person_list

        def transform_pose(input_pose, from_frame, to_frame):
            # **Assuming /tf2 topic is being broadcasted
            pose_stamped = PoseStamped()
            pose_stamped.pose = input_pose
            pose_stamped.header.frame_id = from_frame
            pose_stamped.header.stamp = rospy.Time.now()
            output_pose_stamped = None
            try:
                # ** It is important to wait for the listener to start listening. Hence the rospy.Duration(1)
                while not self.tf_buffer.can_transform:
                    rospy.loginfo("Cannot transform from {} to {}".format(from_frame, to_frame))
                output_pose_stamped = self.tf_buffer.transform(pose_stamped, to_frame, rospy.Duration(1))

                return output_pose_stamped


            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                raise

            
        def avaliable_seat_list():
            global ed
            avaliable_seat = list(self.chair_list[:])
            for person in self.person_list:
                person_id      = person[0]
                min_distance = 10000000
                person_pose = person[1]
                person_pose = transform_pose(person_pose, "realsense_pitch", "map")
                closest_chair = -1

                chairs_distance = []

                person_location = "Person {} @ {}\n".format(person_id, person_pose.pose.position)
                new_line = "==========================\n"


                rospy.loginfo(person_location+new_line)
                # compare with chair
                for i,chair in enumerate(ed.get_chair_poses()): ## TODO ed get chair list
                    
                    chair_pose = chair
                
                    distance = float(((chair_pose.position.x - person_pose.pose.position.x)**2 + (chair_pose.position.y - person_pose.pose.position.y)**2)**0.5)

                    rospy.loginfo("Distance from chair#{}: {}".format(i, distance))

                    chairs_distance.append((chair,distance))

                sorted_chair = sorted(chairs_distance, key=lambda x: x[1])
                bond_chair = sorted_chair[0][0]

                if bond_chair in avaliable_seat:  
                    rospy.loginfo("Remove bond chair")
                    avaliable_seat.remove(bond_chair)
            
            return avaliable_seat
        
        #===============================================start=============================================
        # start person tracker
        # (person_id,point)

        rospy.sleep(0.5)
        result_person_list = []
        avaliable_seat = []
        for i in range(5):
            result_person_list = detect(rs.get_image())
            if (result_person_list) and (i!=0):
                for result in result_person_list:
                    # if there are no
                    if not result[0] in [r[0] for r in self.person_list]:
                        self.person_list.append(result)

        avaliable_seat = ['Walkie']
        # avaliable_seat = avaliable_seat_list()
        # rospy.loginfo("avaliable seat are:" + str(avaliable_seat))
        
        if len(avaliable_seat) == 0:
            return 'continue_No_seat'
        else:
            userdata.avaliable_seat_out = avaliable_seat
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
        smach.State.__init__(self,outcomes=['continue_Introduce_guest'], input_keys=['avaliable_seat_in'])
        
    def execute(self,userdata):
        rospy.loginfo('Executing Seat state')
        # list of avaliable seat
        avaliable_seat = userdata.avaliable_seat_in
        # announce that there is available seat
        # point to the furniture
        speak("There is an available seat here")
        return 'continue_Introduce_guest'

class Introduce_guest(smach.State):
    def __init__(self):
        rospy.loginfo('Initiating Introduce_guest state')
        smach.State.__init__(self,outcomes=['continue_Introduce_host'])
        self.rotate_pub = rospy.Publisher("/walkie2/cmd_vel", Twist, queue_size=10)
    def execute(self,userdata):
        rospy.loginfo('Executing Introduce_guest state')
        global person_count, gm, rs, PERSON1_DES
        # find the host and face the robot to the host
        rotate_msg = Twist()
        rotate_msg.angular.z = 0.1

        is_found = False
        while not is_found:
            self.rotate_pub.publish(rotate_msg)
            detections = faceRec.detect(rs.get_image())
            for name,location in detections.items():
                rospy.loginfo('Detected: {}'.format(name))
                if name == gm.get_guest_name("host") and 400 < location[0] < 800:
                    cancel = Twist()
                    cancel.linear.x = 0
                    cancel.linear.y = 0
                    cancel.angular.z = 0
                    is_found = True

        self.rotate_pub.publish(cancel)

        # clearly identify the person being introduced and state their name and favorite drink
        if person_count == 1:
            speak("Hello {host_name}, the guest name is {guest_1}".format(host_name = gm.get_guest_name("host"), guest_1 = gm.get_guest_name("guest_1")))
            speak("His favorite drink is {fav_drink1}".format(fav_drink1 = gm.get_guest_fav_drink("guest_1")))
        if person_count == 2:
            speak("Hello {host_name}, the new guest is {guest_2}".format(host_name = gm.get_guest_name("host"), guest_2 = gm.get_guest_name("guest_2")))
            speak("His favorite drink is {fav_drink2}".format(fav_drink2 = gm.get_guest_fav_drink("guest_2")))
        return 'continue_Introduce_host'

    
class Introduce_host(smach.State):
    def __init__(self):
        rospy.loginfo('Initiating Introduce_host state')
        smach.State.__init__(self,outcomes=['continue_Navigate_to_Standby'])
        self.rotate_pub = rospy.Publisher("/walkie2/cmd_vel", Twist, queue_size=10)
        
    def execute(self,userdata):
        rospy.loginfo('Executing Introduce_host state')
        # find the guest and face the robot to the guest

        # clearly identify the person being introduced annd state their name and favorite drink

        global person_count, gm, rs , PERSON1_DES
        # find the host and face the robot to the host
        rotate_msg = Twist()
        rotate_msg.angular.z = -0.1

        is_found = False
        while not is_found:
            self.rotate_pub.publish(rotate_msg)
            detections = faceRec.detect(rs.get_image())
            ############
            # return
            ############
            for name,location in detections.items():
                if name == gm.get_guest_name("guest_{}".format(person_count)) and 400 < location[0] < 800:
                    cancel = Twist()
                    cancel.linear.x = 0
                    cancel.linear.y = 0
                    cancel.angular.z = 0
                    is_found = True

        self.rotate_pub.publish(cancel)

        if person_count == 1:
            # find the guest1 and face the robot to the guest1
            speak("Hello {guest_1}, the host's name is {host_name}".format(guest_1 = gm.get_guest_name("guest_1"), host_name = gm.get_guest_name("host")))
            speak("His favorite drink is {fav_drink_host}".format(fav_drink_host = gm.get_guest_fav_drink("host")))
        if person_count == 2:
            # find the guest_2 and face the robot the the guest_2
            speak("Hello {guest_2}, the host's name is {host_name}".format(guest_2 = gm.get_guest_name("guest_2"), host_name = gm.get_guest_name("host")))
            speak("His favorite drink is {fav_drink_host}".format(fav_drink_host= gm.get_guest_fav_drink("host")))
            speak("The guest next to you is {guest_1}".format(guest_1 = gm.get_guest_name("guest_1")))
            speak(PERSON1_DES)
            # reset the variable 
            PERSON1_DES = ""
        return 'continue_Navigate_to_Standby'


if __name__ == '__main__':
    rospy.init_node('receptionist_task')

    tf_Buffer = tf2_ros.Buffer()

    ed = EnvironmentDescriptor("../config/fur_data_onsite.yaml")
    ed.visual_robotpoint()
    gm = GuestNameManager("../config/receptionist_database.yaml")
    gm.reset()
    person_count = 0
    PERSON1_DES = ""

    image_pub = rospy.Publisher("/blob/image_blob", Image, queue_size=1)
    navigation = go_to_Navigation()

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
    # person description model
    port_personDescription = 10009
    personDescription = CustomSocket(host, port_personDescription)
    personDescription.clientConnect()

    rs = Realsense()
    rs.wait() # wait for camera intrinsics

    # Flask nlp server
    stt = SpeechToText("nlp")
    t = threading.Thread(target = stt.run ,name="nlp")
    t.start()

    # Create a SMACH state machine
    sm_top = smach.StateMachine(outcomes=['SUCCEEDED'])

    # smach userdata
    sm_top.userdata.avaliable_seat = []

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
                                            'continue_Seat':'Seat'},
                               remapping  ={'avaliable_seat_out':'avaliable_seat'})
        smach.StateMachine.add('No_seat', No_seat(),
                               transitions={'continue_Introduce_guest':'Introduce_guest'})
        smach.StateMachine.add('Seat', Seat(),
                               transitions={'continue_Introduce_guest':'Introduce_guest'},
                               remapping  ={'avaliable_seat_in':'avaliable_seat'})
        smach.StateMachine.add('Introduce_guest', Introduce_guest(),
                               transitions={'continue_Introduce_host':'Introduce_host'})
        smach.StateMachine.add('Introduce_host', Introduce_host(),
                               transitions={'continue_Navigate_to_Standby':'Standby'})

    sis = smach_ros.IntrospectionServer('Server_name', sm_top, '/Receptionist')
    sis.start()
    # Execute SMACH plan
    outcome = sm_top.execute()
    rospy.spin()
    sis.stop()