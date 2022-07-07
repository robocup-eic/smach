# !/usr/bin/env python

import roslib
import rospy
import smach
import smach_ros

# import msgs file
from geometry_msgs.msg import Pose

# import for speed-to-text
from flask import Flask, request
import threading

# import for text-to-speech
import requests
import json
from util.nlp_server import SpeechToText, speak
import time

# import yaml reader
from util.guest_name_manager import GuestNameManager
from util.environment_descriptor import EnvironmentDescriptor

# SimpleActionClient
import actionlib

# ros pub sub
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo

# navigation
import tf2_ros
from nav_msgs.msg import Odometry
from math import pi
import tf
import tf2_msgs
from geometry_msgs.msg import Twist
from actionlib_msgs.msg import GoalStatus

# realsense and computer vision
import requests
import numpy as np
import cv2
from geometry_msgs.msg import PoseStamped, Twist ,Vector3, TransformStamped, Pose
from std_msgs.msg import Bool,Int64
import socket
from util.custom_socket import CustomSocket
from util.realsense import Realsense


class Start_signal(smach.State):
    def __init__(self):
        rospy.loginfo('Initiating Start_signal state')
        smach.State.__init__(self,outcomes=['continue_Navigate_living_room'])
        self.FRAME_COUNT_LIMIT = 5
        self.close_distance = 1 # meter
        self.move_base_client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    
    def execute(self,userdata):
        rospy.loginfo('Executing Start_signal state')
        # wait for the door to open
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
                speak("door open")
                break
            if distance > self.close_distance:
                frame_count += 1
            else:
                frame_count = 0
        
        return 'continue_Navigate_living_room'


class Navigate_living_room(smach.State):
    def __init__(self):
        rospy.loginfo('Initiating Navigate_living_room state')
        smach.State.__init__(self, outcomes=['continue_Approach_person',
                                             'continue_Find_person'])
        # image
        self.x_pixel = None
        self.y_pixel = None
        self.bridge = CvBridge()
        self.person_id = -1

    def execute(self, userdata):
        global posi, image_pub
        # turn on person tracker model
        # navigate to the center of living room during using person tracker
        # if the model detects a person cancel the goal and find a Pose() of that person and send it to Approach_person state
        # if the robot reaches goal and does not locate a person go to Find_person state
        
        # turn on person tracker model
        def detect(frame):
            # scale image incase image size donot match cv server
            frame = rs.check_image_size_for_cv(frame)
            # send frame to server and recieve the result
            result = personTrack.req(frame)
            # rescale pixel incase pixel donot match
            frame = rs.check_image_size_for_ros(frame)

            # if there is no person just skip
            if len(result["result"]) == 0:
                rospy.loginfo("guest not found yet")
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
                    # found the closest person
                    break
                    
            image_pub.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))
            # 3d pose

            # rescale pixel incase pixel donot match
            x_coord, y_coord, z_coord = rs.get_coordinate(self.x_pixel, self.y_pixel, ref=(frame.shape[1], frame.shape[0]))
            # data from comuter vision realsense x is to the right, y is to the bottom, z is toward.
            # get 3d person point
            # person_pose = Pose()
            posi.position.x, posi.position.y, posi.position.z = z_coord - 0.5, -x_coord, -y_coord
            posi.orientation.x, posi.orientation.y, posi.orientation.z, posi.orientation.w = 0,0,0,1
            
            def transform_pose(input_pose, from_frame, to_frame):
                # **Assuming /tf2 topic is being broadcasted
                tf_buffer = tf2_ros.Buffer()
                pose_stamped = PoseStamped()
                pose_stamped.pose = input_pose
                pose_stamped.header.frame_id = from_frame
                pose_stamped.header.stamp = rospy.Time.now()
                try:
                    # ** It is important to wait for the listener to start listening. Hence the rospy.Duration(1)
                    output_pose_stamped = tf_buffer.transform(
                        pose_stamped, to_frame, rospy.Duration(1))
                    return output_pose_stamped.pose
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                    raise

            posi = transform_pose(posi, "realsense", "map")

            if 0.75 < z_coord < 8:
                rospy.sleep(0.1)
                return True
            return False
        
        rospy.loginfo('Executing Navigate_living_room state')
        # navigate to the center of living room during using person tracker
        def go_to_Navigation(location):
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
        
        # navigate to center of living room position and turn on person tracker
        standby = go_to_Navigation('living_room')
        while True:
            if detect(rs.get_image()) == True:
                # stop moving (cancel goal)
                self.move_base_client.cancel_goal()

                cancel = Twist()
                stop_pub = rospy.Publisher("/walkie/cmd_vel",Twist,queue_size=1)

                cancel.linear.x = 0.0
                cancel.linear.y = 0.0
                cancel.angular.z = 0.0

                stop_pub.publish(cancel)
                
                # reset parameter
                self.x_pixel = None
                self.y_pixel = None
                self.person_id = -1
                
                # get person Pose()
                ทำไงใครทำเป็นพิมที

                # send pose to the next state
                posi = person_pose
                
                return 'continue_Approach_person'

            if standby:
                rospy.loginfo('Arrived at center of living room')
                return 'continue_Find_person'
            else:
                rospy.logerr('Navigation failed')
                return 'continue_Find_person'

         # -------------------------------------------------------------------------------------



class Approach_person(smach.State):
    def __init__(self):
        rospy.loginfo('Initiating Approach_person state')
        smach.State.__init__(self,outcomes=['continue_Ask'])
    def execute(self, userdata):
        rospy.loginfo('Executing Approach_person state')
        global posi
        # receive the person pose from Navigate_living_room state or Find_person state
        # approach the person in order to ask the person
        # reset the pose() to zero before return 
        return 'continue_Ask'


class Find_person(smach.State):
    def __init__(self):
        rospy.loginfo('Initiating Find_person state')
        smach.State.__init__(self,outcomes=['continue_Approach_person'])
        # image
        self.x_pixel = None
        self.y_pixel = None
        self.bridge = CvBridge()
        self.person_id = -1

        # pub sub
        self.rotate_pub = rospy.Publisher("/walkie2/cmd_vel", Twist, queue_size=10)
        self.rotate_msg = Twist()
        self.rotate_msg.angular.z = 0.1

        self.stop_pub = rospy.Publisher("/walkie2/cmd_vel",Twist,queue_size=1)
        self.cancel = Twist()
        self.cancel.linear.x = 0
        self.cancel.linear.y = 0
        self.cancel.angular.z = 0

    def execute(self, userdata):
        rospy.loginfo('Executing Find_person state')
        global posi, image_pub
        # turn on person tracker model
        # rotate around during using person tracker
        # if the model detects a person cancel the goal and check the face in the database
        # if it does not match the database send pose  
        # send the pose to the approach person

        # turn on person tracker model
        def detect(frame):
            # scale image incase image size donot match cv server
            frame = rs.check_image_size_for_cv(frame)
            # send frame to server and recieve the result
            result = personTrack.req(frame)
            # rescale pixel incase pixel donot match
            frame = rs.check_image_size_for_ros(frame)

            # if there is no person just skip
            if len(result["result"]) == 0:
                rospy.loginfo("guest not found yet")
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

        self.rotate_pub.publish(self.rotate_msg)

        is_found = False
        while not is_found:
            time.sleep(0.5)
            detections = faceRec.detect(rs.get_image())
            if self.detect()
            for detection in detections:
                if detection["name"] != gm.get_guest_name("host") and 500 < detection["x"] < 700:
                    self.stop_pub(self.cancel)
                    is_found = True

        posi.orientation.x = 1
        posi.orientation.y = 2
        posi.orientation.z = 3
        posi.orientation.w = 4
        posi.position.x = 5
        posi.position.y = 6
        posi.position.z = 7

        return 'continue_Approach_person'


class Ask(smach.State):
    def __init__(self):
        rospy.loginfo('Initiating Ask state')
        smach.State.__init__(self,outcomes=['continue_Find_person',
                                            'continue_Navigate_to_start'])
    def execute(self, userdata):
        global count_person, gm
        rospy.loginfo('Executing Ask state')
        # ask name and save person's location
        speak("what is your name?")
        stt.listen()

        while True:
            if stt.body is not None:
                if stt.body["intent"] == "my_name" and "people" in stt.body.keys()
                    count_person += 1
                    person_name = stt.body["people"]
                    gm.add_guest_name("guest_{}".format(count_person), person_name) #(role, name)
                    stt.clear()
                    # save robot current location into yaml file !!!!!!!!!!! TODO
                    if count_person < 2:
                        return 'continue_Find_person'
                    else:
                        return 'continue_Navigate_to_start'
                else:
                    speak("Pardon?")
                    stt.clear()
                    stt.listen()

        
class Navigate_to_start(smach.State):
    def __init__(self):
        rospy.loginfo('Initiating Navigate_to_start state')
        smach.State.__init__(self,outcomes=['continue_Announce'])
    def execute(self, userdata):
        rospy.loginfo('Executing Navigate_to_start state')
        # walk back to the operator (location saved in yaml)
        return 'continue_Announce'


class Announce(smach.State):
    def __init__(self):
        rospy.loginfo('Initiating Announce state')
        smach.State.__init__(self,outcomes=['continue_SUCCEEDED'])
    def execute(self, userdata):
        rospy.loginfo('Executing Announce state')
        global gm
        # announce the person's name 
        # compare the person's location to the furiture's location
        # announce the person's location relative to the closest furniture 
        speak("Hello, I found two people in the room")
        speak("The first person is {} which is located next to the {}".format(gm.get_guest_name("guest_1"), "table")) # TODO change furniture 
        rospy.sleep(1)
        speak("The second person is {} which is located next to the {}".format(gm.get_guest_name("guest_2"), "chair")) # TODO change furniture
        rospy.sleep(2)
        speak("I have finished my task")
        return 'continue_SUCCEEDED'


if __name__ == '__main__':
    # initiate ros node
    rospy.init_node('find_my_friend_task')
    
    # initiate global variable
    count_person = 0
    posi = Pose()

    ed = EnvironmentDescriptor("../config/fur_data.yaml")
    gm = GuestNameManager("../config/find_my_mate_database.yaml")

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

    # start state machine
    sm_top = smach.StateMachine(outcomes=['SUCCEEDED'])
    # declare userdata
    # sm_top.userdata.sm_pose = Pose()
    with sm_top:
        smach.StateMachine.add('Start_signal', Start_signal(),
                               transitions={'continue_Navigate_living_room':'Navigate_living_room'})
        smach.StateMachine.add('Navigate_living_room', Navigate_living_room(),
                               transitions={'continue_Find_person':'Find_person',
                                            'continue_Approach_person':'Approach_person'})
        smach.StateMachine.add('Find_person', Find_person(),
                               transitions={'continue_Approach_person':'Approach_person'})
        smach.StateMachine.add('Approach_person', Approach_person(),
                               transitions={'continue_Ask':'Ask'})
        smach.StateMachine.add('Ask', Ask(),
                               transitions={'continue_Navigate_to_start':'Navigate_to_start',
                                            'continue_Find_person':'Find_person'})
        smach.StateMachine.add('Navigate_to_start', Navigate_to_start(),
                               transitions={'continue_Announce':'Announce'})
        smach.StateMachine.add('Announce', Announce(),
                               transitions={'continue_SUCCEEDED':'SUCCEEDED'})

        #sis start
        sis = smach_ros.IntrospectionServer('Server_name', sm_top, '/Findmyfriendroot')
        sis.start()
        outcomes = sm_top.execute()

        rospy.spin()
        sis.stop()