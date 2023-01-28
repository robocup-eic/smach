#!/usr/bin/env python

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

# # import computer vision and realsense
# import cv2
# import numpy as np
# from util.realsense import Realsense

# # import for speech-to-text
# from flask import Flask, request
# import threading

# # import for text-to-speech
# import requests
# import json
# from util.nlp_server import SpeechToText, speak
# import time

#Bring in the simple action client
import actionlib

#Bring in the .action file and messages used by the move based action
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from util.custom_socket import CustomSocket
from cv_bridge import CvBridge, CvBridgeError #
from sensor_msgs.msg import Image, CameraInfo #
from actionlib_msgs.msg import GoalStatus

# Utils function
from math import atan, pi, radians
from util.environment_descriptor import EnvironmentDescriptor
# from util.realsense import Realsense

import math
from tf.transformations import euler_from_quaternion, quaternion_from_euler

# def detect(frame, rs, bridge, lower_bound = 0.75, upper_bound = 1.5):
#     global image_pub
#     # scale image incase image size donot match cv server
#     frame = rs.check_image_size_for_cv(frame)
#     # send frame to server and recieve the result
#     result = personTrack.req(frame)
#     # rescale pixel incase pixel donot match
#     frame = rs.check_image_size_for_ros(frame)

#     # if there is no person just skip
#     if len(result["result"]) == 0:
#         # rospy.loginfo("guest not found yet")
#         image_pub.publish(bridge.cv2_to_imgmsg(frame, "bgr8"))
#         return None

#     # not found person yet
#     center_pixel_list = []
#     for track in result["result"]:
#         x_pixel = int((track[2][0]+track[2][2])/2)
#         y_pixel = int((track[2][1]+track[2][3])/2)
#         depth = rs.get_coordinate(x_pixel, y_pixel, ref=(1280,720))[2] # numpy array
#         center_pixel_list.append((x_pixel, y_pixel, depth, track[0])) # (x, y, depth, perons_id)
    
#     x_pixel = min(center_pixel_list, key=lambda x: x[2])[0]
#     y_pixel = min(center_pixel_list, key=lambda x: x[2])[1]

#     # filter x, y pixel at the edge
#     if not (300 < x_pixel < 900):
#         rospy.loginfo("X_pixel: {}, Y_pixel: {}".format(x_pixel, y_pixel))
#         rospy.loginfo("Human not in the middle")
#         return False

#     x_pixel, y_pixel = rs.rescale_pixel(x_pixel, y_pixel)
#     # visualize purpose
#     frame = cv2.circle(frame, (x_pixel, y_pixel), 5, (0, 255, 0), 2)
#     frame = cv2.rectangle(frame, rs.rescale_pixel(track[2][0], track[2][1]), rs.rescale_pixel(track[2][2], track[2][3]), (0, 255, 0), 2)

#     image_pub.publish(bridge.cv2_to_imgmsg(frame, "bgr8"))
#     # 3d pose

#     rospy.loginfo("X_pixel: {}, Y_pixel: {}".format(x_pixel, y_pixel))
#     # rescale pixel incase pixel donot match

#     x_coord, y_coord, z_coord = rs.get_coordinate(x_pixel, y_pixel, ref=(frame.shape[1], frame.shape[0]))
#     rospy.loginfo("Target person is at coordinate: {}".format((x_coord, y_coord, z_coord)))
#     # data from comuter vision realsense x is to the right, y is to the bottom, z is toward.

#     if lower_bound < z_coord < upper_bound:
#         rospy.sleep(0.1)
#         return (x_coord, y_coord, z_coord)
#     return None

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

class Start_signal(smach.State):

    def __init__(self):
        rospy.loginfo('Initiating Start_signal state')
        smach.State.__init__(self,outcomes=['continue_Navigate_information_point'])
        self.FRAME_COUNT_LIMIT = 5
        self.close_distance = 1 # meter
    
    def execute(self,userdata):
        rospy.loginfo('Executing Start_signal state')
        return 'continue_Navigate_information_point'
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
                speak("Door open")
                break
            if distance > self.close_distance:
                frame_count += 1
            else:
                frame_count = 0
        
        return 'continue_Navigate_information_point'

class Standby(smach.State):

    def __init(self):
        rospy.loginfo('Initiating Standby state')
        smach.State.__init__(self, outcomes = ['continue_DIRECTION'])
    
    def execute(self, userdata):
        global navigation, rs, location, stt

        rospy.loginfo('Executing Standby state')
        return 'continue_DIRECTION'

        navigation.move("information_point") #TODO should be information point

       


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
        
        # run person detection to detect person coming in to the frame

        # start person tracker
        rospy.sleep(0.5)
        while True:
            if detect(rs.get_image()):
                self.x_pixel = None
                self.y_pixel = None
                self.person_id = -1
                break

        speak("Hello, where do you want to go?")
        stt.listen()

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
        #Wait to listen for command

        #Say hey walkie, wait for signal and receive command

        # wait for "Hey Walkie, where is ___" command
        # save the location name in String somewhere
        return 'continue_DIRECTION'

# Return location listened from the nlp
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

class Direction(smach.State):

    def __init(self):
        rospy.loginfo('Initiating Direction state')
        smach.State.__init__(self, outcomes = ['continue_FOLLOW'])
    
    def execute(self, userdata):

        global navigation, location, ed, stt, path
        rospy.loginfo('Executing Direction state')
        return 'continue_FOLLOW'
        while not (location in [d['name'] for d in ed.data_yaml]):
            #location not in fur_data
            speak("I don't recognize that location. Where do you want to go again?")
            stt.clear()
            stt.listen()
            location = listen_for_intent(stt)
        robot_pos = self.tfBuffer.lookup_transform('map','base_footprint',rospy.Time.now()-rospy.Duration.from_sec(1.0)).pose
        try:
            path = ed.get_directions(robot_pos,location)
        except IndexError:
            path = tuple(location)
        
        # if len(path) == 1:
        #     speak("come this way")
        # else:
        #     speak("The {} is located in the {}".format(location,path[-1]))
        #     rospy.sleep(0.1)
        #     speak()
        #     for i in path:
        #         speak("take the exit near{} to get to {}".format(path[i]))





class Planning(smach.State):  
    def __init__(self):
        smach.State.__init__(self, outcomes=['continue_stop','continue_arrived'])
        rospy.loginfo('Initiating state Planning')
        self.move_base_client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    
    def execute(self, userdata):
        rospy.loginfo('Executing state Planning')
        global target_lost, is_stop, last_pose, robot_inside, location, path, is_arrived
        count = 0
        robot_pos = self.tfBuffer.lookup_transform('map','base_footprint',rospy.Time.now()-rospy.Duration.from_sec(1.0)).pose
        path = ed.get_directions(robot_pos, 'sink')
        while True:
            go_to_Navigation.move_no_block(path[count])
            result = self.move_base_client.get_state()
            if result == GoalStatus.SUCCEEDED :
                count +=1
            if count == len(path)+1:
                is_arrived = True
                return 'continue_arrived'
            if is_stop == True:
                return 'continue_stop'

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

    ed = EnvironmentDescriptor('/home/arch/smach/smach_task/config/fur_data_onsite.yaml')
    ed.read_yaml()

    # person tracker
    host = "0.0.0.0"
    port_personTrack = 11000
    personTrack = CustomSocket(host,port_personTrack)
    personTrack.clientConnect()

    #Navigation manager
    navigation = go_to_Navigation()
    
    #image publisher
    image_pub = rospy.Publisher("/blob/image_blob", Image, queue_size=1)

    # start state machine
    sm = smach.StateMachine(outcomes=['Succeeded','Aborted'])

    with sm:
        smach.StateMachine.add('Start_signal',Start_signal(),
                                transitions={'continue_Navigate_information_point':'Standby'})
        smach.StateMachine.add('Standby',Standby(),
                                transitions={'continue_DIRECTION':'Direction'}) 
        smach.StateMachine.add('Direction',Direction(),
                        transitions={'continue_FOLLOW':'FOLLOW'})
        # Create sub smach state machine
        sm_follow = smach.Concurrence(outcomes=['STOP', 'ARRIVED'], default_outcome = 'ARRIVED', outcome_map = {'STOP':{'Stop_command':'continue_stop','Follow_person':'continue_stop','Get_bounding_box':'continue_stop','Check_location':'continue_stop'}})      
        with sm_follow:
            smach.Concurrence.add('Follow_person',Planning()) #navigate
        smach.StateMachine.add('FOLLOW', sm_follow, transitions={'STOP':'Lost','ARRIVED':'Navigate_information_point'})

        # Set up                                                    
        sis = smach_ros.IntrospectionServer('Server_name',sm,'/PatterRoot')
        sis.start()
        
        outcome = sm.execute()

        rospy.spin()
        sis.stop()
