#!/usr/bin/env python
import numpy as np
from math import pi
from cv_bridge import CvBridge, CvBridgeError
import time
from flask import Flask, request
import threading
import requests
import cv2
import socket
from util.custom_socket import CustomSocket
from util.realsense import Realsense
from util.guest_name_manager import GuestNameManager
from util.environment_descriptor import EnvironmentDescriptor
from util.nlp_client import *

import rospy
import smach
import math
import smach_ros
import tf
import tf2_ros
import tf2_msgs
import tf2_geometry_msgs
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PoseStamped, Vector3, TransformStamped, Pose, Quaternion
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import Bool, Int16, Int64, Float32
from sensor_msgs.msg import Image, CameraInfo
from visualization_msgs.msg import Marker
from moveit_msgs.msg import DisplayTrajectory
import actionlib
import moveit_commander
import json
import copy
from tf.transformations import quaternion_from_euler,euler_from_quaternion

class go_to_Navigation():
    def __init__(self):
        self.move_base_client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        self.pubpose = rospy.Publisher('gosl',PoseStamped,queue_size=1)
    
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
        
    def nav2goal(self,pose):
        posl = PoseStamped()
        posl.header.frame_id = "map"
        posl.header.stamp = rospy.Time.now()


        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now() - rospy.Duration.from_sec(1)
        goal.target_pose.pose.position = pose.position
        goal.target_pose.pose.position.z = 0
        goal.target_pose.pose.orientation = pose.orientation

        posl.pose = goal.target_pose.pose
        self.pubpose.publish(posl)

        self.move_base_client.send_goal(goal)
        self.move_base_client.wait_for_result()
        while True:
            result = self.move_base_client.get_state()
            rospy.loginfo("status {}".format(result))
            if result == GoalStatus.SUCCEEDED :
                return True
            else:
                return False

#***********************************************************************************************

class Start_signal(smach.State):
    def __init__(self):
        rospy.loginfo('Initiating Start_signal state')
        smach.State.__init__(self,outcomes=['A'])


    def execute(self,userdata):
        rospy.loginfo('Executing Start_signal state')
        rospy.loginfo("*****************READY****************")
        
        speak("Initiate task: restaurant")
        
        return 'A'


class Get_pose(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['D'])
        rospy.loginfo('Initiating state Pose')
        self.bridge = rs.bridge

        # initiate list_object and countFrame
        self.object_list_all=[]
        self.countFrame=0

        self.pub_realsense_pitch_absolute_command = rospy.Publisher("/realsense_pitch_absolute_command", Int16, queue_size=1)
    
    def callback(self, data):
        # change subscribed data to numpy.array and save it as "frame"
        self.frame = self.bridge.imgmsg_to_cv2(data,'bgr8')
        self.frame = cv2.resize(self.frame,(1280,720))
        # send frame to server and recieve the result      
        result = what_is_that.req(self.frame)
        
        # add countFrame counter and append the object to the list
        self.countFrame += 1
        if len(result['what_is_that']) > 0:
            self.object_list_all.append(str(result['what_is_that'][0]))
        
        # Print list of detected objects
        # print("list_object = ",self.list_object)
        
        # check number of frame
        # print("counter frame = " + str(self.countFrame))
            
    def execute(self, userdata):
        rospy.loginfo('Executing state Pose')
        global object_list
        self.object_list_all = []
        self.countFrame = 0
        
        rospy.sleep(2)
        speak("Please show your hand to the camera and point at the object")
        rospy.logwarn("Please show your hand to the camera and point at the object")
        rospy.logfatal("Walkie is scaning skeleton 2 obj")

        self.sub = rospy.Subscriber("/camera/color/image_raw", Image, self.callback)
        
        # realsense 0
        self.pub_realsense_pitch_absolute_command.publish(0)
        time.sleep(1)
        # wait to capture 5 frame
        speak("I am looking")
        rospy.logwarn("looking")
        while self.countFrame < 50:
            rospy.sleep(0.01)
        
        # realsense -35
        # speak("looking down")
        # rospy.logwarn("looking down")
        # self.pub_realsense_pitch_absolute_command.publish(-20)
        # time.sleep(1)
        # # wait to capture 5 frame
        # while self.countFrame < 40:
        #     rospy.sleep(0.01)
            
        # stop subscribing /camera/color/image_raw
        self.sub.unregister()
        
        # if there is no object
        if len(self.object_list_all) == 0:
            object_list = []
            return 'D'
        # if there is an object, find most common object
        else:
            rospy.loginfo(self.object_list_all)
            obj = list(set(self.object_list_all))
            obj.sort(reverse=True, key=self.object_list_all.count)
            print("found object", obj)
            object_list = obj
            return 'D'

class Find_operator(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Q'])
        rospy.loginfo('Initiating state Find_operator')

        self.rotate_pub = rospy.Publisher("/walkie2/cmd_vel", Twist, queue_size=10)
        self.rotate_msg = Twist()
        # if count_group ==4:
        #     self.rotate_msg.angular.z = -0.2
        # else:
        self.rotate_msg.angular.z = 0.2

        self.cancel = Twist()
        self.cancel.linear.x = 0
        self.cancel.linear.y = 0
        self.cancel.angular.z = 0

        self.bridge = rs.bridge

        #Transforming Pose
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)

    def execute(self, userdata):
        rospy.loginfo('Executing state Find_operator')
        global personTrack, ed, count_group, rs

        count_group += 1

        def transform_pose(input_pose, from_frame, to_frame):
            # **Assuming /tf2 topic is being broadcasted
            pose_stamped = PoseStamped()
            pose_stamped.pose = input_pose
            pose_stamped.header.frame_id = from_frame
            # pose_stamped.header.stamp = rospy.Time.now()
            output_pose_stamped = None
            try:
                # ** It is important to wait for the listener to start listening. Hence the rospy.Duration(1)
                while not self.tf_buffer.can_transform:
                    rospy.loginfo("Cannot transform from {} to {}".format(from_frame, to_frame))
                output_pose_stamped = self.tf_buffer.transform(pose_stamped, to_frame, rospy.Duration(1))

                return output_pose_stamped.pose

            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                raise

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
                # filter w h small
                w = abs(track[2][0]-track[2][2])
                h = abs(track[2][1]-track[2][3])
                if (w*h < 200*100) or (self.x_pixel==1280) or (self.y_pixel == 720):
                    continue
                depth = rs.get_coordinate(self.x_pixel, self.y_pixel, ref=(1280,720))[2] # numpy array
                center_pixel_list.append((self.x_pixel, self.y_pixel, depth, track[0])) # (x, y, depth, perons_id)

            if len(center_pixel_list)==0:
                return False
            
            self.x_pixel = min(center_pixel_list, key=lambda x: x[2])[0]
            self.y_pixel = min(center_pixel_list, key=lambda x: x[2])[1]

            # filter x, y pixel at the edge
            if not (300 < self.x_pixel < 900):
                rospy.loginfo("Target not in the middle")
                return False

            self.x_pixel, self.y_pixel = rs.rescale_pixel(self.x_pixel, self.y_pixel)
            # visualize purpose
            frame = cv2.circle(frame, (self.x_pixel, self.y_pixel), 5, (0, 255, 0), 2)
            frame = cv2.rectangle(frame, rs.rescale_pixel(track[2][0], track[2][1]), rs.rescale_pixel(track[2][2], track[2][3]), (0, 255, 0), 2)
        
            image_pub.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))
            # 3d pose

            rospy.loginfo("X_pixel: {}, Y_pixel: {}".format(self.x_pixel, self.y_pixel))
            # rescale pixel incase pixel donot match

            x_coord, y_coord, z_coord = rs.get_coordinate(self.x_pixel, self.y_pixel, ref=(frame.shape[1], frame.shape[0]))
            rospy.loginfo("Target person is at coordinate: {}".format((x_coord, y_coord, z_coord)))
            # data from comuter vision realsense x is to the right, y is to the bottom, z is toward.
                        
            if 1.0 < z_coord :
                posi = Pose()
                posi.position.x, posi.position.y, posi.position.z = z_coord-0.5 , -x_coord, 0

                human_posi = transform_pose(posi, "realsense_pitch", "base_footprint")

                delta_x = human_posi.position.x
                delta_y = human_posi.position.y
                yaw = math.atan(delta_y/delta_x) # yaw

                human_posi.position.x -= 0.5

                posi.position.x, posi.position.y, posi.position.z = human_posi.position.x, human_posi.position.y, human_posi.position.z
                posi.orientation.x, posi.orientation.y, posi.orientation.z, posi.orientation.w = tf.transformations.quaternion_from_euler(0, 0, yaw)
                self.posi = transform_pose(posi, "base_footprint", "map")
                return True

            rospy.loginfo("Human out of range")
            return False
        # -------------------------------------------------------------------------------------
        
        # rotating until find person
        # reset()
        
        while True:
            self.rotate_pub.publish(self.rotate_msg)
            
            if detect(rs.get_image()) :
                speak("I found operator") 
                self.rotate_pub.publish(self.cancel)
                rospy.loginfo("go to operator at"+str(self.posi))
                # navigation.nav2goal(self.posi)
                break
            time.sleep(0.01)
        return 'Q'

class Text_to_speech(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['E','continue_get_pose','B'])
    def execute(self, userdata):
        rospy.loginfo('Executing state Text_to_speech')
        global object_list, count_group, point_time
        point_time += 1

        rospy.loginfo(object_list)
        if len(object_list) == 0:
            if point_time < 3:
                speak("I cannot see the object,I will looking again")
                rospy.logwarn("You are not pointing at any object")
                return 'continue_get_pose'
                
            else:
                speak("Let's move to the next group")
                rospy.logwarn("Let's move to the next group")
                point_time = 0
                return 'B'
        else:
            point_time = 0
            # is_correct = False
            # for i in range(len(object_list[:4])):
            #     if is_correct == True:
            #         break
            rospy.logfatal("Walkie get pointing obj")
            speak("You are pointing at " + object_list[0])
            rospy.logwarn("You are pointing at " + object_list[0])
            time.sleep(1)
                # speak("what should I do next?")
                # rospy.logwarn("what should I do next?")
                # stt.listen()
                # stt.clear()
            
            if count_group < 4:
                speak("Let's move to the next group")
                rospy.sleep(10)
                # is_correct = False
                return 'B'
            else:
                speak("What should I do next")
                rospy.logwarn("What should I do next")
                rospy.sleep(3)
                speak("ok")
                rospy.logwarn("ok sir")
                return 'E'

            time.sleep(5)
            speak("ok sir")
            rospy.logwarn("ok sir")
            rospy.logfatal("Walkie is going to pick an obj")
            navigation('stage5')
            return 'E'


if __name__ == '__main__':
    rospy.init_node('hand_me_that')

    object_list = []
    count_group = 0
    p = 3
    point_time = 0

    navigation = go_to_Navigation()

    # host
    host = "0.0.0.0"

    # obj detect
    # port_object = 10008
    # object_detection = CustomSocket(host=host, port=port_object)
    # object_detection.clientConnect()

    # what is that
    port_wtf = 10002
    what_is_that = CustomSocket(host, port_wtf)
    what_is_that.clientConnect()

    # person tracker model
    port_personTrack = 10020
    personTrack = CustomSocket(host,port_personTrack)
    personTrack.clientConnect()
    

    ed = EnvironmentDescriptor("/home/eic/ros/smach/smach_task/config/fucku.yaml")

    rospy.sleep(2)

    # publisher and subscriber
    lift_pub                     = rospy.Publisher('lift_command', Bool, queue_size=1)
    a                            = rospy.Publisher("posem",Marker,queue_size=1)
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',DisplayTrajectory, queue_size=1)
    gripper_publisher            = rospy.Publisher('/cr3_gripper_command',Bool,queue_size=1)

    rs = Realsense()
    rs.wait() # wait for camera intrinsics

    image_pub = rospy.Publisher("/blob/image_blob", Image, queue_size=1)

    
    ed = EnvironmentDescriptor("/home/eic/ros/smach/smach_task/config/fucku.yaml")

    # Start state machine
    sm = smach.StateMachine(outcomes=['E'])
    with sm:
        smach.StateMachine.add('O', Start_signal(),
                               transitions={'A': 'R'})
        smach.StateMachine.add('C', Get_pose(),
                               transitions={'D': 'D'})
        smach.StateMachine.add('D', Text_to_speech(),
                               transitions={'E': 'E','continue_get_pose':'C','B':'R'})
        smach.StateMachine.add('R', Find_operator(),
                               transitions={'Q': 'C'})

        
    # Set up
    sis = smach_ros.IntrospectionServer('Server_name',sm,'/Root')
    sis.start()
    outcome = sm.execute()
    rospy.spin()
    sis.stop()