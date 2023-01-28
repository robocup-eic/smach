#!/usr/bin/env python

"""
$ source walkie2_ws/devel/setup.bash
$ roslaunch realsense2_camera rs_rgbd.launch align_depth:=true color_width:=1280 color_height:=720 color_fps:=30 depth_width:=1280 depth_height:=720 depth_fps:=30 filters:=pointcloud
kill flask in background
$ kill -9 $(lsof -t -i:5000)
"""
import numpy
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
import copy


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
        
    def nav2goal(self,pose,frame):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = frame
        goal.target_pose.header.stamp = rospy.Time.now() - rospy.Duration.from_sec(1)
        goal.target_pose.pose = pose
        self.move_base_client.send_goal(goal)
        self.move_base_client.wait_for_result()
        while True:
            result = self.move_base_client.get_state()
            rospy.loginfo("status {}".format(result))
            if result == GoalStatus.SUCCEEDED :
                return True
            else:
                return False
#---------------------------------------------------------------
class Walkie_Rotate(smach.State) :
    def __init__(self):
        rospy.loginfo('Initiating Walkie_Rotate state')
        smach.State.__init__(self,outcomes=['To_cus'],output_keys=['posesave'])
        self.rotate_pub = rospy.Publisher("/walkie2/cmd_vel", Twist, queue_size=10)
        self.bridge = CvBridge()
        self.save = ()
    
    def execute(self,userdata):
        rospy.loginfo('Executing Walkie_Rotate state')
        global image_pub, personDescription

        def detect(frame):
            o = False
            # scale image incase image size donot match cv server
            frame = rs.check_image_size_for_cv(frame)
            # send frame to server and recieve the result
            detections = HandRaising.req(frame)
            # rescale pixel incase pixel donot match
            frame = rs.check_image_size_for_ros(frame)

            center_pixel_list = []
            for id in detections.keys():

                print(detections[id])

                if detections[id]["hand_raised"] == True:
                    x, y, w, h, hand_raised = (detections[id][k] for k in ("x", "y", "w", "h", "hand_raised"))
                    # max_x, min_x, max_y, min_y, hand_raised = res[id]
                    color = (0, 255, 0) if hand_raised else (0, 0, 255)
                    frame = cv2.rectangle(frame, (x, y), (x + w, y + h), color, 3)
                    frame = cv2.circle(frame, (x+w/2, y+h/2), 5, color, 2)

                    if hand_raised == True:
                        center_pixel_list.append((x+w/2, y+h/2,id))
                        print(center_pixel_list)
                        o = True
                else:
                    # rospy.loginfo("guest not found yet")
                    image_pub.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))
                    return False
            
            if o == True:
                image_pub.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))

                minz = 999999
                for id in center_pixel_list:
                    x_coord, y_coord, z_coord = rs.get_coordinate(id[0],id[1], ref=(frame.shape[1], frame.shape[0]))
                    rospy.loginfo("Target person is at coordinate: {}".format((x_coord, y_coord, z_coord)))
                    if z_coord < minz:
                        minz = z_coord
                        self.save = (x_coord, y_coord, z_coord,id[2])
            
                return True
            
            return False
                    


            
        # find people raising hand
        rotate_msg = Twist()
        # rotate_msg.angular.z = 0.1
        rotate_msg.angular.z = 0.0

        # speak to start
        speak("I'm ready")
        time.sleep(0.5)
        speak("I'm looking for the waving customer")

        start_time = time.time()
        while True:
            print(detect(rs.get_image()))
            if detect(rs.get_image()) == True:
                userdata.posesave = self.save
                break



        #desc = personDescription.req(frame_ori)
        speak("I found the customer raising hand")
        # time.sleep(0.5)
        # speak(desc)
        # time.sleep(1.0)
        speak("I will come to you")
        


        return 'To_cus'
            
class Walkie_Speak(smach.State) :
    def __init__(self):
        rospy.loginfo('Initiating Walkie_Speak state')
        smach.State.__init__(self,outcomes=['continue_succeed'])
    
    def execute(self,userdata):
        rospy.loginfo('Executing Walkie_Speak state')


        # speak("I found the customer raising hand")
        # time.sleep(0.5)
        # speak("I will come to you")
        return 'continue_succeed'

class to_cutomer(smach.State):
    def __init__(self):
            rospy.loginfo('Initiating Walkie_Speak state')
            smach.State.__init__(self,outcomes=['speak'],input_keys=['posesave'])
            self.tf_buffer =  tf2_ros.Buffer()
            self.listener = tf2_ros.TransformListener(self.tf_buffer)

        
    def execute(self,userdata):
        rospy.loginfo('Executing Walkie_Speak state')
        posesave = userdata.posesave

        # tune coordinate
        recieved_pose = Pose()
        recieved_pose.position.x = posesave[0]
        recieved_pose.position.y = posesave[1]
        recieved_pose.position.z = posesave[2]
        recieved_pose.position.x -= 0.05
        recieved_pose.position.y += 0.03
        recieved_pose.position.z += 0.07

        rospy.loginfo('\n-----------------------')
        rospy.loginfo(recieved_pose)
        rospy.loginfo('-----------------------\n')
        

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
        
        transformed_pose = transform_pose(
            recieved_pose, "realsense_pitch", "base_footprint")
        rospy.loginfo(transformed_pose.position.x)
        # transformed_pose.position.x -= 1000
        transformed_pose.orientation.x = 0
        transformed_pose.orientation.y = 0
        transformed_pose.orientation.z = 0
        transformed_pose.orientation.w = 1

        navigation.nav2goal(transform_pose,"base_footprint")

        return 'speak'





#---------------------------------------------------------------

if __name__ == '__main__':
    rospy.init_node('restaurant_task')

    tf_Buffer = tf2_ros.Buffer()
    navigation = go_to_Navigation()
    ed = EnvironmentDescriptor("fucku.yaml")
    # ed.visual_robotpoint()

    # connect to server
    host = "0.0.0.0"
    # host = socket.gethostname()
    # hand raising detection
    port_HandRaising = 10011
    HandRaising = CustomSocket(host, port_HandRaising)
    HandRaising.clientConnect()

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
    sm_top.userdata.posesave = ()

    image_pub = rospy.Publisher("/blob/image_blob", Image, queue_size=1)

    # Open the container
    with sm_top:
        # Add states to the container
        # smach.StateMachine.add('Turn_Around_Walkie', Turn_Around_Walkie(),
        #                         transitions={'continue_Find_Hand_raising':'Find_Hand_raising'})
        smach.StateMachine.add('Turn_Around_Walkie', Walkie_Rotate(),
                                transitions={'To_cus':'To_cus'},
                                remapping={'posesave':'posesave'})




        smach.StateMachine.add('Walkie_Speak', Walkie_Speak(),
                                transitions={'continue_succeed':'SUCCEEDED'})

        smach.StateMachine.add('To_cus', to_cutomer(),
                                transitions={'speak':'Walkie_Speak'},
                                remapping={'posesave':'posesave'})
        

    sis = smach_ros.IntrospectionServer('Server_name', sm_top, '/Restaurant_task')
    sis.start()
    # Execute SMACH plan
    outcome = sm_top.execute()
    rospy.spin()
    sis.stop()