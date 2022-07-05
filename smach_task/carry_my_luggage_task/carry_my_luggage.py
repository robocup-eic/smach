#!/usr/bin/env python

"""
kill flask in background
kill -9 $(lsof -t -i:5000)
"""

import roslib
import rospy
import smach
import smach_ros
import time
import tf
import tf2_ros
import tf2_msgs
from geometry_msgs.msg import PoseStamped, Twist ,Vector3, TransformStamped
import socket
import requests
import cv2
from math import pi
import pyrealsense2.pyrealsense2 as rs2

# import for speed-to-text
from flask import Flask, request
import threading

# import for text-to-speech
import requests
import json
from nlp_server import SpeechToText, speak
import time

#Bring in the simple action client
import actionlib

#Bring in the .action file and messages used by the move based action
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from custom_socket import CustomSocket
from cv_bridge import CvBridge, CvBridgeError #
from sensor_msgs.msg import Image, CameraInfo #


class Standby(smach.State):
    def __init__(self):
        rospy.loginfo('Initiating state Standby')
        smach.State.__init__(self, outcomes=['continue_follow'])
        
    def execute(self, userdata):
        global stt
        rospy.loginfo('Executing state Standby')
        start_time = 0
        while True:
            if stt.body is not None:
                print(stt.body)
                if stt.body["intent"] == "follow_people": # waiting for "follow me" command
                    stt.clear()

                    # Put bag hanger position code in here TODO

                    return "continue_follow"

            if time.time() - start_time > 10:
                speak("Please put your bag on my arm")
                start_time = time.time()

            time.sleep(0.01)


class Ask_if_arrived(smach.State):
    def __init__(self):
        rospy.loginfo('Initiating state Ask_if_arrived')
        smach.State.__init__(self, outcomes=['continue_standby','continue_place_luggage'])

    def execute(self, userdata):
        global stt
        rospy.loginfo('Executing state Ask_if_arrived')
        speak("Are we arrived?")
        
        while True:
            if stt.body["intent"] is not None:
                speak("Please say yes or no")
                rospy.loginfo(self.stt.body["intent"])

            if stt.body["intent"] == "Yes": # waiting for "follow me" command
                stt.body["intent"] = None
                return "continue_place_luggage"

            if stt.body["intent"] == "No": # waiting for "carry my luggage" command
                stt.body["intent"] = None
                speak("Please say follow me if you want me to follow me again")
                return "continue_standby"

            time.sleep(0.01)


class Place_luggage(smach.State):
    def __init__(self):
        rospy.loginfo('Initiating state Place_luggage')
        smach.State.__init__(self, outcomes=['continue_standby'])

    def execute(self, userdata):
        rospy.loginfo('Executing state Place_luggage')
        speak("Please pick your bag from my arm")
        time.sleep(10)
        return 'continue_standby'

class Check_position(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['continue_stop'])
        rospy.loginfo('Initiating state Check_position')

    def execute(self, userdata):
        rospy.loginfo('Executing state Check_position')
        global robot_inside, is_stop, target_lost
        while True:
            # if robot's is about to go outside set robot_inside = False
            
            # if robot's is about to go inside set robot_inside = True
            
            if is_stop :
                return 'continue_stop'

            if target_lost :
                return 'continue_stop'
            

class Stop_command(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['continue_stop'])
        rospy.loginfo('Initiating state Stop_command')

    def execute(self, userdata):
        rospy.loginfo('Executing state Stop_command')
        global target_lost, is_stop, stt
        # Wait for "stop" command or target lost
        while True:
            if stt.body is not None:
                if stt.body["intent"] == "stop":
                    is_stop = True
                    stt.clear()
                    return "continue_stop"

                if target_lost:
                    return "continue_stop"
            time.sleep(0.01)


class Follow_person(smach.State):  
    def __init__(self):
        smach.State.__init__(self, outcomes=['continue_stop'])
        rospy.loginfo('Initiating state Follow_person')
        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

    def execute(self, userdata):
        rospy.loginfo('Executing state Follow_person')
        global target_lost, is_stop, last_pose, robot_inside
        pose = TransformStamped()

        goal_send_interval = 5 # send goal at least every 5 seconds or wait until previous goal.
        start_time = 0
        
        while True:
            while robot_inside :
                try:
                    if time.time() - start_time > goal_send_interval:
                        pose = self.tfBuffer.lookup_transform('map','human_frame',rospy.Time.now()-rospy.Duration.from_sec(1.0))
                        goal = MoveBaseGoal()
                        goal.target_pose.header.frame_id = "map"
                        goal.target_pose.header.stamp = rospy.Time.now()-rospy.Duration.from_sec(1)
                        goal.target_pose.pose.position.x = pose.transform.translation.x
                        goal.target_pose.pose.position.y = pose.transform.translation.y
                        goal.target_pose.pose.orientation = pose.transform.rotation

                        self.client.send_goal(goal)
                        last_pose = (pose.transform.translation.x, pose.transform.translation.y, pose.transform.translation.z)
                        start_time = time.time()
                        # rospy.loginfo("Sending new goal: Quarternion is {}, {}, {}, {}".format(pose.transform.rotation.w,pose.transform.rotation.x,pose.transform.rotation.y,pose.transform.rotation.z))

                        if  is_stop == True:
                            self.client.cancel_goal()

                            cancel = Twist()
                            stop_pub = rospy.Publisher("/walkie/cmd_vel",Twist,queue_size=1)

                            cancel.linear.x = 0.0
                            cancel.linear.y = 0.0
                            cancel.angular.z = 0.0
                            
                            stop_pub.publish(cancel)

                            speak("I'm stopped")

                            return "continue_stop"

                        elif not robot_inside :
                            break

                        elif target_lost == True:

                            self.client.cancel_goal()

                            cancel = Twist()
                            stop_pub = rospy.Publisher("/walkie/cmd_vel",Twist,queue_size=1)

                            cancel.linear.x = 0.0
                            cancel.linear.y = 0.0
                            cancel.angular.z = 0.0
                            
                            stop_pub.publish(cancel)

                            speak("I lost you, where are you my friend.")

                            return "continue_stop"

                        else:
                            wait = self.client.wait_for_result(rospy.Duration.from_sec(1.0))
                    else:
                        rospy.sleep(0.1)

                except Exception as e:
                    pass

            while not robot_inside:
                    try:
                        if time.time() - start_time > goal_send_interval:
                            pose = self.tfBuffer.lookup_transform('map','human_frame',rospy.Time.now()-rospy.Duration.from_sec(1.0))
                            goal = MoveBaseGoal()
                            goal.target_pose.header.frame_id = "map"
                            goal.target_pose.header.stamp = rospy.Time.now()-rospy.Duration.from_sec(1)
                            goal.target_pose.pose.position.x = pose.transform.translation.x
                            goal.target_pose.pose.position.y = pose.transform.translation.y
                            goal.target_pose.pose.orientation = pose.transform.rotation

                            self.client.send_goal(goal)
                            last_pose = (pose.transform.translation.x, pose.transform.translation.y, pose.transform.translation.z)
                            start_time = time.time()
                            # rospy.loginfo("Sending new goal: Quarternion is {}, {}, {}, {}".format(pose.transform.rotation.w,pose.transform.rotation.x,pose.transform.rotation.y,pose.transform.rotation.z))

                            if  is_stop == True:
                                self.client.cancel_goal()

                                cancel = Twist()
                                stop_pub = rospy.Publisher("/walkie/cmd_vel",Twist,queue_size=1)

                                cancel.linear.x = 0.0
                                cancel.linear.y = 0.0
                                cancel.angular.z = 0.0
                                
                                stop_pub.publish(cancel)

                                speak("I'm stopped")

                                return "continue_stop"

                            elif robot_inside :
                                break

                            elif target_lost == True:

                                self.client.cancel_goal()

                                cancel = Twist()
                                stop_pub = rospy.Publisher("/walkie/cmd_vel",Twist,queue_size=1)

                                cancel.linear.x = 0.0
                                cancel.linear.y = 0.0
                                cancel.angular.z = 0.0
                                
                                stop_pub.publish(cancel)

                                speak("I lost you, where are you my friend.")

                                return "continue_stop"

                            else:
                                wait = self.client.wait_for_result(rospy.Duration.from_sec(1.0))
                        else:
                            rospy.sleep(0.1)

                    except Exception as e:
                        pass


class Get_bounding_box(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['continue_stop'])
        rospy.loginfo('Initiating state Get_bounding_box')

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

        # condition variable
        self.found_target = False
        self.lost_frame = 0
    
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

    def execute(self, userdata):
        # function used in callback functions
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

        def detect():
            global target_lost, person_id, last_pose
            if self.frame is None:
                return
            # scale image incase image size donot match cv server
            self.frame = check_image_size_for_cv(self.frame)
            # send frame to server and recieve the result
            result = self.c.req(self.frame)
            if len(result["result"]) == 0:
                self.image_pub.publish(self.bridge.cv2_to_imgmsg(self.frame, "bgr8"))
                return
            if not self.intrinsics:
                rospy.logerr("no camera intrinsics")
                return
            # rescale pixel incase pixel donot match
            self.frame = check_image_size_for_ros(self.frame)

            lost = True
            for track in result["result"]:
                # track : [id, class_name, [x1,y1,x2,y2]]
                # rospy.loginfo("Track ID: {} at {}".format(track[0],track[2]))

                # first time found person
                if not self.found_target:
                    person_id = track[0]
                    self.x_pixel = int((track[2][0]+track[2][2])/2)
                    self.y_pixel = int((track[2][1]+track[2][3])/2)
                    lost = False
                    self.found_target = True
                    self.lost_frame = 0
                    rospy.loginfo("Found person: target id is {}".format(track[0]))
                    # visualize purpose
                    self.frame = cv2.circle(self.frame, (self.x_pixel, self.y_pixel), 5, (0, 255, 0), 2)
                    self.frame = cv2.rectangle(self.frame, rescale_pixel(track[2][0], track[2][1]), rescale_pixel(track[2][2], track[2][3]), (0, 255, 0), 2)
                    self.frame = cv2.putText(self.frame, str(person_id), (track[2][0], track[2][1] + 10), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
                else:
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

        rospy.loginfo('Executing state Get_bounding_box')
        global is_stop, target_lost, robot_inside

        image_sub = rospy.Subscriber("/camera/color/image_raw", Image , self.yolo_callback)
        depth_sub = rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image , self.depth_callback)
        depth_info_sub = rospy.Subscriber("/camera/aligned_depth_to_color/camera_info", CameraInfo , self.info_callback)
        self.image_pub = rospy.Publisher("/blob/image_blob", Image, queue_size=1)
        rospy.sleep(0.5)

        while True:
            while robot_inside:
                detect()
                rospy.sleep(0.1)

                if  target_lost == True:
                    
                    image_sub.unregister()
                    depth_info_sub.unregister()
                    depth_sub.unregister()
                    return 'continue_stop'
                
                elif not robot_inside:
                    break

                elif is_stop == True:
                    image_sub.unregister()
                    depth_info_sub.unregister()
                    depth_sub.unregister()
                    return 'continue_stop'
            while not robot_inside:
                detect()
                rospy.sleep(0.1)

                if  target_lost == True:
                    
                    image_sub.unregister()
                    depth_info_sub.unregister()
                    depth_sub.unregister()
                    return 'continue_stop'
                
                elif robot_inside:
                    break

                elif is_stop == True:
                    
                    image_sub.unregister()
                    depth_info_sub.unregister()
                    depth_sub.unregister()
                    return 'continue_stop'


if __name__ == '__main__':
    # initiate ROS node
    rospy.init_node('carry_my_luggage')
    # initiate the global variable
    target_lost = False
    is_stop = False
    robot_inside = True
    person_id = None
    stop_rotate = False
    # Flask nlp server
    stt = SpeechToText("nlp")
    t = threading.Thread(target = stt.run ,name="nlp")
    t.start()

    # start state machine
    sm = smach.StateMachine(outcomes=['Succeeded','Aborted'])
    with sm:
        smach.StateMachine.add('Standby',Standby(),
                                transitions={'continue_follow':'FOLLOW'})
        
        
        smach.StateMachine.add('Ask_if_arrived',Ask_if_arrived(),
                                transitions={'continue_standby':'Standby',
                                             'continue_place_luggage':'Place_luggage'})

        smach.StateMachine.add('Place_luggage',Place_luggage(),
                                transitions={'continue_standby':'Standby'})
        
        # Create sub smach state machine
        sm_follow = smach.Concurrence(outcomes=['Stop'], default_outcome = 'Stop')      
        with sm_follow:
            smach.Concurrence.add('Stop_command',Stop_command())
            smach.Concurrence.add('Follow_person',Follow_person())
            smach.Concurrence.add('Get_bounding_box',Get_bounding_box())
            smach.Concurrence.add('Check_position',Check_position())
        smach.StateMachine.add('FOLLOW', sm_follow, transitions={'Stop':'Ask_if_arrived'})
    

        # Set up                                                    
        sis = smach_ros.IntrospectionServer('Server_name',sm,'/PatterRoot')
        sis.start()
        
        outcome = sm.execute()

        rospy.spin()
        sis.stop()