#!/usr/bin/env python

"""
kill flask in background
kill $(lsof -t -i:5000)
"""

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

# realsense and computer vision
import requests
import cv2
import numpy as np
import pyrealsense2.pyrealsense2 as rs2
from geometry_msgs.msg import PoseStamped, Twist ,Vector3, TransformStamped
from std_msgs.msg import Bool,Int64
import socket

# SimpleActionClient
import actionlib

# Brings in the .action file and messages used by the move base action
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import Bool
from client.custom_socket import CustomSocket
import time
import threading
from client.nlp_server import SpeechToText

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo

class Standby(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['continue_follow','continue_pointing'])
        rospy.loginfo('Initiating state Standby')
        global stt
        self.stt = stt

    def execute(self, userdata):
        rospy.loginfo('Executing state Standby')
        # Wait for "follow me" or "what is that" command
        while True:
            if self.stt.body is not None:
                print(self.stt.body)
            
                if self.stt.body["intent"] == "follow_people":
                    self.stt.body["intent"] = None
                    return "continue_follow"
            
                if self.stt.body["intent"] == "what_is_that":
                    self.stt.body["intent"] = None
                    return "continue_pointing"

            time.sleep(0.01)

class Stop_command(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['continue_stop'])
        rospy.loginfo('Initiating state Stop_command')
        global stt
        self.stt = stt

    def execute(self, userdata):
        rospy.loginfo('Executing state Stop_command')
        global target_lost
        global is_stop
        # Wait for "stop" command or target lost
        while True:
            if self.stt.body["intent"] == "stop":
                is_stop = True
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
        global target_lost
        global is_stop
        pose = TransformStamped()
        
        while True:
            try:
                pose = self.tfBuffer.lookup_transform('base_footprint','human_frame',rospy.Time.now()-rospy.Duration.from_sec(1.0))

                goal = MoveBaseGoal()
                goal.target_pose.header.frame_id = "base_footprint"
                goal.target_pose.header.stamp = rospy.Time.now()-rospy.Duration.from_sec(1)
                goal.target_pose.pose.position.x = pose.transform.translation.x
                goal.target_pose.pose.position.y = pose.transform.translation.y
                goal.target_pose.pose.orientation = pose.transform.rotation

                rospy.loginfo("Sending new goal: Quarternion is {}, {}, {}, {}".format(pose.transform.rotation.w,pose.transform.rotation.x,pose.transform.rotation.y,pose.transform.rotation.z))

                self.client.send_goal(goal)

                if  is_stop == True:
                    wait = self.client.cancel_goal()

                    cancel = Twist()
                    stop_pub = rospy.Publisher("/walkie/cmd_vel",Twist,queue_size=1)

                    cancel.linear.x = 0.0
                    cancel.linear.y = 0.0
                    cancel.angular.z = 0.0
                    
                    stop_pub.publish(cancel)

                    return "continue_stop"

                elif target_lost == True:
                    # wait untill the robot reaches the lastest goal
                    wait = self.client.wait_for_result()
                    # If the robot reaches the lastest goal
                    if wait:
                        return "continue_stop"
                    else:
                        # TODO recovery behavior
                        pass
                else:
                    wait = self.client.wait_for_result(rospy.Duration.from_sec(1.0))

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
        self.last_pose = None
        self.lost_frame=0
    
        

    def detect(self):
        global target_lost

        # scale image incase image size donot match cv server
        self.frame = self.check_image_size_for_cv(self.frame)
        # send frame to server and recieve the result
        result = self.c.req(self.frame)
        self.frame = self.check_image_size_for_ros(self.frame)

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
                self.lost_frame=0
                rospy.loginfo("Found person: target id is {}".format(track[0]))

                # visualize purpose
                self.frame = cv2.circle(self.frame, (self.x_pixel, self.y_pixel), 5, (0, 255, 0), 2)
                self.frame = cv2.rectangle(self.frame, self.rescale_pixel(track[2][0], track[2][2]), self.rescale_pixel(track[2][1], track[2][3]), (0, 255, 0), 2)
                self.frame = image = cv2.putText(self.frame, person_id, (track[2][0], track[2][2] + 10), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
            elif track[0] == person_id:
                self.x_pixel = int((track[2][0]+track[2][2])/2)
                self.y_pixel = int((track[2][1]+track[2][3])/2)

                lost = False
                self.lost_frame=0
                # rospy.loginfo("Found Target")
        
                # visualize purpose
                self.frame = cv2.circle(self.frame, (self.x_pixel, self.y_pixel), 5, (0, 255, 0), 2)
                self.frame = cv2.rectangle(self.frame, self.rescale_pixel(track[2][0], track[2][2]), self.rescale_pixel(track[2][1], track[2][3]), (0, 255, 0), 2)
                self.frame = image = cv2.putText(self.frame, person_id, (track[2][0], track[2][2] + 10), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
        
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(self.frame, "bgr8"))

        # check if person tracker can find any person
        if  lost==True:
            self.lost_frame += 1
            br = tf.TransformBroadcaster()
            br.sendTransform(self.last_pose,tf.transformations.quaternion_from_euler(0, 0, 0),rospy.Time.now(),'human_frame','realsense_mount_1')
        
        else:
            # check if it lost forever
            if self.lost_frame >= 300:
                target_lost = True

            # 3d pose
            if not self.intrinsics:
                rospy.logerr("no camera intrinsics")
                return None

            pix = (self.x_pixel, self.y_pixel)
            depth = self.depth_image[pix[1], pix[0]] # [y, x] for numpy array
            result = rs2.rs2_deproject_pixel_to_point(self.intrinsics, [pix[0], pix[1]], depth)  # [x, y] for realsense lib
            
            # data from comuter vision realsense x is to the right, y is to the bottom, z is toward.
            x_coord, y_coord, z_coord = result[0]/1000, result[1]/1000, result[2]/1000

            if z_coord > 1:
                # rospy.loginfo("Target is at: ({}, {})".format(self.x_pixel, self.y_pixel))
                # rospy.loginfo("Depth is {}".format(depth))
                # rospy.loginfo("Detected at (x,y,z): {}".format([r/1000 for r in result]))
                rospy.sleep(0.1)

                # camera frame for tf x is point toward and y is point left.
                x = z_coord-1 # set the goal point to be 1 meter away from person
                y = -x_coord
                z = -y_coord
                self.last_pose = (x, y, z)
                br = tf.TransformBroadcaster()
                br.sendTransform((x, y, z),tf.transformations.quaternion_from_euler(0, 0, 0),rospy.Time.now(),'human_frame','realsense_mount_1')

    # function used in callback functions
    def check_image_size_for_cv(self, frame):
        if frame.shape[0] != 720 and frame.shape[1] != 1280:
            frame = cv2.resize(frame, (1280, 720))
        return frame

    def check_image_size_for_ros(self, frame):
        if frame.shape[0] != self.intrinsics.height and frame.shape[1] != self.intrinsics.width:
            frame = cv2.resize(frame, (self.intrinsics.width, self.intrinsics.height))
        return frame

    def rescale_pixel(self, x, y):
        x = int(x*self.intrinsics.width/1280)
        y = int(y*self.intrinsics.height/720)
        return (x, y)
    
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
        """
                            +Z           
        -y   realsense frame|                
        | +z                |    
        |/                  |      
        o---> +x            |  +X    
                            | / 
        +Y -----------------o camera_link frame tf/

        """
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(frame, frame.encoding)
            # rescale pixel incase pixel donot match
            self.depth_image = self.check_image_size_for_ros(self.depth_image)

        except CvBridgeError as e:
            print(e)
            return
        except ValueError as e:
            return

    def depth_callback(self, frame):

        if (self.x_pixel is None) or (self.y_pixel is None):
            return
        try:
            cv_image = self.bridge.imgmsg_to_cv2(frame, frame.encoding)
            cv_image = cv2.resize(cv_image,(1280, 720))
            # pick one pixel among all the pixels with the closest range:
            pix = (self.x_pixel, self.y_pixel)
            if self.intrinsics:
                depth = cv_image[pix[1], pix[0]]
                
                result = rs2.rs2_deproject_pixel_to_point(self.intrinsics, [pix[0], pix[1]], depth)
                
                # data from comuter vision realsense x is to the right, y is to the bottom, z is toward.
                x_coord, y_coord, z_coord = result[0]/1000, result[1]/1000, result[2]/1000

                # camera frame for tf x is point toward and y is point left.
                x = z_coord-1 # set the goal point to be 1 meter away from person
                y = -x_coord
                z = -y_coord
                if x > 0:
                    # rospy.loginfo("Target is at: ({}, {})".format(self.x_pixel, self.y_pixel))
                    # rospy.loginfo("Depth is {}".format(depth))
                    # rospy.loginfo("Detected at (x,y,z): {}".format([r/1000 for r in result]))
                    rospy.sleep(0.1)

                    br = tf.TransformBroadcaster()
                    br.sendTransform((x, y, z),tf.transformations.quaternion_from_euler(0, 0, 0),rospy.Time.now(),'human_frame','realsense_mount_1')
                    self.z_distance = x

        except CvBridgeError as e:
            print(e)
            return
        except ValueError as e:
            return
        pass

    def yolo_callback(self, data): 

        global target_lost
        global person_id
        
        # change subscribed data to numpy.array and save it as "frame"
        self.frame = self.bridge.imgmsg_to_cv2(data,'bgr8')
        self.frame = cv2.resize(self.frame, (1280, 720))
        
        # send frame to server and recieve the result      
        result = self.c.req(self.frame)

        lost = True
        
        for track in result["result"]:
            # track : [id, class_name, [x,y,w,h]]
            # rospy.loginfo("Track ID: {} at {}".format(track[0],track[2]))

            # first time found person
            if not self.found_target:
                person_id = track[0]
                self.x_pixel = int((track[2][0]+track[2][2])/2)
                self.y_pixel = int((track[2][1]+track[2][3])/2)

                lost = False
                self.found_target = True
                self.lost_frame=0
                rospy.loginfo("Found person: target id is {}".format(track[0]))

            elif track[0] == person_id:
                self.x_pixel = int((track[2][0]+track[2][2])/2)
                self.y_pixel = int((track[2][1]+track[2][3])/2)

                lost = False
                self.lost_frame=0
                # rospy.loginfo("Found Target")

        if  lost==True:
            self.lost_frame += 1
        if self.lost_frame >= 300:
            target_lost = True

    def execute(self, userdata):
        rospy.loginfo('Executing state Get_bounding_box')
        global target_lost
        global is_stop
        image_sub = rospy.Subscriber("/camera/color/image_raw", Image , self.yolo_callback)
        depth_sub = rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image , self.depth_callback)
        depth_info_sub = rospy.Subscriber("/camera/aligned_depth_to_color/camera_info", CameraInfo , self.info_callback)
        self.image_pub = rospy.Publisher("/blob/image_blob", Image, queue_size=1)
        
        while True:



            if  target_lost == True:
                
                image_sub.unregister()
                depth_info_sub.unregister()
                depth_sub.unregister()
                return 'continue_stop'
            
            elif is_stop == True:
                
                image_sub.unregister()
                depth_info_sub.unregister()
                depth_sub.unregister()
                return 'continue_stop'


class Pose(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['continue_text_to_speech'],
                                    input_keys=['pose_input'],
                                    output_keys=['pose_output'])
        rospy.loginfo('Initiate state Pose')
        
        global pose_c

        # connect to server
        self.c = pose_c
        
        self.bridge = CvBridge()

        # initiate listObject and countFrame
        self.listObject=[]
        self.countFrame=0
    
    def callback(self, data):
        # change subscribed data to numpy.array and save it as "frame"
        self.frame = self.bridge.imgmsg_to_cv2(data,'bgr8')
        self.frame = cv2.resize(self.frame,(1280,720))
        
        # send frame to server and recieve the result      
        result = self.c.req(self.frame)
        
        # add countFrame counter and append the object to the list
        self.countFrame += 1
        if len(result['pointing_at']) != 0:
            self.listObject.append(str(result['pointing_at'][0]))
        
        # Print list of detected objects
        print("listObject = ",self.listObject)
        
        # check number of frame
        print("counter frame = " + str(self.countFrame))
            
    def execute(self, userdata):
        rospy.loginfo('Executing state Pose')
        self.listObject=[]
        self.countFrame=0

        self.sub = rospy.Subscriber("/camera/color/image_raw", Image, self.callback)
        
        # wait to capture 5 frame
        while self.countFrame < 40:
            rospy.sleep(0.01)
        
        # stop subscribing /camera/color/image_raw
        self.sub.unregister()
        
        # if there is no object
        if len(self.listObject) == 0:
            userdata.pose_output = 'no_object'
            return 'continue_text_to_speech'
        # if there is an object, find most common object
        else:
            print('Most common object =', max(set(self.listObject), key=self.listObject.count))
            userdata.pose_output = max(set(self.listObject), key=self.listObject.count)
            return 'continue_text_to_speech'


class Text_to_speech(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['continue_succeeded'],
                                    input_keys=['tts_input'])
    def execute(self, userdata):
        rospy.loginfo('Executing state Text_to_speech')
        print("You are pointing at " + userdata.tts_input)
        if userdata.tts_input == 'no_object':
            # text to Speech
            d = {"text" : "You are not pointing at any object"}
            requests.post('http://localhost:5003/tts', json=d)
        else:
            # text to speech
            d = {"text" : "You are pointing at " + userdata.tts_input}
            requests.post('http://localhost:5003/tts', json=d)
        return 'continue_succeeded'
    

if __name__ == '__main__':
    rospy.init_node('what_is_that_smach')
    target_lost = False
    person_id = None
    is_stop = False
    stop_rotate = False

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
    t = threading.Thread(target = stt.run ,name="flask")
    t.start()

    # Start state machine
    sm = smach.StateMachine(outcomes=['Succeeded','Aborted'])
    with sm:
        smach.StateMachine.add('Standby',Standby(),
                        transitions={'continue_follow':'FOLLOW','continue_pointing':'OBJECT_DETECTION'})
        
        # Create sub smach state machine "FOLLOW"
        sm_follow = smach.Concurrence(outcomes=['Stop'],default_outcome = 'Stop')
                                        
        with sm_follow:
            smach.Concurrence.add('Stop_command',Stop_command())
            smach.Concurrence.add('Follow_person',Follow_person())
            smach.Concurrence.add('Get_bounding_box',Get_bounding_box())
        smach.StateMachine.add('FOLLOW', sm_follow, transitions={'Stop':'Standby'})
        
        # Create sub smach state machine "OBJECT_DETECTION"
        sm_object_detection = smach.StateMachine(outcomes=['Succeeded'])
        with sm_object_detection:
            smach.StateMachine.add('Pose',Pose(),transitions={'continue_text_to_speech':'Text_to_speech'},
                                                   remapping={'pose_input':'object_is',
                                                                'pose_output':'object_is'}) 
            smach.StateMachine.add('Text_to_speech',Text_to_speech(),transitions={'continue_succeeded':'Succeeded'},
                                                                        remapping={'tts_input':'object_is'})
        smach.StateMachine.add('OBJECT_DETECTION',sm_object_detection,transitions={'Succeeded':'Standby'})
        
    # Set up
    sis = smach_ros.IntrospectionServer('Server_name',sm,'/PatterRoot')
    sis.start()
    outcome = sm.execute()
    rospy.spin()
    sis.stop()