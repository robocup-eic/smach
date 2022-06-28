#!/usr/bin/env python

from turtle import st
import roslib
import rospy
import smach
import smach_ros
import time
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
from nlp_server import SpeechToText
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
        # self.follow_command = True
        # self.request_command = False
        global stt
        self.stt = stt
        
    def execute(self, userdata):
        rospy.loginfo('Executing state Standby')
        # if self.follow_command == True:    
        #     return 'continue_follow'
        # if self.request_command == True:
        #     return 'continue_request_luggage'
        start_time = 0
        while True:
            if self.stt.body is not None:
                print(self.stt.body)
                if self.stt.body["intent"] == "follow_people": # waiting for "follow me" command
                    self.stt.body["intent"] = None
                    return "continue_follow"

            if time.time() - start_time > 10:
                speak("Please put your bag on my arm")
                start_time = time.time()

            time.sleep(0.01)

def speak(text) :
    try :
        url = 'http://localhost:5003/tts'
        x = requests.post(url, json={'text':text})
        return x
    except :
        print("error to connect speak api.")

class Request_luggage(smach.State):
    def __init__(self):
        rospy.loginfo('Initiating state Request_luggage')
        smach.State.__init__(self, outcomes=['continue_standby'])
        # self.request = True

    def execute(self, userdata):
        rospy.loginfo('Executing state Request_luggage')
        # if self.request == True:
        #     return 'cotinue_standby'
        print('Please put your bag on my arm')
        d = {"text" : "Please put your bag on my arm"}
        x = request.post('http://localhost:5003/tts', json=d)
        time.sleep(10)
        return 'continue_standby'


class Ask_if_arrived(smach.State):
    def __init__(self):
        rospy.loginfo('Initiating state Ask_if_arrived')
        smach.State.__init__(self, outcomes=['continue_standby','continue_place_luggage'])
        # self.check = True
        global stt
        self.stt = stt

    def execute(self, userdata):
        rospy.loginfo('Executing state Ask_if_arrived')
        # if self.check == True:
        #     return 'cotinue_standby'
        # else:
        #     return 'continue_place_luggage'
        print('Are we arrived?')
        d = {"text" : "Are we arrived?"}
        x = request.post('http://localhost:5003/tts', json=d)
        
        while True:
            if self.stt.body["intent"] is not None:
                print(self.stt.body["intent"])

            if self.stt.body["intent"] == "Yes": # waiting for "follow me" command
                self.stt.body["intent"] = None
                return "continue_place_luggage"

            if self.stt.body["intent"] == "No": # waiting for "carry my luggage" command
                self.stt.body["intent"] = None
                return "continue_standby"

            time.sleep(0.01)

class Place_luggage(smach.State):
    def __init__(self):
        rospy.loginfo('Initiating state Place_luggage')
        smach.State.__init__(self, outcomes=['continue_standby'])
        # self.check = True

    def execute(self, userdata):
        rospy.loginfo('Executing state Place_luggage')
        # if self.check == True:
        #     return 'cotinue_standby'
        print('Please pick your bag from my arm')
        d = {"text" : "Please pick your bag from my arm"}
        x = request.post('http://localhost:5003/tts', json=d)
        time.sleep(10)
        return 'continue_standby'
        
# completed
class Stop_command(smach.State):
    def __init__(self):
        rospy.loginfo('Initiating state Stop_command')
        smach.State.__init__(self, outcomes=['continue_stop','continue_find_person'])
        #self.check = True
        global stt
        self.stt = stt
    def execute(self, userdata):
        rospy.loginfo('Executing state Stop_command')
        global target_lost
        global is_stop
        while True:
            if self.stt.body["intent"] == "stop":
                is_stop = True
                return 'continue_stop'
            if target_lost:
                return 'continue_find_person'
            time.sleep(0.01)


class Follow_person(smach.State):
    def __init__(self):
        rospy.loginfo('Initiating state Follow_person')
        smach.State.__init__(self, outcomes=['continue_stop','continue_find_person'])
        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.wait_time = 5.0
        
        rospy.sleep(5)
       
        self.client.wait_for_server()
        rospy.loginfo("Action server is up, we can send new goal!")
    
    
    def execute(self, userdata):
        rospy.loginfo('Executing state Follow_person')
        global target_lost
        global is_stop
        pose = TransformStamped()
        while True:
            try:
                pose = self.tfBuffer.lookup_transform('map','human_frame',rospy.Time.now()-rospy.Duration.from_sec(1.0))
                goal = MoveBaseGoal()
                goal.target_pose.header.frame_id = "map"
                goal.target_pose.header.stamp = rospy.Time.now()-rospy.Duration.from_sec(1)
                goal.target_pose.pose.position.x = pose.transform.translation.x
                goal.target_pose.pose.position.y = pose.transform.translation.y
                goal.target_pose.pose.orientation = pose.transform.rotation

                self.client.send_goal(goal) 
                if is_stop == True:
                    # wait untill the robot reaches the lastest goal
                    self.wait = self.client.cancel_goal()

                    cancel = Twist()
                    stop_pub = rospy.Publisher("cmd_vel",Twist,queue_size=1)

                    cancel.linear.x = 0.0
                    cancel.linear.y = 0.0
                    cancel.angular.z = 0.0
                    
                    stop_pub.publish(cancel)
                    rospy.loginfo("FUCK")

                    return "continue_stop"
                if target_lost == True:
                    # wait untill the robot reaches the lastest goal
                    wait = self.client.wait_for_result()
                    # If the robot reaches the lastest goal
                    if wait:
                        return "continue_find_person"
                else:
                    # pass  
                    wait = self.client.wait_for_result(rospy.Duration.from_sec(1.0))
                    # rospy.sleep(1)
            except Exception as e:
                pass

# completed
class Get_bounding_box(smach.State):
    def __init__(self):
        rospy.loginfo('Initiating state Get_bounding_box')
        smach.State.__init__(self, outcomes=['continue_stop','continue_find_person'])
        # connect to server
        host = socket.gethostname()
        port = 11000
        self.c = CustomSocket(host,port)
        self.c.clientConnect()

        self.found_target = False
        self.target_ID = None
        self.x_pixel = None
        self.y_pixel = None  
        self.intrinsics = None
        self.x_coord = None
        self.y_coord = None  
        self.lost_frame=0

        self.pub_tf = rospy.Publisher("/tf", tf2_msgs.msg.TFMessage, queue_size=1)
       
        self.bridge = CvBridge()


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

    def depth_callback(self, frame):

        if (self.x_pixel is None) or (self.y_pixel is None):
            return
        try:
            cv_image = self.bridge.imgmsg_to_cv2(frame, frame.encoding)
            # pick one pixel among all the pixels with the closest range:
            pix = (self.x_pixel, self.y_pixel)
            # line = '\rDepth at pixel(%3d, %3d): %7.1f(mm).' % (pix[0], pix[1], cv_image[pix[1], pix[0]])
            if self.intrinsics:
                depth = cv_image[pix[1], pix[0]]
                result = rs2.rs2_deproject_pixel_to_point(self.intrinsics, [pix[0], pix[1]], depth)
                x = result[0]/1000
                y = result[2]/1000
                # line += '  Coordinate: %8.2f %8.2f %8.2f.' % (result[0], result[1], result[2])
                if y>=0.5:
                    x_coord = x
                    y_coord = y

                    # line += '\r'
                    # print(line)
                    rospy.sleep(0.1)

                    t = TransformStamped()
                    t.header.frame_id = "realsense_mount_1"
                    t.header.stamp = rospy.Time.now()
                    t.child_frame_id = "/human_frame"
                    t.transform.translation.x = y_coord-1.0
                    t.transform.translation.y = -x_coord
                    t.transform.translation.z = 0.0

                    quat = tf.transformations.quaternion_from_euler(float(0),float(0),float(0))

                    t.transform.rotation.x = quat[0]
                    t.transform.rotation.y = quat[1]
                    t.transform.rotation.z = quat[2]
                    t.transform.rotation.w = quat[3]

                    
                    tfm = tf2_msgs.msg.TFMessage([t])
                    self.pub_tf.publish(tfm)

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
        self.frame = cv2.resize(self.frame, (640,480))
        
        # send frame to server and recieve the result      
        result = self.c.req(self.frame)

        move_realsense = Vector3()

        lost = True
        
        for track in result["result"]:
            
        # if enable info flag then print details about each track
            if not self.found_target:
                person_id = track[0]
                self.x_pixel = int((track[2][0]+track[2][2])/2)
                self.y_pixel = int((track[2][1]+track[2][3])/2)
                # print("Found Person to track (ID: {})".format(person_id))
                # print("Location of the person({}) : {} {}".format(person_id,self.x_pixel, self.y_pixel))

                lost = False
                self.found_target = True
                self.lost_frame=0

            elif track[0] == person_id:
                self.x_pixel = int((track[2][0]+track[2][2])/2)
                self.y_pixel = int((track[2][1]+track[2][3])/2)
                # print("Location of the person({}) : {} {}".format(person_id, self.x_pixel, self.y_pixel))

                lost = False
                self.lost_frame=0

                if track[2][0] <= 230 :
                    move_realsense.z = -1
                    
                
                if track[2][0] >= 250 :
                    move_realsense.z = 1
                    
                if track[2][1] <= 310 :
                    move_realsense.y = -1
                    
                if track[2][1] >= 330 :
                    move_realsense.y = 1
                
                if (track[2][0] > 230 and track[2][0] <= 250) and (track[2][1] > 310 and track[2][1] <= 330): 
                    move_realsense.x = 0
                    move_realsense.y = 0
                    move_realsense.z = 0
                
        move_realsense_pub = rospy.Publisher('servo_dir', Vector3, queue_size=1) 

        move_realsense_pub.publish(move_realsense)

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
        

        while True:
            if  target_lost == True:
                
                image_sub.unregister()
                depth_info_sub.unregister()
                depth_sub.unregister()
                return 'continue_find_person'
            
            elif is_stop == True:
                
                image_sub.unregister()
                depth_info_sub.unregister()
                depth_sub.unregister()
                return 'continue_stop'


class Rotate(smach.State):
    def __init__(self):
        rospy.loginfo('Initiating state Rotate')
        smach.State.__init__(self, outcomes=['continue_found_person','continue_stop_rotate'])
        self.move = Twist()
        self.found_person=False
    def execute(self, userdata):
        global target_lost, stop_rotate
        rospy.loginfo('Executing state Rotate')

        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

        # set all value to 0
        self.move.linear.x = 0.0
        self.move.linear.y = 0.0
        self.move.angular.z = 0
        self.pub.publish(self.move)

        rospy.sleep(2)

        duration = 20

        #set angular velocity to pi*4/duration rad/s
        self.move.angular.z = pi*4/duration
        self.pub.publish(self.move)

        start = time.time()
        while time.time() - start < duration:
            if not target_lost:
                self.move.angular.z = 0
                self.pub.publish(self.move)
                return 'continue_found_person'

        stop_rotate = True
        return 'continue_stop_rotate'


class Check_bounding_box(smach.State):
    def __init__(self):
        rospy.loginfo('Initiating state Check_bounding_box')
        smach.State.__init__(self, outcomes=['continue_found_person','continue_stop_rotate'])
        # connect to server
        host = socket.gethostname()
        port = 11000
        self.c = CustomSocket(host,port)
        self.c.clientConnect()
        
        self.found_target = False
        self.target_ID = None
        self.x_pixel = None
        self.y_pixel = None  
        self.intrinsics = None
        self.x_coord = None
        self.y_coord = None 
        self.lost_frame=0

        self.bridge = CvBridge()     

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

        except ValueError as e:
            return
        pass

    def yolo_callback(self, data): 

        global target_lost
        global person_id

        # change subscribed data to numpy.array and save it as "frame"
        self.frame = self.bridge.imgmsg_to_cv2(data,'bgr8')
        self.frame = cv2.resize(self.frame, (640,480))
        
        # send frame to server and recieve the result      
        result = self.c.req(self.frame)

        for track in result["result"]:
            if person_id == track[0]:
                target_lost = False
               
    def execute(self, userdata):
        global target_lost, stop_rotate
        image_sub = rospy.Subscriber("/camera/color/image_raw", Image , self.yolo_callback)
        depth_info_sub = rospy.Subscriber("/camera/depth/camera_info", CameraInfo , self.info_callback)
        rospy.loginfo('Executing state Check_bounding_box')
        while True:
            if  target_lost == False:
                image_sub.unregister()
                depth_info_sub.unregister()
                return 'continue_found_person'
            elif stop_rotate == True:
                image_sub.unregister()
                depth_info_sub.unregister()
                return 'continue_stop_rotate'


if __name__ == '__main__':
    # initiate ROS node
    rospy.init_node('carry_my_luggage')
    # initiate the global variable
    target_lost = False
    is_stop = False
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
        sm_follow = smach.Concurrence(outcomes=['Stop','Find_person'],
                                        default_outcome = 'Stop',
                                        outcome_map = {'Find_person':{'Stop_command':'continue_find_person','Follow_person':'continue_find_person','Get_bounding_box':'continue_find_person'}})        
        with sm_follow:
            smach.Concurrence.add('Stop_command',Stop_command())
            smach.Concurrence.add('Follow_person',Follow_person())
            smach.Concurrence.add('Get_bounding_box',Get_bounding_box())
        smach.StateMachine.add('FOLLOW', sm_follow, transitions={'Find_person':'FIND_PERSON','Stop':'Ask_if_arrived'})
    
        # Create sub smach state machine
        sm_find_person = smach.Concurrence(outcomes=['Stop_rotate','Found_person'], 
                                            default_outcome='Stop_rotate',
                                            outcome_map = {'Found_person':{'Check_bounding_box':'continue_found_person'}})
        with sm_find_person:
             smach.Concurrence.add('Rotate',Rotate())
             smach.Concurrence.add('Check_bounding_box',Check_bounding_box())
        smach.StateMachine.add('FIND_PERSON',sm_find_person, transitions={'Stop_rotate':'Aborted','Found_person':'FOLLOW'})

        # Set up                                                    
        sis = smach_ros.IntrospectionServer('Server_name',sm,'/PatterRoot')
        sis.start()
        
        outcome = sm.execute()

        rospy.spin()
        sis.stop()
    
