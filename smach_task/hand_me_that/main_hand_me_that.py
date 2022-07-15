#!/usr/bin/env python

"""
kill flask in background
kill -9 $(lsof -t -i:5000)
"""
# smach
import roslib
import rospy
import smach
import smach_ros

# navigation
import tf2_ros
from nav_msgs.msg import Odometry
from math import pi, atan
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

# ros pub sub
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import Bool
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo

# other utility
from util.custom_socket import CustomSocket
from util.nlp_server import SpeechToText, speak
from util.realsense import Realsense
from util.environment_descriptor import EnvironmentDescriptor
import time
import threading

# other
import math


class Start_signal(smach.State):
    def __init__(self):
        rospy.loginfo('Initiating Start_signal state')
        smach.State.__init__(self,outcomes=['continue_Standby'])
        self.FRAME_COUNT_LIMIT = 5
        self.close_distance = 1 # meter
        
        
        self.moving_pub = rospy.Publisher("/walkie2/cmd_vel", Twist, queue_size=10)


    def execute(self,userdata):
        rospy.loginfo('Executing Start_signal state')

        global rs

        self.moving_msg = Twist()
        self.moving_msg.linear.x = 0.2

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

                # move forward
                #Moving through entrance door
                start_time = time.time()
                while time.time() - start_time < DOOR_TIME:
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
        
        return 'continue_Standby'

class Standby(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['continue_follow','continue_pointing'])
        rospy.loginfo('Initiating state Standby')

        self.rotate_pub = rospy.Publisher("/walkie2/cmd_vel", Twist, queue_size=10)
        self.rotate_msg = Twist()
        self.rotate_msg.angular.z = -0.1

        self.cancel = Twist()
        self.cancel.linear.x = 0
        self.cancel.linear.y = 0
        self.cancel.angular.z = 0

    def execute(self, userdata):
        rospy.loginfo('Executing state Standby')
        global personTrack, ed

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

                return output_pose_stamped.pose


            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                raise

        def detect(frame):
            global posi
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
            frame = cv2.putText(frame, str(self.person_id), rs.rescale_pixel(track[2][0], track[2][1] + 10), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)

            image_pub.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))
            # 3d pose

            rospy.loginfo("X_pixel: {}, Y_pixel: {}".format(self.x_pixel, self.y_pixel))
            # rescale pixel incase pixel donot match

            x_coord, y_coord, z_coord = rs.get_coordinate(self.x_pixel, self.y_pixel, ref=(frame.shape[1], frame.shape[0]))
            rospy.loginfo("Target person is at coordinate: {}".format((x_coord, y_coord, z_coord)))
            # data from comuter vision realsense x is to the right, y is to the bottom, z is toward.
                        
            if 1.0 < z_coord < 4:
                rospy.sleep(0.1)
                posi.position.x, posi.position.y, posi.position.z = z_coord-0.5 , -x_coord, 0

                human_posi = transform_pose(posi, "realsense_pitch", "base_footprint")

                delta_x = human_posi.position.x
                delta_y = human_posi.position.y
                yaw = math.atan(delta_y/delta_x) # yaw

                posi.position.x, posi.position.y, posi.position.z = human_posi.position.x, human_posi.position.y, human_posi.position.z
                posi.orientation.x, posi.orientation.y, posi.orientation.z, posi.orientation.w = tf.transformations.quaternion_from_euler(0, 0, yaw)
                posi = transform_pose(posi, "base_footprint", "map")
                if ed.out_of_areana(posi):
                    rospy.loginfo("Human out of arena")
                    return False
                return True

            rospy.loginfo("Human out of range")
            return False
        # -------------------------------------------------------------------------------------
        
        # rotating until find person
        while True:
            self.rotate_pub.publish(self.rotate_msg)
            if detect(rs.get_image()) :
                speak("found operator") 
                self.rotate_pub.publish(self.cancel)
                break
            time.sleep(0.01)
            
        self.rotate_pub.publish(self.cancel)
        return 'continue_OBJECT_DETECTION'

class Pose(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['continue_text_to_speech'],
                                    output_keys=['pose_output'])
        rospy.loginfo('Initiate state Pose')
        self.bridge = CvBridge()

        # initiate list_object and countFrame
        self.list_object=[]
        self.countFrame=0
    
    def callback(self, data):
        # change subscribed data to numpy.array and save it as "frame"
        self.frame = self.bridge.imgmsg_to_cv2(data,'bgr8')
        self.frame = cv2.resize(self.frame,(1280,720))
        
        # send frame to server and recieve the result      
        result = what_is_that.req(self.frame)
        
        # add countFrame counter and append the object to the list
        self.countFrame += 1
        if len(result['what_is_that']) > 0:
            self.list_object.append(str(result['what_is_that'][0]))
        
        # Print list of detected objects
        # print("list_object = ",self.list_object)
        
        # check number of frame
        # print("counter frame = " + str(self.countFrame))
            
    def execute(self, userdata):
        rospy.loginfo('Executing state Pose')
        self.list_object=[]
        self.countFrame=0

        self.sub = rospy.Subscriber("/camera/color/image_raw", Image, self.callback)
        
        # wait to capture 5 frame
        while self.countFrame < 40:
            rospy.sleep(0.01)
        
        # stop subscribing /camera/color/image_raw
        self.sub.unregister()
        
        # if there is no object
        if len(self.list_object) == 0:
            userdata.pose_output = []
            return 'continue_text_to_speech'
        # if there is an object, find most common objecteeee
            print("You' =", max(set(self.list_object), key=self.list_object.count))
            object_list_unique = list(set(self.list_object))
            userdata.pose_output = object_list_unique.sort(key=self.list_object.count)
            return 'continue_text_to_speech'


class Text_to_speech(smach.State):
    def __init__(self):
        
        
        
        
        ded'
    

if __name__ == '__main__':
    rospy.init_node('hand_me_that')

    ###################################################
    DOOR_TIME = 6
    ###################################################

    # what is that
    host = "0.0.0.0"
    port = 10008
    what_is_that = CustomSocket(host, port)
    what_is_that.clientConnect()

    # person tracker model
    port_personTrack = 11000
    personTrack = CustomSocket(host,port_personTrack)
    personTrack.clientConnect()
    
    
    # Flask nlp server
    stt = SpeechToText("nlp")
    stt.clear()
    t = threading.Thread(target = stt.run ,name="flask")
    t.start()

    rs = Realsense()
    rs.wait() # wait for camera intrinsics

    image_pub = rospy.Publisher("/blob/image_blob", Image, queue_size=1)


    ed = EnvironmentDescriptor("../config/fur_data_onsite.yaml")

    # Start state machine
    sm = smach.StateMachine(outcomes=['Succeeded','Aborted'])
    with sm:
        smach.StateMachine.add("Start_signal", Start_signal(), transitions={"continue_standby":"Standby"})
        smach.StateMachine.add('Standby',Standby(),
                        transitions={'continue_pointing':'OBJECT_DETECTION'})
        
        # Create sub smach state machine "OBJECT_DETECTION"
        sm_object_detection = smach.StateMachine(outcomes=['Succeeded'])
        with sm_object_detection:
            smach.StateMachine.add('Pose', Pose(), transitions={'continue_text_to_speech':'Text_to_speech'},
                                                   remapping={'pose_input':'object_is'})
            smach.StateMachine.add('Text_to_speech', Text_to_speech(), transitions={'continue_succeeded':'Succeeded'},
                                                                        remapping={'tts_input':'object_is'})
        smach.StateMachine.add('OBJECT_DETECTION', sm_object_detection, transitions={'Succeeded':'Standby'})
        
    # Set up
    sis = smach_ros.IntrospectionServer('Server_name',sm,'/Root')
    sis.start()
    outcome = sm.execute()
    rospy.spin()
    sis.stop()