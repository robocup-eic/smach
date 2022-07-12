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
import tf2_geometry_msgs

import math

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

        goal.target_pose.pose.position = position.position
        goal.target_pose.pose.position.z = 0
        goal.target_pose.pose.orientation = position.orientation

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
        smach.State.__init__(self,outcomes=['continue_Navigate_living_room'])
        self.FRAME_COUNT_LIMIT = 5
        self.close_distance = 1 # meter

        self.moving_pub = rospy.Publisher("/walkie2/cmd_vel", Twist, queue_size=10)
    
    def execute(self,userdata):
        rospy.loginfo('Executing Start_signal state')
        # wait for the door to open
        global rs
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

        #Pub sub
        self.stop_pub = rospy.Publisher("/walkie2/cmd_vel",Twist,queue_size=1)
        self.cancel = Twist()
        self.cancel.linear.x = 0
        self.cancel.linear.y = 0
        self.cancel.angular.z = 0

        #Tranform manager
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)



    def execute(self, userdata):
        global posi, image_pub, navigation
        # turn on person tracker model
        # navigate to the center of living room during using person tracker
        # if the model detects a person cancel the goal and find a Pose() of that person and send it to Approach_person state
        # if the robot reaches goal and does not locate a person go to Find_person state

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

            # data from comuter vision realsense x is to the right, y is to the bottom, z is toward.
                        
            if 1.0 < z_coord < 4:
                rospy.sleep(0.1)
                posi.position.x, posi.position.y, posi.position.z = z_coord-0.3, -x_coord, 0

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
             
        rospy.loginfo('Executing Navigate_living_room state')
        # navigate to the center of living room during using person tracker
        
        # navigate to center of living room position and turn on person tracker
        result = navigation.move('livingroom')
        # navigation.move_no_block('living_room')
        while True:

            # result = navigation.move_base_client.get_state()
            # rospy.loginfo("status {}".format(result))
            if result:
                rospy.loginfo('Arrived at center of living room')
                return 'continue_Find_person'
            if detect(rs.get_image()):
                # found person
                speak("found person")
                # stop moving (cancel goal)
                navigation.move_base_client.cancel_goal()

                self.stop_pub.publish(self.cancel)
                
                # reset parameter
                self.x_pixel = None
                self.y_pixel = None
                self.person_id = -1
                
                return 'continue_Approach_person'
         # -------------------------------------------------------------------------------------



class Approach_person(smach.State):
    def __init__(self):
        rospy.loginfo('Initiating Approach_person state')
        smach.State.__init__(self,outcomes=['continue_Ask','continue_Find_person'])
    def execute(self, userdata):
        rospy.loginfo('Executing Approach_person state')
        global posi, navigation, count_person, save_posi
        # receive the person pose from Navigate_living_room state or Find_person state
        # approach the person in order to ask the person
        # reset the pose() to zero before return 
        reach_person = navigation.move_position(posi)

        if not reach_person:
            posi = Pose()
            return 'continue_Find_person'

        rospy.loginfo("Arrived at person#{}".format(count_person+1))
        
        # when the robot arrived at the destination save person's location and reset posi
 
        count_person += 1

        save_posi.position.x = posi.position.x+0.3
        save_posi.position.y, save_posi.position.z = posi.position.y, posi.position.z
        save_posi.orientation = posi.orientation

        posi = Pose()

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

        #Transforming Pose
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)

        # pub sub
        self.rotate_pub = rospy.Publisher("/walkie2/cmd_vel", Twist, queue_size=10)
        self.rotate_msg = Twist()
        self.rotate_msg.angular.z = 0.1

        self.stop_pub = rospy.Publisher("/walkie2/cmd_vel",Twist,queue_size=1)
        self.cancel = Twist()
        self.cancel.linear.x = 0
        self.cancel.linear.y = 0
        self.cancel.angular.z = 0

        self.rotate_pub = rospy.Publisher("/walkie2/cmd_vel", Twist, queue_size=10)
        

    def execute(self, userdata):
        rospy.loginfo('Executing Find_person state')
        global posi, image_pub, rs, tracked_person_id, personTrack
        # rotate 90 degree to avoid the first person

        # turn on person tracker model
        
        # rotate around to find the next person while turning a person tracker model
        # if the model detects a person cancel the goal and check the face in the database
        # if it does not match the database send pose  
        # send the pose to the approach person

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
                depth = rs.get_coordinate(self.x_pixel, self.y_pixel, ref=(1280,720))[2] # numpy array
                center_pixel_list.append((self.x_pixel, self.y_pixel, depth, track[0])) # (x, y, depth, perons_id)
            
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

            # data from comuter vision realsense x is to the right, y is to the bottom, z is toward.
                        
            if 1.0 < z_coord < 4:
                rospy.sleep(0.1)
                posi.position.x, posi.position.y, posi.position.z = z_coord-0.3 , -x_coord, 0

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
        
    
        while True:
            self.rotate_pub.publish(self.rotate_msg)
            if detect(rs.get_image()) : 
                self.stop_pub.publish(self.cancel)
                break
            
        self.rotate_pub.publish(self.cancel)
        return 'continue_Approach_person'
        


class Ask(smach.State):
    def __init__(self):
        rospy.loginfo('Initiating Ask state')
        smach.State.__init__(self,outcomes=['continue_Find_person',
                                            'continue_Navigate_to_start'])

        self.rotate_pub = rospy.Publisher("/walkie2/cmd_vel", Twist, queue_size=10)

        self.stop_pub = rospy.Publisher("/walkie2/cmd_vel",Twist,queue_size=1)
        self.cancel = Twist()
        self.cancel.linear.x = 0
        self.cancel.linear.y = 0
        self.cancel.angular.z = 0

        self.rotate_msg = Twist()
        self.rotate_msg.linear.x = 0
        self.rotate_msg.linear.y = 0
        self.rotate_msg.angular.z = 0.1

    def execute(self, userdata):
        global count_person, gm, save_posi
        rospy.loginfo('Executing Ask state')
        # ask name and save person's location
        speak("what is your name?")
        stt.listen()

        
        while True:
            if stt.body is not None:
                if stt.body["intent"] == "my_name" and "people" in stt.body.keys():
                    person_name = stt.body["people"]
                    gm.add_guest_name("guest_{}".format(count_person), person_name) #(role, name)
                    gm.add_guest_location("guest_{}".format(count_person),save_posi)
                    rospy.loginfo("Guest#{}'s name is {}".format(count_person, person_name))
                    rospy.loginfo("Guest#{}'s location is {}".format(count_person, save_posi))
                    
                    if count_person < 3:

                        # rotate 90 degree to avoid the first person
                        start_time = time.time()
                        while time.time() - start_time < 10:
                            rospy.loginfo("Rotating...")
                            self.rotate_pub.publish(self.rotate_msg)
                            rospy.sleep(0.1)
                        stt.clear()
                        return 'continue_Find_person'
                    else:
                        stt.clear()
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
        global navigation
        # walk back to the operator (location saved in yaml)

        rospy.loginfo("Moving to start point")
        
        standby = navigation.move('operator_position') #start point will be declare 2H before test

        return 'continue_Announce'


class Announce(smach.State):
    def __init__(self):
        rospy.loginfo('Initiating Announce state')
        smach.State.__init__(self,outcomes=['continue_SUCCEEDED'])
    def execute(self, userdata):
        rospy.loginfo('Executing Announce state')
        global gm, ed, count_person
        # announce the person's name 
        # compare the person's location to the furiture's location
        # compare with chair
        closest_locations = {}

        for i in range(1,count_person+1):

            guest_location = gm.get_guest_location("guest_{}".format(i))
            obj_dist = []

            for obj in ed.get_obj_poses(): ## TODO ed get chair list

                if not "position" in obj.keys():
                    continue

                obj_pose = obj["position"]
            
                distance = math.sqrt((guest_location.position.x - obj_pose.x)**2 + (guest_location.position.y - obj_pose.y)**2)

                obj_dist.append((obj['name'], distance))

            obj_dist = sorted(obj_dist, key=lambda x: x[1])
            closest_locations["guest_{}".format(i)] = obj_dist[0][0]
            rospy.loginfo('{}'.format(obj_dist))

        order_dict = {1:'first',2:'second',3:'third'}

        # announce the person's location relative to the closest furniture 
        speak("Hello, I found {} people in the room".format(len(count_person)))

        for i in range(count_person):
            speak("The {} person is {} which is located next to the {}".format(order_dict[i],gm.get_guest_name("guest_{}".format(i)), closest_locations["guest_{}".format(i)])) # TODO change furniture 
            # rospy.sleep(1)
            # speak("The second person is {} which is located next to the {}".format(gm.get_guest_name("guest_2"), closest_locations["guest_2"])) # TODO change furniture
            # rospy.sleep(2)
        speak("I have finished my task")
        return 'continue_SUCCEEDED'


if __name__ == '__main__':


    # initiate ros node
    rospy.init_node('find_my_friend_task')
    
    # initiate global variable
    count_person = 0
    posi = Pose()
    save_posi =Pose()

    ed = EnvironmentDescriptor("../config/fur_data_onsite.yaml")
    gm = GuestNameManager("../config/find_my_mate_database.yaml")
    
    # ed.visual_robotpoint()

    image_pub = rospy.Publisher("/blob/image_blob", Image, queue_size=1)

    navigation = go_to_Navigation()

    tracked_person_id = [-99]

    # connect to server
    host = "0.0.0.0"
    # host = socket.gethostname()

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
                               transitions={'continue_Ask':'Ask', 'continue_Find_person':'Find_person'})
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