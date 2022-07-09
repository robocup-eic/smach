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
from geometry_msgs.msg import PoseStamped, Twist ,Vector3, TransformStamped, Point
from std_msgs.msg import String

# import computer vision and realsense
import cv2
import numpy as np
from util.realsense import Realsense

# import for speech-to-text
from flask import Flask, request
import threading

# import for text-to-speech
import requests
import json
from util.nlp_server import SpeechToText, speak
import time

#Bring in the simple action client
import actionlib

#Bring in the .action file and messages used by the move based action
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from util.custom_socket import CustomSocket
from cv_bridge import CvBridge, CvBridgeError #
from sensor_msgs.msg import Image, CameraInfo #
from actionlib_msgs.msg import GoalStatus

# Utils function
from math import atan, pi
from util.environment_descriptor import EnvironmentDescriptor
from util.realsense import Realsense

class Start_signal(smach.State):
    def __init__(self):
        rospy.loginfo('Initiating Start_signal state')
        smach.State.__init__(self,outcomes=['continue_Standby'])
        self.FRAME_COUNT_LIMIT = 5
        self.close_distance = 1 # meter
        self.move_base_client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        
    def execute(self,userdata):
        rospy.loginfo('Executing Start_signal state')

        global ed
        
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
        
        # navigate to standby position
        rospy.loginfo("moving to standby")
        standby = go_to_Navigation('standby')
        if standby:
            rospy.loginfo('Walky stand by, Ready for order')
        else:
            rospy.logerr('Navigation failed')
        
        return 'continue_Standby'

class Standby(smach.State):
    def __init__(self):
        rospy.loginfo('Initiating state Standby')
        smach.State.__init__(self, outcomes=['continue_follow'])
        self.has_bag = False
        # image publisher for visualize
        self.image_pub = rospy.Publisher("/blob/image_blob", Image, queue_size=1)
        self.bridge = CvBridge()
        
    def execute(self, userdata):
        global stt, is_stop, target_lost, rs
        is_stop = False
        target_lost = False
        rospy.loginfo('Executing state Standby')
        start_time = 0
        while True:

            # for visualize
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(rs.get_image(), "bgr8"))

            if stt.body is not None:
                print(stt.body)
                if stt.body["intent"] == "follow_people": # waiting for "follow me" command
                    stt.clear()
                    speak("I'm following you")
                    # Put bag hanger position code in here TODO
                    self.has_bag = True
                    return "continue_follow"

            if time.time() - start_time > 10 and not self.has_bag:
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

            
            if stt.body:
                rospy.loginfo(stt.body["intent"])

                if stt.body["intent"] == "affirm": # waiting for "follow me" command
                    stt.clear()
                    return "continue_place_luggage"

                elif stt.body["intent"] == "deny": # waiting for "carry my luggage" command
                    stt.clear()
                    speak("Please say follow me if you want me to follow you again")
                    return "continue_standby"

                else:
                    speak("Please say yes or no")
                    stt.clear()
                    stt.listen()
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
        self.detect_radius = 0.5
        
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        robot_inside = True

    def execute(self, userdata):
        

        rospy.loginfo('Executing state Check_position')
        global robot_inside, is_stop, target_lost, ed

        

        exit_position = ed.get_center_point("exit")

        while True:
            
            pose = self.tfBuffer.lookup_transform('map','base_footprint',rospy.Time.now()-rospy.Duration.from_sec(1.0))

            distance = ((pose.transform.translation.x-exit_position.x)**2+(pose.transform.translation.y-exit_position.y)**2)**0.5

            rospy.loginfo('Distance to Exit : {} m'.format(distance))

            if distance<self.detect_radius:

                robot_inside = False
                rospy.loginfo("Robot outside exit, Switch to non-move_base person follower")
                       
            if is_stop or target_lost:
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

        self.cancel = Twist()
        self.stop_pub = rospy.Publisher("/walkie2/cmd_vel",Twist,queue_size=1)

        self.cancel.linear.x = 0
        self.cancel.linear.y = 0
        self.cancel.angular.z = 0

        # follow with whole body
        self.follow_cmd = String()
        self.follow_cmd_pub = rospy.Publisher("/human/follow_cmd",String,queue_size=1)

        self.follow_cmd.data = "follow"
        self.is_cancelled = False

        # realsense follow
        self.realsense_follow_cmd = String()
        self.realsense_follow_cmd_pub = rospy.Publisher("/human/realsense_follow_cmd", String, queue_size=1)
    def execute(self, userdata):

        rospy.loginfo('Executing state Follow_person')
        global target_lost, is_stop, last_pose, robot_inside
        pose = TransformStamped()

        goal_send_interval = 1.5 # send goal at least every 5 seconds or wait until previous goal.
        start_time = 0

        self.is_cancelled = False
        
        while True:

            if robot_inside :
                
                self.follow_cmd.data = "stop"
                self.follow_cmd_pub.publish(self.follow_cmd)

                self.realsense_follow_cmd_pub.publish("follow")
                # rospy.loginfo("realsense follow command")

                try:

                    if time.time() - start_time > goal_send_interval:
                        pose = self.tfBuffer.lookup_transform('base_footprint','human_frame',rospy.Time.now()-rospy.Duration.from_sec(1.0))
                        goal = MoveBaseGoal()
                        goal.target_pose.header.frame_id = "base_footprint"
                        goal.target_pose.header.stamp = rospy.Time.now()-rospy.Duration.from_sec(1)
                        goal.target_pose.pose.position.x = pose.transform.translation.x
                        goal.target_pose.pose.position.y = pose.transform.translation.y
                        #TODO 
                        delta_x = pose.transform.translation.x
                        delta_y = pose.transform.translation.y
                        yaw = atan(delta_x/delta_y) # yaw
                        quarternion_orientation = tf.transformations.quaternion_from_euler(0, 0, yaw)
                        goal.target_pose.pose.orientation.x = quarternion_orientation[0]
                        goal.target_pose.pose.orientation.y = quarternion_orientation[1]
                        goal.target_pose.pose.orientation.z = quarternion_orientation[2]
                        goal.target_pose.pose.orientation.w = quarternion_orientation[3]

                        last_pose_tf = self.tfBuffer.lookup_transform('map','human_frame',rospy.Time.now()-rospy.Duration.from_sec(1.0))
                        self.client.send_goal(goal)
                        last_pose = (last_pose_tf.transform.translation.x, last_pose_tf.transform.translation.y, last_pose_tf.transform.translation.z)
                        start_time = time.time()

                        rospy.loginfo("Sending new goal: X,Y,Z is {}, {}, {}".format(pose.transform.translation.x,pose.transform.translation.y,pose.transform.translation.z))

                        if  is_stop:
                            self.client.cancel_goal()
                            self.stop_pub.publish(self.cancel)
                            self.realsense_follow_cmd_pub.publish("stop")
                            speak("I'm stopped")
                            return "continue_stop"

                        elif target_lost:
                            self.client.cancel_goal()
                            self.stop_pub.publish(self.cancel)
                            self.realsense_follow_cmd_pub.publish("stop")
                            speak("I have lost you, where are you my friend.")
                            return "continue_stop"
                    else:
                        
                        # wait = self.client.wait_for_result(rospy.Duration.from_sec(1.0))
                        if target_lost:
                            self.client.cancel_goal()
                            self.stop_pub.publish(self.cancel)
                            speak("I have lost you, where are you my friend.")
                            self.realsense_follow_cmd_pub.publish("stop")
                            return "continue_stop"


                except Exception as e:

                    # rospy.loginfo(e)

                    if  is_stop:
                        self.client.cancel_goal()
                        self.stop_pub.publish(self.cancel)
                        speak("I'm stopped")
                        self.realsense_follow_cmd_pub.publish("stop")
                        return "continue_stop"
                    
                    if target_lost:
                        self.client.cancel_goal()
                        self.stop_pub.publish(self.cancel)
                        speak("I have lost you, where are you my friend.")
                        self.realsense_follow_cmd_pub.publish("stop")
                        return "continue_stop"
            else:
                self.realsense_follow_cmd_pub.publish("stop")
                try:
                    if not self.is_cancelled:
                        self.client.cancel_goal()
                        self.is_cancelled = True
                        self.follow_cmd_pub.publish("follow")
                        
                    if  is_stop:
                        self.follow_cmd_pub.publish("stop")
                        self.stop_pub.publish(self.cancel)
                        speak("I'm stopped")
                        return "continue_stop"

                    elif target_lost:
                        self.follow_cmd.data = "stop"
                        self.follow_cmd_pub.publish(self.follow_cmd)
                        self.stop_pub.publish(self.cancel)
                        speak("I lost you, where are you my friend.")
                        return "continue_stop"

                except Exception as e:

                    if  is_stop:

                        self.follow_cmd.data = "stop"
                        self.follow_cmd_pub.publish(self.follow_cmd)
                        self.stop_pub.publish(self.cancel)
                        speak("I'm stopped")
                        return "continue_stop"

                    elif target_lost:
                        self.follow_cmd.data = "stop"
                        self.follow_cmd_pub.publish(self.follow_cmd)
                        self.stop_pub.publish(self.cancel)
                        speak("I lost you, where are you my friend.")
                        return "continue_stop"
                

class Get_bounding_box(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['continue_stop'])
        rospy.loginfo('Initiating state Get_bounding_box')

        # image
        self.x_pixel = None
        self.y_pixel = None
        self.bridge = CvBridge()

        # condition variable
        self.lost_frame = 0
        self.lost_threshold=30

        self.rel_pub  = rospy.Publisher("/human/rel_coor", Point, queue_size=1)
        self.abs_pub  = rospy.Publisher("/human/abs_coor", Point, queue_size=1)

        self.rel_point = Point()
        self.abs_point = Point()

        # image publisher for visualize
        self.image_pub = rospy.Publisher("/blob/image_blob", Image, queue_size=1)

    def execute(self, userdata):

        def detect(frame):
            global rs, target_lost, person_id, last_pose, personTrack
            
            lost = True
            # scale image incase image size donot match cv server
            frame = rs.check_image_size_for_cv(frame)
            # send frame to server and recieve the result
            result = personTrack.req(frame)
            # rescale pixel incase pixel donot match
            frame = rs.check_image_size_for_ros(frame)
            # check if there are any person
            if len(result["result"]) == 0:
                self.image_pub.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))
                self.lost_frame += 1
                rospy.loginfo("Lost target for {} consecutive frames".format(self.lost_frame))

                if self.lost_frame >= self.lost_threshold:
                    target_lost = True

                if last_pose is not None:
                    br = tf.TransformBroadcaster()
                    br.sendTransform(last_pose, tf.transformations.quaternion_from_euler(0, 0, 0),rospy.Time.now(),'human_frame', 'map')

                return 

            # not found person yet
            if person_id == -1:

                center_pixel_list = []
                for track in result["result"]:
                    self.x_pixel = int((track[2][0]+track[2][2])/2)
                    self.y_pixel = int((track[2][1]+track[2][3])/2)
                    depth = rs.get_coordinate(self.x_pixel, self.y_pixel, ref=(1280,720))[2] # numpy array
                    center_pixel_list.append((self.x_pixel, self.y_pixel, depth, track[0])) # (x, y, depth, perons_id)
                
                # find closest person
                person_id = min(center_pixel_list, key=lambda x: x[2])[3] # get person id with min depth
                rospy.loginfo("Found target person: target ID {}".format(person_id))

            for track in result["result"]:
                # track : [id, class_name, [x1,y1,x2,y2]]
                # rospy.loginfo("Track ID: {} at {}".format(track[0],track[2]))
                if track[0] == person_id:
                    self.x_pixel = int((track[2][0]+track[2][2])/2)
                    self.y_pixel = int((track[2][1]+track[2][3])/2)
                    self.x_pixel, self.y_pixel = rs.rescale_pixel(self.x_pixel, self.y_pixel)

                    lost = False
                    self.lost_frame = 0
                    # rospy.loginfo("Found Target")
            
                    # visualize purpose
                    frame = cv2.circle(frame, (self.x_pixel, self.y_pixel), 5, (0, 255, 0), 2)
                    frame = cv2.rectangle(frame, rs.rescale_pixel(track[2][0], track[2][1]), rs.rescale_pixel(track[2][2], track[2][3]), (0, 255, 0), 2)
                    frame = cv2.putText(frame, str(person_id), rs.rescale_pixel(track[2][0], track[2][1] + 10), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)

            self.image_pub.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))

            # check if person tracker can find any person
            if lost:

                self.lost_frame += 1

                rospy.loginfo("Lost target for {} consecutive frames".format(self.lost_frame))

                if self.lost_frame >= self.lost_threshold:
                    target_lost = True

                if last_pose is not None:
                    br = tf.TransformBroadcaster()
                    br.sendTransform(last_pose, tf.transformations.quaternion_from_euler(0, 0, 0),rospy.Time.now(),'human_frame', 'map')
            
            else:
                # 3d pose
                x_coord, y_coord, z_coord = rs.get_coordinate(self.x_pixel, self.y_pixel, ref=(frame.shape[1], frame.shape[0]))
            
                rospy.loginfo("Target is at {}, {}, {}".format(x_coord, y_coord, z_coord))

                self.abs_point.x = x_coord
                self.abs_point.y = y_coord
                self.abs_point.z = z_coord

                self.rel_point.x = self.x_pixel - frame.shape[1]/2
                self.rel_point.y = self.y_pixel - frame.shape[0]/2

                self.abs_pub.publish(self.abs_point)
                self.rel_pub.publish(self.rel_point)

                if 0.5 < z_coord < 8:
                    # rospy.loginfo("Target is at: ({}, {})".format(self.x_pixel, self.y_pixel))
                    # rospy.loginfo("Depth is {}".format(depth))
                    # rospy.loginfo("Detected at (x,y,z): {}".format([r/1000 for r in result]))
                    # rospy.sleep(0.01)

                    # camera frame for tf x is point toward and y is point left.
                    x = z_coord - 0.5 # set the goal point to be 1 meter away from person
                    y = -x_coord
                    z = -y_coord
                    br = tf.TransformBroadcaster()
                    br.sendTransform((x, y, z),tf.transformations.quaternion_from_euler(0, 0, 0),rospy.Time.now(),'human_frame','realsense_pitch')

        rospy.loginfo('Executing state Get_bounding_box')
        global target_lost, is_stop, person_id, last_pose

        # reset variable
        self.x_pixel = None
        self.y_pixel = None
        person_id = -1
        self.lost_frame = 0
        rospy.sleep(0.5)

        # SKIP_FRAME = 10
        # frame_count = 0
        # random_image = np.random.randint(255, size=(720,1280,3),dtype=np.uint8)

        # reset
        rs.reset()
        while True:
            # if frame_count < SKIP_FRAME:
            #     rospy.loginfo(frame_count)
            #     detect(random_image)
            #     frame_count += 1
            # else:
            detect(rs.get_image())
            rospy.sleep(0.01)

            if  target_lost == True:
                return 'continue_stop'
            
            elif is_stop == True:
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
    last_pose = None

    ed = EnvironmentDescriptor('/home/eic/ros/smach/smach_task/config/fur_data.yaml')
    ed.read_yaml()

    # person tracker
    host = "0.0.0.0"
    port_personTrack = 11000
    personTrack = CustomSocket(host,port_personTrack)
    personTrack.clientConnect()

    rs = Realsense()
    rs.wait() # wait for camera intrinsics

    # Flask nlp server
    stt = SpeechToText("nlp")
    t = threading.Thread(target = stt.run ,name="nlp")
    t.start()

    rs = Realsense()
    rs.wait() # wait for camera intrinsics

    # start state machine
    sm = smach.StateMachine(outcomes=['Succeeded','Aborted'])
    with sm:
        smach.StateMachine.add('Start_signal',Start_signal(),
                                transitions={'continue_Standby':'Standby'})

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