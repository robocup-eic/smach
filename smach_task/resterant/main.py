#!/usr/bin/env python

"""
$ source walkie2_ws/devel/setup.bash
$ roslaunch realsense2_camera rs_rgbd.launch align_depth:=true color_width:=1280 color_height:=720 color_fps:=30 depth_width:=1280 depth_height:=720 depth_fps:=30 filters:=pointcloud
kill flask in background
$ kill -9 $(lsof -t -i:5000)
"""
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
from nlp_client import speak

import rospy
import smach
import smach_ros
import tf
import tf2_ros
import tf2_msgs
import tf2_geometry_msgs
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PoseStamped, Vector3, TransformStamped, Pose
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

def lift_cb(data) :
    while (not data.data) : pass

def wait_for_state_update(box_name,scene,box_is_known=False, box_is_attached=False, timeout=4):
    start = rospy.get_time()
    seconds = rospy.get_time()
    while (seconds - start < timeout) and not rospy.is_shutdown():
      # Test if the box is in attached objects
      attached_objects = scene.get_attached_objects([box_name])
      is_attached = len(attached_objects.keys()) > 0

      # Test if the box is in the scene.
      # Note that attaching the box will remove it from known_objects
      is_known = box_name in scene.get_known_object_names()

      # Test if we are in the expected state
      if (box_is_attached == is_attached) and (box_is_known == is_known):
        return True

      # Sleep so that we give other threads time on the processor
      rospy.sleep(0.1)
      seconds = rospy.get_time()

    # If we exited the while loop without returning then we timed out
    return False

def lift_command(cmd) :
    """
    lift command
    up is True
    down is False
    """

    time.sleep(1)
    lift_pub.publish(cmd)
    rospy.sleep(0.5)
    lift_pub.publish(cmd)

    rospy.Subscriber("done", Bool, lift_cb)

def add_table(table_name, scene):
    print(table_name)
    global ed
    cxl = []
    cyl = []

    height = ed.get_height(table_name)
    cxl,cyl = ed.cornervar(table_name)

    table_pose = PoseStamped()
    # table_pose.header.stamp = rospy.Time.now()
    table_pose.header.frame_id = "map"
    ed_point = ed.get_center_point(table_name)
    print(ed_point)
    table_pose.pose.position.x = ed_point.x
    table_pose.pose.position.y = ed_point.y
    table_pose.pose.position.z = ed_point.z + height/2

    length_x = max(cxl) - min(cxl)
    length_y = max(cyl) - min(cyl)

    table_pose.pose.orientation.w = 1.0
    print(table_pose)
    scene.add_box(table_name, table_pose, size=(0.01 ,0.01 ,0.01))
    return wait_for_state_update(table_name,scene,True)

def set_home_walkie(move_group = moveit_commander.MoveGroupCommander("arm")):
            joint_goal = move_group.get_current_joint_values()
            print(joint_goal)
            joint_goal[0] = 0.0
            joint_goal[1] = 0.0
            joint_goal[2] = 2.267
            joint_goal[3] = 0.875
            joint_goal[4] = 3.14
            joint_goal[5] = 2.355
            
            
            # joint_goal[0] = 0.0
            # joint_goal[1] = 0.0
            # joint_goal[2] = 0.0
            # joint_goal[3] = 0.0
            # joint_goal[4] = 0.0
            # joint_goal[5] = 0.0

            # raw_input("press enter to move to home position:")
            move_group.go(joint_goal, wait=True)
            print(joint_goal)
            move_group.stop()

def serving(move_group = moveit_commander.MoveGroupCommander("arm")):
            joint_goal = move_group.get_current_joint_values()
            print(joint_goal)
            joint_goal[0] = -2.6358
            joint_goal[1] = -0.258
            joint_goal[2] = -0.542
            joint_goal[3] = -0.604
            joint_goal[4] = -0.2322
            joint_goal[5] = -0.14

            move_group.go(joint_goal, wait=True)
            print(joint_goal)
            move_group.stop()

def pre(move_group = moveit_commander.MoveGroupCommander("arm")):
            joint_goal = move_group.get_current_joint_values()
            print(joint_goal)
            joint_goal[5] = 2.355
            move_group.go(joint_goal, wait=True)
            print(joint_goal)
            move_group.stop()

def go_to_pose_goal(pose = Pose(),move_group = moveit_commander.MoveGroupCommander("arm"),robot = moveit_commander.RobotCommander()):


    move_group.set_planner_id("SPARS")
    move_group.set_planning_time(5)
    move_group.set_pose_target(pose)
    m = Marker()
    m.header.frame_id = "base_footprint"
    m.header.stamp = rospy.Time.now()
    m.type = Marker.ARROW
    m.action = Marker.ADD
    m.pose = pose
    m.scale.x = 0.1
    m.scale.y = 0.05
    m.scale.z = 0.05
    m.color.a = 1
    m.color.r = 1
    m.color.g = 0
    m.color.b = 0

  
    rospy.sleep(5)
    a.publish(m)

    plan = move_group.plan()

    display_trajectory = DisplayTrajectory()
    display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory.append(plan)
    display_trajectory_publisher.publish(display_trajectory)
    

    # raw_input("using %s ,press enter:" %move_group.get_planner_id())

    move_group.execute(plan, wait=True)

def catesian_go(goal_pose = Pose(),move_group = moveit_commander.MoveGroupCommander("arm"),robot = moveit_commander.RobotCommander()):
    waypoints = []
    waypoints.append(goal_pose)

    plan, fraction = move_group.compute_cartesian_path(waypoints, 0.01, 0.0)

    display_trajectory = DisplayTrajectory()
    display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory.append(plan)
    display_trajectory_publisher.publish(display_trajectory)

    # raw_input("using catesian path with fraction %f ,press enter:" % fraction)
    move_group.execute(plan, wait=True)

#---------------------------------------------------------------

class fake(smach.State) :
    def __init__(self):
        rospy.loginfo('Initiating fake')
        smach.State.__init__(self,outcomes=['standby'])
        self.rotate_pub = rospy.Publisher("/walkie2/cmd_vel", Twist, queue_size=10)
    
    def execute(self,userdata):
        rospy.loginfo('Executing fake')
        speak("hello I am walkieeee")
        rospy.sleep(6)
        speak("ok, This is the bar")
        rospy.sleep(6)
        speak("ok")

        rotate_msg = Twist()
        rotate_msg.angular.z = 0.2

        start = rospy.Time.now()
        while rospy.Time.now() - start < rospy.Duration(5) :
            self.rotate_pub.publish(rotate_msg)
        
        self.rotate_pub.publish(Twist())

        return 'standby'


class Walkie_Rotate(smach.State) :
    def __init__(self):
        rospy.loginfo('Initiating Walkie_Rotate state')
        smach.State.__init__(self,outcomes=['To_cus'],output_keys=['posesave'])
        self.rotate_pub = rospy.Publisher("/walkie2/cmd_vel", Twist, queue_size=10)
        self.bridge = CvBridge()
        self.save = ()
    
    def execute(self,userdata):
        rospy.loginfo('Executing Walkie_Rotate state')
        global image_pub, personDescription, state
        state = "blank"

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
        speak("waiting for a customer to raise their hand")

        start_time = time.time()
        while True:
            # print(detect(rs.get_image()))
            if detect(rs.get_image()) == True:
                userdata.posesave = self.save
                break



        #desc = personDescription.req(frame_ori)
        speak("A customer raised their hand")
        time.sleep(0.5)
        # speak(desc)
        # time.sleep(1.0)
        speak("I'm coming")

        return 'To_cus'
            
class Walkie_Speak(smach.State) :
    def __init__(self):
        rospy.loginfo('Initiating Walkie_Speak')
        smach.State.__init__(self,outcomes=['to_bar','turn_around_walkie','continue_ABORTED'],output_keys=['order'])
    
    def execute(self,userdata):
        global state,order_name
        rospy.loginfo('Executing Walkie_Speak state')
        
            
        # listen
        # res_listen = listen()

        # userdata.order = res_listen['object']

        if state == "blank" :
            speak("order or bill")
        
            # state = listen()
            req = raw_input("req:")

            if req == "order" :
                speak("Can I get you something sir?")
                order_name = raw_input("order:")
                userdata.order= order_name
                
                # while True:
                    # res_listen = listen()
                    # if res_listen["intent"] == "restaurant_order" & 'object' in res_listen:
                    #     object = res_listen["object"]
                    #     break
                    # else:
                    #     print("Sorry I don't understand, Could you rephrase that?")
                        
                return "to_bar"
            elif req == "bill" :
                speak("your order list is orange juice 80 dollar")
                speak("notebook 20 dollar")
                speak("super drink 30 dollar")
                speak("so the total price is 131 dollar")
                return 'continue_ABORTED'

        elif state == "picked" :
            speak("This is your")
            return 'turn_around_walkie'

class to_bar(smach.State) :
    def __init__(self):
        rospy.loginfo('Initiating tobsr')
        smach.State.__init__(self,outcomes=['obj'])
    
    def execute(self,userdata):
        rospy.loginfo('Executing to nsd')
        bar = Pose()
        bar.orientation.w = 1
        navigation.nav2goal(bar)

        return 'obj'

class to_cutomer(smach.State):
    def __init__(self):
            rospy.loginfo('Initiatin to cud')
            smach.State.__init__(self,outcomes=['speak'],input_keys=['posesave'])
            self.tf_buffer =  tf2_ros.Buffer()
            self.listener = tf2_ros.TransformListener(self.tf_buffer)

        
    def execute(self,userdata):
        rospy.loginfo('Executing to cud')
        posesave = userdata.posesave

        # tune coordinate
        
        x = posesave[0]
        y = posesave[1]
        z = posesave[2]

        recieved_pose = Pose()
        recieved_pose.position.x = z-0.5
        recieved_pose.position.y = -x
        recieved_pose.position.z = -y
        recieved_pose.orientation.w = 1.0
        # recieved_pose.position.x -= 0.05
        # recieved_pose.position.y += 0.03
        # recieved_pose.position.z += 0.07

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
            recieved_pose, "realsense_pitch", "map")
        rospy.loginfo("***------------------------------------*****")
        # transformed_pose.position.x -= 1000
        # transformed_pose.orientation.x = 0
        # transformed_pose.orientation.y = 0
        # transformed_pose.orientation.z = 0
        # transformed_pose.orientation.w = 1

        # gosl = Pose()
        # gosl.position.x = transformed_pose.position.z
        # gosl.position.y = transformed_pose.position.x
        # gosl.orientation.w = 1

        rospy.loginfo(transformed_pose)

        navigation.nav2goal(transformed_pose)
        # navigation.nav2goal(recieved_pose,'realsense_pitch')

        return 'speak'

class GetObjectName(smach.State):
    def __init__(self):
        rospy.loginfo('Initiating state GetObjectName')
        smach.State.__init__(self, outcomes=['continue_GetObjectPose'], output_keys=['objectname_output'],input_keys=['order'])

    def execute(self, userdata):
        rospy.loginfo('Executing state GetObjectName')
        global GROUP
        # sending object name to GetobjectName state (change string right here)
        userdata.objectname_output = userdata.order

                
        return 'continue_GetObjectPose'

class GetObjectPose(smach.State):
    def __init__(self):
        global object_detection, navigation
        rospy.loginfo('Initiating state GetObjectPose')
        smach.State.__init__(self, outcomes=['continue_Pick', 'continue_ABORTED'], input_keys=['objectname_input', 'objectpose_output'], output_keys=['objectpose_output'])
        # initiate variables
        self.object_name = ""
        self.center_pixel_list = [] # [(x1, y1, id), (x2, y2, id), ...] in pixels
        self.object_pose_list = [] # [(x1, y1, z1, id), (x1, y1, z1, id), ...] im meters
        self.object_pose = Pose()
        self.tf_stamp = None

        # connect to CV server
        rospy.loginfo("connected object detection server")

        # realsense down
        self.pub_realsense_pitch_absolute_command = rospy.Publisher("/realsense_pitch_absolute_command", Int16, queue_size=1)
        self.pub_realsense_yaw_absolute_command = rospy.Publisher("/realsense_yaw_absolute_command", Int16, queue_size=1)

    def execute(self, userdata):
        rospy.loginfo('Executing state GetObjectPose')
        global object_detection, rs

        def run_once():
            while rs.intrinsics is None:
                time.sleep(0.1)
            rospy.loginfo("realsense image width, height = ({}, {})".format(self.intrinsics.width, self.intrinsics.height))
            object_detection.req(np.random.randint(255, size=(720, 1280, 3), dtype=np.uint8))

        def detect():
            global object_detection
            rospy.loginfo("Start detecting")
            # scale image incase image size donot match cv server
            self.frame = rs.check_image_size_for_cv(rs.frame)
            # send frame to server and recieve the result
            result = {"n":0}
            while result['n'] == 0:
                result = object_detection.req(self.frame)
            self.frame = rs.check_image_size_for_ros(self.frame)
            rospy.loginfo("result {}".format(result))
            if result['n'] == 0:
                return False 
            # object detection bounding box 2d
            for bbox in result['result']:
                if bbox[2] != self.object_name:
                    continue
                else:
                    # receive xyxy
                    x_pixel = int(bbox[3]+bbox[5]/2)
                    y_pixel = int(bbox[4]+bbox[6]/2)
                    (x_pixel, y_pixel) = rs.rescale_pixel(x_pixel, y_pixel)
                    object_id = 1 # TODO change to object tracker
                    self.center_pixel_list.append((x_pixel, y_pixel, object_id))
                    # visualize purpose
                    self.frame = cv2.circle(self.frame, (x_pixel, y_pixel), 5, (0, 255, 0), 2)
                    self.frame = cv2.rectangle(self.frame, rs.rescale_pixel(bbox[3], bbox[4]), rs.rescale_pixel(bbox[3] + bbox[5], bbox[4] + bbox[6]), (0, 255, 0), 2)
            
            self.image_pub.publish(rs.bridge.cv2_to_imgmsg(self.frame, "bgr8"))

            if len(self.center_pixel_list) == 0:
                return False
            # 3d pose
            if not self.intrinsics:
                rospy.logerr("no camera intrinsics")
                return False
            for center_pixel in self.center_pixel_list:
                rospy.loginfo("found {}".format(center_pixel))
                x_coord, y_coord, z_coord = rs.get_coordinate(center_pixel[0], center_pixel[1])

                # filter only object with more than 50 cm
                if z_coord >= 0.5:
                    rospy.sleep(0.1)
                    self.object_pose_list.append((x_coord, y_coord, z_coord, center_pixel[2]))

                    self.tf_stamp = TransformStamped()
                    self.tf_stamp.header.frame_id = "/realsense_pitch"
                    self.tf_stamp.header.stamp = rospy.Time.now()
                    self.tf_stamp.child_frame_id = "/object_frame_{}".format(center_pixel[2]) # object_id
                    self.tf_stamp.transform.translation.x = z_coord
                    self.tf_stamp.transform.translation.y = -x_coord
                    self.tf_stamp.transform.translation.z = -y_coord

                    quat = tf.transformations.quaternion_from_euler(
                        float(0), float(0), float(0))

                    self.tf_stamp.transform.rotation.x = quat[0]
                    self.tf_stamp.transform.rotation.y = quat[1]
                    self.tf_stamp.transform.rotation.z = quat[2]
                    self.tf_stamp.transform.rotation.w = quat[3]

            self.image_sub.unregister()
            self.depth_sub.unregister()
            rospy.loginfo("Object found!")
            print(" I found a water bottle")
            self.object_pose = find_closest_object()
            return True

        # function used in callback functions


        def find_closest_object():
            if len(self.object_pose_list) == 0:
                return None
            
            z_min = 10000000000
            for object_pose in self.object_pose_list:
                if object_pose[2] < z_min:
                    object_pose_z_min = object_pose
                    z_min = object_pose[2]
                    
            return xyz_to_pose(object_pose_z_min[0], object_pose_z_min[1], object_pose_z_min[2])

        def xyz_to_pose(x, y, z):
            """
            transform xyz in realsense coord to camera_link coord
            """
            # set object pose
            object_pose = Pose()
            object_pose.position.x = z
            object_pose.position.y = -x
            object_pose.position.z = -y
            object_pose.orientation.x = 0
            object_pose.orientation.y = 0
            object_pose.orientation.z = 0
            object_pose.orientation.w = 1
            return object_pose

        # all call_back functions


        def yolo_callback(data):
            try:
                # change subscribed data to numpy.array and save it as "frame"
                self.frame = self.bridge.imgmsg_to_cv2(data, 'bgr8')
            except CvBridgeError as e:
                print(e)



        def set_home_walkie():
            group_name = "arm"
            move_group = moveit_commander.MoveGroupCommander(group_name)
            joint_goal = move_group.get_current_joint_values()
            print(joint_goal)
            joint_goal[0] = 0.0
            joint_goal[1] = 0.0
            joint_goal[2] = 2.267
            joint_goal[3] = 0.875
            joint_goal[4] = 3.14
            joint_goal[5] = 2.355
            move_group.go(joint_goal, wait=True)
            move_group.stop()

        # ----------------------------------------------start-----------------------------------------------------
        # subscribe topics
        rospy.Subscriber(
            "/camera/aligned_depth_to_color/camera_info", CameraInfo, rs.info_callback)
        self.image_sub = rospy.Subscriber(
            "/camera/color/image_raw", Image, yolo_callback, queue_size=1, buff_size=52428800)
        self.depth_sub = rospy.Subscriber(
            "/camera/aligned_depth_to_color/image_raw", Image, rs.depth_callback, queue_size=1, buff_size=52428800)
        self.image_pub = rospy.Publisher(
            "/blob/image_blob", Image, queue_size=1)
        self.pub_tf = rospy.Publisher(
            "/tf", tf2_msgs.msg.TFMessage, queue_size=1)

        # recieving object name from GetObjectName state
        self.object_name = userdata.objectname_input
        rospy.loginfo(self.object_name)

        # print("navigating")
        # navigation.move('table')

        # navigation
        # while True:
        #     print("navigating")
        #     result = navigation.move('table')
        #     if result:
        #         break
        

        # realsense down
        for i in range(10) :
            self.pub_realsense_pitch_absolute_command.publish(-35)
            self.pub_realsense_yaw_absolute_command.publish(0)
            time.sleep(0.1)

        # arm sethome
        set_home_walkie()

        # lift down
        lift_command(False)
        for i in range(10) :
            lift_state.publish(0.0)
            realsense_pitch_angle.publish(-35)

        print("life go")

        # run_once function
        run_once()
        print("detect once")
        while not rospy.is_shutdown():
            
            rospy.loginfo("------ Running 3D detection ------")
            rs.reset()
            if detect(): 
                userdata.objectpose_output = self.object_pose
                rospy.loginfo(self.object_pose)
                return 'continue_Pick'
        return 'continue_ABORTED'

class Pick(smach.State):
    def __init__(self):
        rospy.loginfo('Initiating state Pick')
        smach.State.__init__(self, outcomes=['continue_navigate_volunteer', 'continue_ABORTED'], 
                                   input_keys=['objectpose_input'])
        self.success = False
        self.tf_buffer =  tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)

    def execute(self, userdata):

        global gripper_publisher, state

        rospy.loginfo('Executing state Pick')
        # recieving Pose() from GetObjectPose state
        recieved_pose = userdata.objectpose_input

        # tune coordinate
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
        def pick(pose = Pose()):
            group_name = "arm"
            move_group = moveit_commander.MoveGroupCommander(group_name)
            # scene = moveit_commander.PlanningSceneInterface()
            # rospy.sleep(2)
            # add_table("table",scene)
            robot = moveit_commander.RobotCommander()

            

            pose_goal = Pose()
            # q = quaternion_from_euler(3.14,-1.57,0)
            # pose_goal.orientation = Quaternion(*q)
            pose_goal.position.x = pose.position.x
            pose_goal.position.y = pose.position.y-0.05
            pose_goal.position.z = pose.position.z+0.05
            # q = quaternion_from_euler(0,1.57,0)
            # pose_goal.orientation = Quaternion(*q)
            pose_goal.orientation.x = -0.6668
            pose_goal.orientation.y = 0.24437
            pose_goal.orientation.z = -0.6384
            pose_goal.orientation.w = 0.29675

            grasp_pose      = Pose()
            pregrasp_pose   = Pose()
            lift_pose       = Pose()

            grasp_pose .position.x    = pose_goal.position.x - 0.108
            grasp_pose .position.y    = pose_goal.position.y + 0.018
            grasp_pose .position.z    = pose_goal.position.z
            grasp_pose .orientation.x = pose_goal.orientation.x
            grasp_pose .orientation.y = pose_goal.orientation.y
            grasp_pose .orientation.z = pose_goal.orientation.z
            grasp_pose .orientation.w = pose_goal.orientation.w

            pregrasp_pose.position.x    = pose_goal.position.x - 0.2
            pregrasp_pose.position.y    = pose_goal.position.y + 0.05
            pregrasp_pose.position.z    = pose_goal.position.z 
            pregrasp_pose.orientation.x = pose_goal.orientation.x
            pregrasp_pose.orientation.y = pose_goal.orientation.y
            pregrasp_pose.orientation.z = pose_goal.orientation.z
            pregrasp_pose.orientation.w = pose_goal.orientation.w

            lift_pose.position.x    = pose_goal.position.x - 0.2
            lift_pose.position.y    = pose_goal.position.y
            lift_pose.position.z    = pose_goal.position.z + 0.2
            lift_pose.orientation.x = pose_goal.orientation.x
            lift_pose.orientation.y = pose_goal.orientation.y
            lift_pose.orientation.z = pose_goal.orientation.z
            lift_pose.orientation.w = pose_goal.orientation.w

            print(grasp_pose)
            print(pregrasp_pose)
            print(lift_pose)
            
            go_to_pose_goal(pregrasp_pose, move_group, robot)
            rospy.sleep(3)
            # raw_input("enter to open gripper")
            gripper_publisher.publish(False)
            # pre(move_group)
            rospy.sleep(3)
            catesian_go(grasp_pose, move_group, robot)
            # raw_input("enter to close gripper")
            gripper_publisher.publish(True)
            rospy.sleep(3)
            catesian_go(lift_pose, move_group, robot)
            set_home_walkie()
            # rospy.sleep(3)
            # set_home_walkie(move_group)
            # serving(move_group)
            # speak("here is your water")
            # raw_input("enter")
            # rospy.sleep(2)
            # gripper_publisher.publish(False)
            # raw_input("enter to home")
            # rospy.sleep(2)
            # gripper_publisher.publish(True)
            # navigation.move("back")
            # set_home_walkie(move_group)
            # lift down
            lift_command(False)
            

            return True

            

        lift_command(True)
        for i in range(10) :
            lift_state.publish(0.24)
            realsense_pitch_angle.publish(-35)

        
        rospy.sleep(5)

        transformed_pose = transform_pose(
            recieved_pose, "realsense_pitch", "base_footprint")
        rospy.loginfo(transformed_pose.position.x)
        transformed_pose.orientation.x = 0
        transformed_pose.orientation.y = 0
        transformed_pose.orientation.z = 0
        transformed_pose.orientation.w = 1

        picksucess = False
        picksucess = pick(transformed_pose)
        
        if picksucess == True:
            # navigation.move("back") #comment 7.11PM mon 7 nov 2565
            state = "picked"
            return 'continue_navigate_volunteer'
        else:
            return 'continue_ABORTED'



#---------------------------------------------------------------

if __name__ == '__main__':
    rospy.init_node('restaurant_task')
    drag = ''
    tf_Buffer = tf2_ros.Buffer()
    navigation = go_to_Navigation()
    ed = EnvironmentDescriptor("fucku.yaml")
    # ed.visual_robotpoint()
    # publisher and subscriber
    lift_pub                     = rospy.Publisher('lift_command', Bool, queue_size=1)
    lift_state                   = rospy.Publisher('lift_state', Float32, queue_size=1)
    realsense_pitch_angle            = rospy.Publisher('realsense_pitch_angle', Int16, queue_size=1)
    a                            = rospy.Publisher("posem",Marker,queue_size=1)
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',DisplayTrajectory, queue_size=1)
    gripper_publisher            = rospy.Publisher('/cr3_gripper_command',Bool,queue_size=1)
    # fucklift                     = rospy.Publisher('/lift_')
    # connect to server
    host = "0.0.0.0"
    # host = socket.gethostname()
    # hand raising detection
    port_HandRaising = 10011
    HandRaising = CustomSocket(host, port_HandRaising)
    HandRaising.clientConnect()

    port_object = 10012
    object_detection = CustomSocket(host=host, port=port_object)
    object_detection.clientConnect()

    # person description model
    port_personDescription = 10009
    personDescription = CustomSocket(host, port_personDescription)
    personDescription.clientConnect()

    rs = Realsense()
    rs.wait() # wait for camera intrinsics

    # Flask nlp server

    scene = moveit_commander.PlanningSceneInterface()
    rospy.sleep(2)
    add_table("table",scene)
    # Create a SMACH state machine
    sm_top = smach.StateMachine(outcomes=['SUCCEEDED','ABORTED'])
    sm_top.userdata.posesave = ()
    sm_top.userdata.order = ""

    image_pub = rospy.Publisher("/blob/image_blob", Image, queue_size=1)

    # Open the container
    with sm_top:
        # Add states to the container
        # smach.StateMachine.add('Turn_Around_Walkie', Turn_Around_Walkie(),
        #                         transitions={'continue_Find_Hand_raising':'Find_Hand_raising'})
        smach.StateMachine.add('fake', fake(),
                                transitions={'standby':'Turn_Around_Walkie'})

        smach.StateMachine.add('Turn_Around_Walkie', Walkie_Rotate(),
                                transitions={'To_cus':'To_cus'},
                                remapping={'posesave':'posesave'})

        smach.StateMachine.add('Walkie_Speak', Walkie_Speak(),
                                transitions={'to_bar':'to_bar',
                                             'turn_around_walkie' : 'Turn_Around_Walkie',
                                             'continue_ABORTED' : 'ABORTED'},
                                remapping={'order':'order'})

        smach.StateMachine.add('to_bar',to_bar(),
                                transitions={'obj':'GetObjectName'})

        smach.StateMachine.add('To_cus', to_cutomer(),
                                transitions={'speak':'Walkie_Speak'},
                                remapping={'posesave':'posesave'})
        # ------------------------------ Pick Object --------------------------------------
        smach.StateMachine.add('GetObjectName', GetObjectName(),
                               transitions={'continue_GetObjectPose': 'GetObjectPose'},
                               remapping={'objectname_output': 'string_name',
                                            'order':'order'})
        smach.StateMachine.add('GetObjectPose', GetObjectPose(),
                               transitions={'continue_Pick': 'Pick',
                                            'continue_ABORTED': 'ABORTED'},
                               remapping={'objectname_input': 'string_name',

                                          'objectpose_output': 'object_pose',
                                          'objectpose_output': 'object_pose'})
        smach.StateMachine.add('Pick', Pick(),
                               transitions={'continue_navigate_volunteer': 'GetObjectName',
                                            'continue_ABORTED': 'ABORTED'},
                               remapping={'objectpose_input': 'object_pose'})
        # ----------------------------------------------------------------------------------
        
    sis = smach_ros.IntrospectionServer('Server_name', sm_top, '/Restaurant_task')
    sis.start()
    # Execute SMACH plan
    outcome = sm_top.execute()
    rospy.spin()
    sis.stop()