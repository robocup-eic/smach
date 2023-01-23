#!/usr/bin/env python

import rospy
import smach
import math
import smach_ros
from geometry_msgs.msg import Pose, PoseStamped, Twist

# Move base navigation modules
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
import time

from visualization_msgs.msg import Marker
from moveit_msgs.msg import DisplayTrajectory

# realsense
import pyrealsense2 as rs2
import cv2
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError

# # tf for visualization
import tf2_msgs
import tf
from geometry_msgs.msg import TransformStamped

# # computer vision
# import socket
from util.custom_socket import CustomSocket
from util.nlp_server import SpeechToText, speak
import threading

#Environment descriptor
from util.environment_descriptor import EnvironmentDescriptor
from util.custom_socket import CustomSocket
from util.realsense import Realsense

import numpy as np

# # output
from geometry_msgs.msg import Pose, PoseStamped

# transform to from cameralink to base link
import tf2_ros
import tf2_geometry_msgs

# lift
from std_msgs.msg import Bool, Int16

# cr3 command
import moveit_commander

import copy

class go_to_Navigation():
    def __init__(self):
        self.move_base_client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    
    def move(self,location):
        global ed
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now() - rospy.Duration.from_sec(1)
        print(location)
        goal.target_pose.pose = ed.get_robot_pose(location)
        print(ed.get_robot_pose(location))
        print(goal.target_pose.pose)
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
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    ## BEGIN_SUB_TUTORIAL wait_for_scene_update
    ##
    ## Ensuring Collision Updates Are Receieved
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## If the Python node dies before publishing a collision object update message, the message
    ## could get lost and the box will not appear. To ensure that the updates are
    ## made, we wait until we see the changes reflected in the
    ## ``get_known_object_names()`` and ``get_known_object_names()`` lists.
    ## For the purpose of this tutorial, we call this function after adding,
    ## removing, attaching or detaching an object in the planning scene. We then wait
    ## until the updates have been made or ``timeout`` seconds have passed
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

    rospy.Subscriber("done", Bool, lift_cb)

def add_table(table_name, scene,ed):
    print(table_name)
    
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
            joint_goal[5] = 3.00
            
            
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

def pre(move_group = moveit_commander.MoveGroupCommander("arm")):
            joint_goal = move_group.get_current_joint_values()
            print(joint_goal)
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

def go_to_pose_goal(pose = Pose(),move_group = moveit_commander.MoveGroupCommander("arm"),robot = moveit_commander.RobotCommander()):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    
    ## BEGIN_SUB_TUTORIAL plan_to_pose
    ##
    ## Planning to a Pose Goal
    ## ^^^^^^^^^^^^^^^^^^^^^^^
    ## We can plan a motion for this group to a desired pose for the
    ## end-effector:

    # pose_goal.position.x = 0.5
    # pose_goal.position.y = -0.2
    # pose_goal.position.z = 0.9

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

#***********************************************************************************************

class Start_signal(smach.State):
    def __init__(self):
        rospy.loginfo('Initiating Start_signal state')
        smach.State.__init__(self,outcomes=['A'])
        self.FRAME_COUNT_LIMIT = 5
        self.close_distance = 1 # meter
        self.moving_pub = rospy.Publisher("/walkie2/cmd_vel", Twist, queue_size=10)
        self.pub_realsense_pitch_absolute_command = rospy.Publisher("/realsense_pitch_absolute_command", Int16, queue_size=1)


    def execute(self,userdata):
        rospy.loginfo('Executing Start_signal state')

        global rs
        self.moving_msg = Twist()
        self.moving_msg.linear.x = 0.2

        # Detect door opening
        x_pixel, y_pixel = 1280/2, 720/2
        frame_count = 0

        # set realsense
        # self.pub_realsense_pitch_absolute_command.publish(0)

        while True:
            rospy.sleep(0.5)
            print('ddddd')
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
                while time.time() - start_time < 5:
                    rospy.loginfo("Moving Forward...")
                    self.moving_pub.publish(self.moving_msg)
                    rospy.sleep(0.1)
                # navigation.move("standby1")

                rospy.loginfo("Stop Moving Forward")
                self.moving_msg.linear.x = 0
                self.moving_pub.publish(self.moving_msg)
                break

            if distance > self.close_distance:
                frame_count += 1
            else:
                frame_count = 0
        
        navigation.move("standby1")
        rospy.loginfo("*****************READY****************")
        
        speak("Hello, I am walkie, the house service robot")
        rospy.logwarn("Hello, I am walkie, the house service robot")
        
        return 'A'

class Walkie_Rotate(smach.State) :
    def __init__(self):
        rospy.loginfo('Initiating Walkie_Rotate state')
        smach.State.__init__(self,outcomes=['B'])
        self.rotate_pub = rospy.Publisher("/walkie2/cmd_vel", Twist, queue_size=10)
        self.bridge = CvBridge()
    
    def execute(self,userdata):
        rospy.loginfo('Executing Walkie_Rotate state')
        global image_pub, personDescription

        def draw_bbox(frame, res):
            for id in res.keys():
                x, y, w, h, hand_raised = (res[id][k] for k in ("x", "y", "w", "h", "hand_raised"))
                # max_x, min_x, max_y, min_y, hand_raised = res[id]
                color = (0, 255, 0) if hand_raised else (0, 0, 255)
                cv2.rectangle(frame, (x, y), (x + w, y + h), color, 3)

        # find people raising hand
        rotate_msg = Twist()
        # rotate_msg.angular.z = 0.15
        rotate_msg.angular.z = 0.0

        # speak to start
        speak("Now I am looking for operator who need help")
        rospy.logwarn("Now I am looking for operator who need help")
        rospy.logfatal("Walkie is scan people rasing their hand")

        start_time = time.time()

        is_found = 0
        FRAME_THRES = 20
        frame_ori = None
        while is_found < FRAME_THRES:
            self.rotate_pub.publish(rotate_msg)
            frame = rs.get_image()
            frame_ori = copy.copy(frame)
            detections = HandRaising.req(frame)
            draw_bbox(frame, detections)
            for key in detections.keys():
                rospy.loginfo("found {} person, raised {}".format(len(detections), detections[key]["hand_raised"]))
                x_relative = detections[key]["x"] + (detections[key]["w"] / 2)

                # if (detections[key]["hand_raised"] == True) and (400 < x_relative < 800) :
                if (detections[key]["hand_raised"] == True):
                    cancel = Twist()
                    cancel.linear.x = 0
                    cancel.linear.y = 0
                    cancel.angular.z = 0
                    is_found += 1
            
            image_pub.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))

            # tell the person to show hand higher
            if time.time() - start_time > 10:
                # speak("Please raise your hand higher")
                start_time = time.time()


        self.rotate_pub.publish(cancel)
        rospy.sleep(1)

        # desc = personDescription.req(frame_ori)
        speak("I found someone raising their hand")
        rospy.logwarn("I found someone raising their hand")
        time.sleep(0.5)
        # speak(desc)
        # rospy.logwarn(desc)
        time.sleep(1.0)
        speak("I will come to you")
        rospy.logwarn("I will come to you")
        navigation.move("standby2")

        return 'B'

class to_bottle(smach.State):
    def __init__(self):
        rospy.loginfo('Initiating state Tobottle')
        smach.State.__init__(self, outcomes=['C'])

    def execute(self, userdata):
        rospy.loginfo('Executing state GetObjectName')
        # sending object name to GetobjectName state (change string right here)

        global count_group
        nav = ('shelf','comtable','comtable','pickwater','pickwater')


        navigation.move(nav[count_group])
        rospy.logfatal("Walkie is coming to u")
        
        if count_group == 0:
            speak("May I help you sir")
            rospy.logwarn("May I help you sir")
            stt.listen()
            rospy.logfatal("Walkie is listening")
            rospy.sleep(5)
            speak("ok")
            rospy.logwarn("ok")
        
        return 'C'


class Get_pose(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['D'])
        rospy.loginfo('Initiating state Pose')
        self.bridge = CvBridge()

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
        if count_group ==4:
            self.rotate_msg.angular.z = -0.2
        else:
            self.rotate_msg.angular.z = 0.2

        self.cancel = Twist()
        self.cancel.linear.x = 0
        self.cancel.linear.y = 0
        self.cancel.angular.z = 0

        self.bridge = CvBridge()

        #Transforming Pose
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)

    def execute(self, userdata):
        rospy.loginfo('Executing state Find_operator')
        global personTrack, ed, count_group

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
        def reset():
            rospy.loginfo("Reseting the value")
            self.frame = None
            rospy.sleep(0.1)
            rospy.loginfo("Finished reseting")
            self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, yolo_callback, queue_size=1, buff_size=52428800)
            self.depth_sub = rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, depth_callback, queue_size=1, buff_size=52428800)

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
                        
            if 1.0 < z_coord < 4:
                posi = Pose()
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
        # reset()
        start_time = time.time()
        while time.time()-start_time < 2:
            self.rotate_pub.publish(self.rotate_msg)
        
        while True:
            self.rotate_pub.publish(self.rotate_msg)
            
            if detect(rs.get_image()) :
                speak("I found operator") 
                self.rotate_pub.publish(self.cancel)
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
            is_correct = False
            for i in range(len(object_list[:4])):
                if is_correct == True:
                    break
                rospy.logfatal("Walkie get pointing obj")
                speak("You are pointing at " + object_list[i])
                rospy.logwarn("You are pointing at " + object_list[i])
                time.sleep(1)
                # speak("what should I do next?")
                # rospy.logwarn("what should I do next?")
                # stt.listen()
                # stt.clear()
            
            if count_group < 4:
                speak("Let's move to the next group")
                rospy.sleep(10)
                is_correct = False
                return 'B'
            else:
                speak("What should I do next")
                rospy.logwarn("What should I do next")
                stt.listen()
                stt.clear()
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

class GetObjectName(smach.State):
    def __init__(self):
        rospy.loginfo('Initiating state GetObjectName')
        smach.State.__init__(self, outcomes=['F'], output_keys=['objectname_output'])

    def execute(self, userdata):
        rospy.loginfo('Executing state GetObjectName')
        # sending object name to GetobjectName state (change string right here)
        userdata.objectname_output = OBJECT_NAME
        # speak("hi my name is walkie")
        # stt.listen()
        return 'F'


class GetObjectPose(smach.State):
    def __init__(self):
        global object_detection, navigation
        rospy.loginfo('Initiating state GetObjectPose')
        smach.State.__init__(self, outcomes=['continue_Pick','G'], input_keys=['objectname_input', 'objectpose_output'], output_keys=['objectpose_output'])
        # initiate variables
        self.object_name = ""
        self.center_pixel_list = [] # [(x1, y1, id), (x2, y2, id), ...] in pixels
        self.object_pose_list = [] # [(x1, y1, z1, id), (x1, y1, z1, id), ...] im meters
        self.intrinsics = None
        self.bridge = CvBridge()
        self.frame = None
        self.object_pose = Pose()
        self.tf_stamp = None

        # connect to CV server
        rospy.loginfo("connected object detection server")

        # realsense down
        self.pub_realsense_pitch_absolute_command = rospy.Publisher("/realsense_pitch_absolute_command", Int16, queue_size=1)
        self.pub_realsense_yaw_absolute_command = rospy.Publisher("/realsense_yaw_absolute_command", Int16, queue_size=1)

    def execute(self, userdata):
        rospy.loginfo('Executing state GetObjectPose')
        global object_detection

        def run_once():
            while self.intrinsics is None:
                time.sleep(0.1)
            rospy.loginfo("realsense image width, height = ({}, {})".format(self.intrinsics.width, self.intrinsics.height))
            object_detection.req(np.random.randint(255, size=(720, 1280, 3), dtype=np.uint8))

        def reset():
            rospy.loginfo("Reseting the value")
            self.frame = None
            rospy.sleep(0.1)
            rospy.loginfo("Finished reseting")
            self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, yolo_callback, queue_size=1, buff_size=52428800)
            self.depth_sub = rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, depth_callback, queue_size=1, buff_size=52428800)

        def detect():
            rospy.loginfo("Start detecting")
            # scale image incase image size donot match cv server
            self.frame = check_image_size_for_cv(self.frame)
            # send frame to server and recieve the result
            result = {"n":0}
            while result['n'] == 0:
                result = object_detection.req(self.frame)
            self.frame = check_image_size_for_ros(self.frame)
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
                    (x_pixel, y_pixel) = rescale_pixel(x_pixel, y_pixel)
                    object_id = 1 # TODO change to object tracker
                    self.center_pixel_list.append((x_pixel, y_pixel, object_id))
                    # visualize purpose
                    self.frame = cv2.circle(self.frame, (x_pixel, y_pixel), 5, (0, 255, 0), 2)
                    self.frame = cv2.rectangle(self.frame, rescale_pixel(bbox[3], bbox[4]), rescale_pixel(bbox[3] + bbox[5], bbox[4] + bbox[6]), (0, 255, 0), 2)
            
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(self.frame, "bgr8"))

            if len(self.center_pixel_list) == 0:
                return False
            # 3d pose
            if not self.intrinsics:
                rospy.logerr("no camera intrinsics")
                return False
            for center_pixel in self.center_pixel_list:
                rospy.loginfo("found {}".format(center_pixel))
                depth = self.depth_image[center_pixel[1], center_pixel[0]] # [y, x] for numpy array
                result = rs2.rs2_deproject_pixel_to_point(self.intrinsics, [center_pixel[0], center_pixel[1]], depth) # [x, y] for realsense lib
                x_coord, y_coord, z_coord = result[0]/1000, result[1]/1000, result[2]/1000

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
            speak(" I will pick a water bottle")
            rospy.logwarn(" I will pick a water bottle")
            rospy.logfatal("Walkie found obj to grasp")
            self.object_pose = find_closest_object()
            return True

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
        def info_callback(cameraInfo):
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

        def yolo_callback(data):
            try:
                # change subscribed data to numpy.array and save it as "frame"
                self.frame = self.bridge.imgmsg_to_cv2(data, 'bgr8')
            except CvBridgeError as e:
                print(e)

        def depth_callback(frame):
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
                if self.tf_stamp is not None:
                    # rospy.loginfo("publishing tf")
                    self.tf_stamp.header.stamp = rospy.Time.now()
                    self.pub_tf.publish(tf2_msgs.msg.TFMessage([self.tf_stamp]))

                self.depth_image = self.bridge.imgmsg_to_cv2(frame, frame.encoding)
                # rescale pixel incase pixel donot match
                self.depth_image = check_image_size_for_ros(self.depth_image)

            except CvBridgeError as e:
                print(e)
                return
            except ValueError as e:
                return
            pass

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
            "/camera/aligned_depth_to_color/camera_info", CameraInfo, info_callback)
        self.image_sub = rospy.Subscriber(
            "/camera/color/image_raw", Image, yolo_callback, queue_size=1, buff_size=52428800)
        self.depth_sub = rospy.Subscriber(
            "/camera/aligned_depth_to_color/image_raw", Image, depth_callback, queue_size=1, buff_size=52428800)
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
        self.pub_realsense_pitch_absolute_command.publish(-35)
        self.pub_realsense_yaw_absolute_command.publish(0)
        time.sleep(1)

        # arm sethome
        set_home_walkie()

        # lift down
        lift_command(False)

        # run_once function
        run_once()
        while not rospy.is_shutdown():
            # command = raw_input("Press Enter :")
            # if command == 'q':
            #     break
            rospy.loginfo("------ Running 3D detection ------")
            reset()
            if detect(): 
                userdata.objectpose_output = self.object_pose
                rospy.loginfo(self.object_pose)
                return 'continue_Pick'
        return 'G'


class Pick(smach.State):
    def __init__(self):
        rospy.loginfo('Initiating state Pick')
        smach.State.__init__(self, outcomes=['G'], 
                                   input_keys=['objectpose_input'])
        self.success = False
        self.tf_buffer =  tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)

    def execute(self, userdata):

        global gripper_publisher

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
            global move_group,robot
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

            # grasp_pose                  = pose_goal
            # pregrasp_pose               = pose_goal
            # pregrasp_pose.position.x    -=  0.2
            # lift_pose                   = pose_goal
            # lift_pose.position.z        += 0.1
            # lift_pose.position.x        -= 0.1

            grasp_pose .position.x    = pose_goal.position.x - 0.105
            grasp_pose .position.y    = pose_goal.position.y + 0.02
            grasp_pose .position.z    = pose_goal.position.z
            grasp_pose .orientation.x = pose_goal.orientation.x
            grasp_pose .orientation.y = pose_goal.orientation.y
            grasp_pose .orientation.z = pose_goal.orientation.z
            grasp_pose .orientation.w = pose_goal.orientation.w

            pregrasp_pose.position.x    = pose_goal.position.x - 0.2
            pregrasp_pose.position.y    = pose_goal.position.y
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
            # rospy.sleep(3)
            # raw_input("enter to open gripper")
            gripper_publisher.publish(False)
            # pre(move_group)
            # rospy.sleep(3)
            catesian_go(grasp_pose, move_group, robot)
            # raw_input("enter to close gripper")
            gripper_publisher.publish(True)
            # rospy.sleep(3)
            catesian_go(lift_pose, move_group, robot)
            # rospy.sleep(3)
            set_home_walkie(move_group)
            

            return True

            

        # lift up
        rospy.logfatal("Walkie is going to pick obj")
        lift_command(True)
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
            return 'G'
        else:
            return 'G'

class to_me(smach.State):
    def __init__(self):
        rospy.loginfo('Initiating state Tome')
        smach.State.__init__(self, outcomes=['H'])

    def execute(self, userdata):
        
        rospy.loginfo('Executing state Tome')
        # sending object name to GetobjectName state (change string right here)


        # set_home_walkie(move_group)
        serving(move_group)
        speak("here is your water")
        rospy.logwarn("here is your water")
        # raw_input("enter")
        rospy.sleep(3)
        gripper_publisher.publish(False)
        rospy.sleep(3)

        gripper_publisher.publish(True)
        set_home_walkie(move_group)
        # lift down
        lift_command(False)
        rospy.logfatal("task complete")
        return 'H'


if __name__ == '__main__':
    rospy.init_node('hand_me_that')

    object_list = []
    count_group = 0
    p = 3
    point_time = 0
    OBJECT_NAME = "waterbottle"

    navigation = go_to_Navigation()

    # host
    host = "0.0.0.0"

    # obj detect
    port_object = 10008
    object_detection = CustomSocket(host=host, port=port_object)
    object_detection.clientConnect()

    # what is that
    port_wtf = 10002
    what_is_that = CustomSocket(host, port_wtf)
    what_is_that.clientConnect()

    # hand raising detection
    port_HandRaising = 10011
    HandRaising = CustomSocket(host, port_HandRaising)
    HandRaising.clientConnect()

    # person tracker model
    port_personTrack = 11000
    personTrack = CustomSocket(host,port_personTrack)
    personTrack.clientConnect()

    # person description model
    port_personDescription = 10009
    personDescription = CustomSocket(host, port_personDescription)
    personDescription.clientConnect()
    
    # Flask nlp server
    stt = SpeechToText("nlp")
    stt.clear()
    t = threading.Thread(target = stt.run ,name="flask")
    t.start()

    ed = EnvironmentDescriptor("/home/eic/ros/smach/smach_task/config/fucku.yaml")
    scene = moveit_commander.PlanningSceneInterface()
    rospy.sleep(2)
    add_table("table",scene,ed)

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
    sm = smach.StateMachine(outcomes=['H'])
    with sm:
        smach.StateMachine.add('O', Start_signal(),
                               transitions={'A': 'A'})
        smach.StateMachine.add('A', Walkie_Rotate(),
                               transitions={'B': 'B'})
        smach.StateMachine.add('B', to_bottle(),
                               transitions={'C': 'C'})
        smach.StateMachine.add('C', Get_pose(),
                               transitions={'D': 'D'})
        smach.StateMachine.add('D', Text_to_speech(),
                               transitions={'E': 'E','continue_get_pose':'C','B':'R'})
        smach.StateMachine.add('R', Find_operator(),
                               transitions={'Q': 'B'})
        smach.StateMachine.add('E', GetObjectName(),
                               transitions={'F': 'F'},
                               remapping={'objectname_output': 'string_name'})
        smach.StateMachine.add('F', GetObjectPose(),
                               transitions={'G': 'G','continue_Pick': 'Pick'},
                               remapping={'objectname_input': 'string_name',
                                          'objectpose_output': 'object_pose'})
        smach.StateMachine.add('Pick', Pick(),
                               transitions={'G': 'G'},
                               remapping={'objectpose_input': 'object_pose'})
        smach.StateMachine.add('G', to_me(),
                               transitions={'H': 'H'})

        
    # Set up
    sis = smach_ros.IntrospectionServer('Server_name',sm,'/Root')
    sis.start()
    outcome = sm.execute()
    rospy.spin()
    sis.stop()