#!usr/bin/env python

import rospy
import roslib
import smach
import smach_ros

from geometry_msgs.msg import Pose

# realsense and computer vision
import requests
import numpy as np
import cv2
from geometry_msgs.msg import PoseStamped, Twist ,Vector3, TransformStamped, Pose, Point
from std_msgs.msg import Bool,Int64
import socket
from util.custom_socket import CustomSocket
from util.realsense import Realsense 

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

# import for text-to-speech
import requests
import json
from util.nlp_server import SpeechToText, speak
import time

# import yaml reader
from util.guest_name_manager import GuestNameManager
from util.environment_descriptor import EnvironmentDescriptor

# pick place service
from cr3_moveit_control.srv import PickWithSide
from cr3_moveit_control.srv import cr3_place

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

class Start_signal(smach.State):
    def __init__(self):
        rospy.loginfo('Initiating Start_signal state')
        smach.State.__init__(self,outcomes=['continue_Navigate_object'])
        self.FRAME_COUNT_LIMIT = 5
        self.close_distance = 1 # meter
        
    def execute(self,userdata):
        rospy.loginfo('Executing Start_signal state')
        # wait for the door to open

        global rs, navigation
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
       

   
        return 'continue_Navigate_object'

class Navigate_object(smach.State):
    def __init__(self):
        rospy.loginfo('Initiating Navigate_object state')
        smach.State.__init__(self, outcomes=['continue_ABORTED', 'continue_Get_pose'])

    def execute(self, userdata):
        rospy.loginfo('Execute Navigate_object state')
        global count_location, navigation
        count_location += 1
        rospy.loginfo('Number of location = %s', count_location)
        # navigate to the fixed location of an object (each object has different location)
        # in this case we will do only 2 objects which are cereal box and milk carton
        # count number of object that the robot has reached

        # 
        if count_location == 1:
            navigation.move(MILK_FUR)
            return 'continue_Get_pose'
        elif count_location == 2:
            navigation.move(CEREAL_FUR)
            return 'continue_Get_pose'
        else:
            return 'continue_ABORTED'


class Get_pose(smach.State):
    def __init__(self):
        rospy.loginfo('Initiating Get_pose state')
        smach.State.__init__(self, outcomes=['continue_Pick','continue_Navigate_object'],
                             input_keys = ['Get_pose_out'],
                             output_keys = ['Get_pose_out'])
    def execute(self, userdata):
        rospy.loginfo('Executing Get_pose state')
        global count_location 
        # getting the object pose and send it to the Pick state
        # sending Pose() to Pick state
        userdata.Get_pose_out.position.x = 1
        userdata.Get_pose_out.position.y = 2
        userdata.Get_pose_out.position.z = 3
        userdata.Get_pose_out.orientation.x = 4
        userdata.Get_pose_out.orientation.y = 5
        userdata.Get_pose_out.orientation.z = 6        
        userdata.Get_pose_out.orientation.w = 7
        return 'continue_Pick'

class Get_pose(smach.State):
    def __init__(self):
        rospy.loginfo('Initiating state GetObjectPose')
        smach.State.__init__(self, outcomes=['continue_Pick', 'continue_Navigate_object'], 
                                   input_keys=['Get_pose_out'],
                                   output_keys=['Get_pose_out'])
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
        host = "192.168.8.99"
        port = 10001
        self.c = CustomSocket(host, port)
        self.c.clientConnect()
        rospy.loginfo("connected object detection server")

    def execute(self, userdata):
        rospy.loginfo('Executing state GetObjectPose')
        global count_location
        
        def run_once():
            while self.intrinsics is None:
                time.sleep(0.1)
            rospy.loginfo("realsense image width, height = ({}, {})".format(self.intrinsics.width, self.intrinsics.height))
            self.c.req(np.random.randint(255, size=(720, 1280, 3), dtype=np.uint8))

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
            result = self.c.req(self.frame)
            self.frame = check_image_size_for_ros(self.frame)
            rospy.loginfo("result {}".format(result))
            if result['n'] == 0:
                return None 
            # object detection bounding box 2d
            is_found = False
            for bbox in result['bbox_list']:
                if bbox[4] != self.object_name:
                    continue
                # if found find the position of the object
                else:
                    # receive xyxy
                    x_pixel = int(bbox[0] + (bbox[2]-bbox[0])/2)
                    y_pixel = int(bbox[1] + (bbox[3]-bbox[1])/2)
                    (x_pixel, y_pixel) = rescale_pixel(x_pixel, y_pixel)
                    object_id = 1 # TODO change to object tracker
                    self.center_pixel_list.append((x_pixel, y_pixel, object_id))
                    # visualize purpose
                    self.frame = cv2.circle(self.frame, (x_pixel, y_pixel), 5, (0, 255, 0), 2)
                    self.frame = cv2.rectangle(self.frame, rescale_pixel(bbox[0], bbox[1]), rescale_pixel(bbox[2], bbox[3]), (0, 255, 0), 2)
            
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(self.frame, "bgr8"))
            if not is_found:
                return 'continue_Navigate_object'

            # 3d pose
            if not self.intrinsics:
                rospy.logerr("no camera intrinsics")
                return None
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
                    self.tf_stamp.header.frame_id = "/camera_link"
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
            self.object_pose = find_closest_object()

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
            object_pose_z_min = None
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
        if count_location == 1:
            self.object_name = "Cereal"
        if count_location == 2:
            self.object_name = "Milk"

        # run_once function
        run_once()
        while not rospy.is_shutdown():
            reset()
            detect()
            userdata.Get_pose_out = self.object_pose
            return 'continue_Pick'




        
class Pick(smach.State):
    def __init__(self):
        rospy.loginfo('Initiating Pick state')
        smach.State.__init__(self, outcomes=['continue_Navigate_table'],
                             input_keys = ['Pick_in'])
    def execute(self, userdata):
        rospy.loginfo('Executing Pick state')
        # recieve the Pose() from Get_pose state 
        object_pose = userdata.Pick_in
        print(object_pose)
        # tell robot's arm to pick the object at specify Pose(
        
        rospy.loginfo('--------------------')
        rospy.loginfo(object_pose)

        def transform_pose(input_pose, from_frame, to_frame):
            # **Assuming /tf2 topic is being broadcasted
            tf_buffer = tf2_ros.Buffer()
            listener = tf2_ros.TransformListener(tf_buffer)
            pose_stamped = tf2_geometry_msgs.PoseStamped()
            pose_stamped.pose = input_pose
            pose_stamped.header.frame_id = from_frame
            pose_stamped.header.stamp = rospy.Time.now()
            try:
                # ** It is important to wait for the listener to start listening. Hence the rospy.Duration(1)
                output_pose_stamped = tf_buffer.transform(
                    pose_stamped, to_frame, rospy.Duration(1))
                return output_pose_stamped.pose
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                raise

        def pick_service(goal_pose, side="front"):
            rospy.wait_for_service('pick_service_select_side')
            try:
                pick = rospy.ServiceProxy(
                    'pick_service_select_side', PickWithSide)
                res = pick(goal_pose, side)
                return res.success_grasp
            except rospy.ServiceException as e:
                print("Service call failed: %s" % e)
                return 'continue_ABORTED'

        transformed_pose = transform_pose(object_pose, "realsense_link", "base_link")
        rospy.loginfo(transformed_pose.position.x)
        transformed_pose.orientation.x = 0
        transformed_pose.orientation.y = 0
        transformed_pose.orientation.z = 0
        transformed_pose.orientation.w = 1

        self.success = pick_service(transformed_pose, 'front')

        return 'continue_Navigate_table'


class Navigate_table(smach.State):
    def __init__(self):
        rospy.loginfo('Initiating Navigate_table state')
        smach.State.__init__(self, outcomes=['continue_GetObjectBBX'])
    def execute(self, userdata):
        rospy.loginfo('Executing Navigate_table state')
        # navigate to the table (can be any location chosen by our team )
        global navigation
        nav_status = navigation.move(FLAT_SURFACE)
        if nav_status:
            return 'continue_GetObjectBBX'
        else:
            return 'continue_GetObjectBBK'

class GetObjectBBX(smach.State):
    def __init__(self):
        rospy.loginfo('Initiating state GetObjectBBX')
        smach.State.__init__(self, outcomes= ['continue_GetProperties'], 
                                output_keys= ['ListBBX_output'])
        # initiate variables
        self.bbxA_list= []
        self.intrinsics = None
        self.bridge = CvBridge()
        self.frame = None
        self.tf_stamp = None

        # connect to CV server
        host = "192.168.8.99"
        port = 10001
        self.c = CustomSocket(host, port)
        self.c.clientConnect()
        rospy.loginfo("connected object detection server")

    def execute(self, userdata):
        rospy.loginfo('Executing state GetObjectBBX')

        def run_once():
            while self.intrinsics is None:
                time.sleep(0.1)
            rospy.loginfo("realsense image width, height = ({}, {})".format(self.intrinsics.width, self.intrinsics.height))
            self.c.req(np.random.randint(255, size=(720, 1280, 3), dtype=np.uint8))

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
            result = self.c.req(self.frame)
            self.frame = check_image_size_for_ros(self.frame)
            rospy.loginfo("result {}".format(result))
            # if result['n'] == 0:
            #     return None

            # object detection bounding box 2d
            for bbox in result['bbox_list']:
                # receive xyxy                
                # (xmin,ymin)----------*
                # |                    |
                # |                    |
                # |                    |
                # |                    |
                # *----------(xmax,ymax)
                # Get List of Bounding Box in Pixel

                object_name = bbox[4]
                x_pixel = int(bbox[0] + (bbox[2]-bbox[0])/2)
                y_pixel = int(bbox[1] + (bbox[3]-bbox[1])/2)

                (xcen_pixel, ycen_pixel) = rescale_pixel(x_pixel, y_pixel)
                (xmin_pixel, ymin_pixel) = rescale_pixel(bbox[0], bbox[1])
                (xmax_pixel, ymax_pixel) = rescale_pixel(bbox[2], bbox[3])

                object_id = 1 # TODO change to object tracker

                # visualize purpose
                self.frame = cv2.circle(self.frame, (xcen_pixel, ycen_pixel), 5, (0, 0, 255), 2)
                self.frame = cv2.rectangle(self.frame, (xmin_pixel, ymin_pixel), (xmax_pixel, ymax_pixel), (0, 0, 255), 2)

                all_bbx_result = pixel_2_meter_BBX(xmin_pixel, xmax_pixel, xcen_pixel,
                                                   ymin_pixel, ymax_pixel, ycen_pixel)
                
                self.bbxA_list.append((all_bbx_result,object_name,object_id))
            
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(self.frame, "bgr8"))

            

            self.image_sub.unregister()
            self.depth_sub.unregister()
            rospy.loginfo("Object found!")
        
        def pixel_2_meter_BBX (xmin_pixel, xmax_pixel, xcen_pixel,
                               ymin_pixel, ymax_pixel, ycen_pixel):
            
            # Using realsense2 lib to convert pixel to real life distance
            # with depth between surface to camera
            #  (xmin,ymin)----------.        corner11-------corner21
            #  |                    |        |                     |
            #  |                    |        |                     |
            #  |                    |        |                     |
            #  |                    |        |                     |
            #  '----------(xmax,ymax)        corner12-------corner22

            # check camera intrinsics
            if not self.intrinsics:
                rospy.logerr("no camera intrinsics")
                return None

            # init each pixel of bouding box
            xmin_pixel = int(xmin_pixel)
            ymin_pixel = int(ymin_pixel)
            xmax_pixel = int(xmax_pixel)
            ymax_pixel = int(ymax_pixel)
            xcen_pixel = int(xcen_pixel)
            ycen_pixel = int(ycen_pixel)

            rospy.loginfo("found {}".format(xcen_pixel,ycen_pixel))
            # rescale pixel incase pixel donot match
            self.depth_image = check_image_size_for_ros(self.depth_image)
            depth = self.depth_image[ycen_pixel,  xcen_pixel] # [y, x] for numpy array

            # [x, y] for realsense lib
            center00_result = rs2.rs2_deproject_pixel_to_point(self.intrinsics, [xcen_pixel, ycen_pixel], depth)
            corner11_result = rs2.rs2_deproject_pixel_to_point(self.intrinsics, [xmin_pixel, ymin_pixel], depth)
            corner12_result = rs2.rs2_deproject_pixel_to_point(self.intrinsics, [xmin_pixel, ymax_pixel], depth)
            corner21_result = rs2.rs2_deproject_pixel_to_point(self.intrinsics, [xmax_pixel, ymin_pixel], depth)
            corner22_result = rs2.rs2_deproject_pixel_to_point(self.intrinsics, [xmax_pixel, ymax_pixel], depth)
            
            all_result = (center00_result, corner11_result, corner12_result, corner21_result, corner22_result)

            return all_result

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
            try:
                # if self.tf_stamp is not None:
                    # rospy.loginfo("publishing tf")
                    # self.tf_stamp.header.stamp = rospy.Time.now()
                    # self.pub_tf.publish(tf2_msgs.msg.TFMessage([self.tf_stamp]))

                self.depth_image = self.bridge.imgmsg_to_cv2(frame, frame.encoding)
                

            except CvBridgeError as e:
                print(e)
                return
            except ValueError as e:
                return
            pass

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

        # run_once function
        run_once()
        if not rospy.is_shutdown():
            reset()
            detect()
            userdata.ListBBX_output = self.bbxA_list
        return 'continue_GetProperties'
        

class GetObjectProperties(smach.State):
    def __init__(self):
        rospy.loginfo('Initiating state GetObjectProperties')
        smach.State.__init__(self, outcomes=['continue_Place_object'], 
                             input_keys=['ListBBX_input'], 
                             output_keys=['ObjectPoseList_output', 'ObjectSizeList_output'])
        
        # initiate variables from GetObjectBBX()
        # self.bbx_all_list :
        # [((center00, corner11, corner12, corner21, corner22), class, id), ...]
        # which corner is (x,y,z) in meter
        self.bbx_all_list = []

        # init all object properties
        self.bbxc_point_list = []

        self.ObjPoseList = []
        self.ObjSizeList = []
        self.ObjStanList = []
    
    def execute(self, userdata):
        rospy.loginfo('Executing state GetObjectProperties')

        def GetPose(bbxc_point_list):
            # bbxc_point_list = [((center00, corner11, corner12, corner21, corner22), objname),...] as tuple of point, str
            # get pose from center of bounding box and using direction as diagram below:
            #              Z
            #         X    |
            #          \   |
            #           \  |
            #            \ |
            #  Y-----------o

            objpose_list = []
            for bbxc in bbxc_point_list:
                center00 = bbxc[0][0]

                center_pose = Pose()
                center_pose.position.x = center00.x
                center_pose.position.y = center00.y
                center_pose.position.z = center00.z

                center_pose.orientation.w = 1
                center_pose.orientation.x = 0
                center_pose.orientation.y = 0
                center_pose.orientation.z = 0

                objpose_list.append(center_pose)

            return objpose_list

        def GetSize(bbxc_point_list):
            # bbxc_point_list = [((center00, corner11, corner12, corner21, corner22), objname),...] as tuple of point, str
            # corner11-------corner21      y:= length z:= hight x:= depth
            # |                     |
            # |                     |                Save in size as Vector3:         
            # |                     |       z           size.x = depth (from camara to center of object surface)    
            # |                     |       |           size.y = length(of bounding box )
            # corner12-------corner22   y--*x(in)       size.z = hight (of bounding box )

            objsize_list = []
            for bbxc in bbxc_point_list:
                corner11 = bbxc[0][1]
                corner12 = bbxc[0][2]
                corner21 = bbxc[0][3]
                corner22 = bbxc[0][4]

                bbx_size = Vector3()
                bbx_size.x = corner22.x
                bbx_size.y = abs(corner21.y - corner11.y)
                bbx_size.z = abs(corner11.z - corner12.z)

                objsize_list.append(bbx_size)

            return objsize_list
                
        # ----------------------------------------------start-----------------------------------------------------
        # init data from input
        self.bbx_all_list = userdata.ListBBX_input

        # start runing bounding box in list
        for bbxA in self.bbx_all_list:
            # (xmin,ymin)----------.        corner11-------corner21
            # |                    |        |                     |
            # |                    |        |                     |
            # |                    |        |                     |
            # |                    |        |                     |
            # '----------(xmax,ymax)        corner12-------corner22
            # self.bbx_all_list: ((center00, corner11, corner12, corner21, corner22), class, id)
            all_result = bbxA[0]
            obj_name = bbxA[1]
            obj_id = bbxA[2]

            # depth of camera to plane using point center00
            z = all_result[0][2]/1000

            # set list of point
            checkpoint = []

            # set pose that related with coordinate as: 
            #  x =  z_coord
            #  y = -x_coord
            #  z = -y_coord
            # 
            # +Z           
            # |         +y   realsense frame
            # |         | +z   o object
            # |         |/
            # |  +X     o---> +x
            # | / 
            # 0/-----------------> -Y camera_link frame tf

            for result in all_result:
                x = result[0]/1000
                y = result[1]/1000

                point = Point()
                point.x =  z
                point.y = -x
                point.z = -y

                checkpoint.append(point)
            tuple(checkpoint)

            self.bbxc_point_list.append((checkpoint, obj_name, obj_id))
        
        self.ObjPoseList = GetPose(self.bbxc_point_list)
        self.ObjSizeList = GetSize(self.bbxc_point_list)

        rospy.loginfo("Get all Object Properties, sending to Place")
        userdata.ObjectPoseList_output = self.ObjPoseList
        userdata.ObjectSizeList_output = self.ObjSizeList

        return 'continue_Place_object'
        
class Place_object(smach.State):
    def __init__(self):
        rospy.loginfo('Initiating Place_object state')
        smach.State.__init__(self, outcomes=['continue_Navigate_object',
                                             'continue_SUCCEEDED'],
                                    input_keys = ['object_pose_list_input'])
    def execute(self, userdata):
        global count_placeObject,PLACE_TABLE
        rospy.loginfo('Executing Place_object state')
        count_placeObject += 1
        rospy.loginfo('Number of object placed = %s', count_placeObject)
        def transform_pose(input_pose, from_frame, to_frame):

            # **Assuming /tf2 topic is being broadcasted
            tf_buffer = tf2_ros.Buffer()
            listener = tf2_ros.TransformListener(tf_buffer)

            pose_stamped = tf2_geometry_msgs.PoseStamped()
            pose_stamped.pose = input_pose
            pose_stamped.header.frame_id = from_frame
            # pose_stamped.header.stamp = rospy.Time.now()

            try:
                # ** It is important to wait for the listener to start listening. Hence the rospy.Duration(1)
                output_pose_stamped = tf_buffer.transform(pose_stamped, to_frame, rospy.Duration(1))
                return output_pose_stamped.pose

            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                raise

        def lift_cb(data) :
            while (not data.data) : pass

        def lift_command() :
            lift_pub = rospy.Publisher('lift_command', Bool, queue_size=1)
            time.sleep(1)
            lift_pub.publish(True)

            rospy.Subscriber("done", Bool, lift_cb)

        def place_service(corner11, corner12, corner21, corner22, high, current_collision_object_pos):
            rospy.wait_for_service('cr3_place')
            try:
                place = rospy.ServiceProxy('cr3_place', cr3_place)
                res = place(corner11, corner12, corner21, corner22, high, current_collision_object_pos)
                return res.success_place
            except rospy.ServiceException as e:
                print("Service call failed: %s"%e)

        # ed = EnvironmentDescriptor("../config/fur_data.yaml")
        # corner11 = ed.get_pose_environment("....")
        

        #collision object
        corner1 = Pose()
        corner2 = Pose()
        corner3 = Pose()
        corner4 = Pose()
        high = Point()
        robot_pose = Pose()

        ed = EnvironmentDescriptor("../config/fur_data.yaml")
        corner1.position, corner2.position, corner3.position, corner4.position = ed.get_corner_list(PLACE_TABLE)
        high.z = ed.get_height(PLACE_TABLE)

        robot_pose = transform_pose(Pose(), "world", "robot_pose")

        robot_pose.position.x *= -1
        robot_pose.position.y *= -1
        robot_pose.position.z *= -1

        # rospy.loginfo(robot_pose)

        corner1 = transform_pose(corner1, "world", "robot_pose")
        corner2 = transform_pose(corner2, "world", "robot_pose")
        corner3 = transform_pose(corner3, "world", "robot_pose")
        corner4 = transform_pose(corner4, "world", "robot_pose")

        corner_x = sorted([corner1.position.x, corner2.position.x, corner3.position.x, corner4.position.x])
        corner_y = sorted([corner1.position.y, corner2.position.y, corner3.position.y, corner4.position.y])

        corner11_pose = Point()
        corner12_pose = Point()
        corner21_pose = Point()
        corner22_pose = Point()

        corner11_pose.x = corner_x[0]
        corner12_pose.x = corner_x[1]
        corner21_pose.x = corner_x[2]
        corner22_pose.x = corner_x[3]

        corner11_pose.y = corner_y[0] 
        corner12_pose.y = corner_y[2]
        corner21_pose.y = corner_y[1]
        corner22_pose.y = corner_y[3]

        high.z -= 0.49      #lift state is False
        if (0.25 < high.z) : 
            lift_command()
            high.z -= 0.20  #lift state is True


        # rospy.loginfo("object pose list before tf")

        # rospy.loginfo(userdata.object_pose_list_input[0])
        collision_object_pose = []
        rospy.loginfo("collision_object_pose")
        for object_pose in userdata.object_pose_list_input :
            rospy.loginfo(transform_pose(object_pose, "real_sense_on_sim", "base_link"))
            rospy.loginfo(object_pose)
            collision_object_pose.append(transform_pose(object_pose, "real_sense_on_sim", "base_link"))
        # rospy.loginfo("object pose list after tf")
        # rospy.loginfo(collision_object_pose)
        
        # rospy.loginfo(corner11_pose, corner12_pose, corner21_pose, corner22_pose, collision_object_pose)
        success = place_service(corner11_pose, corner12_pose, corner21_pose, corner22_pose, high, collision_object_pose)
        # success = True
        print(success)

        # success = True

        if (success) : return 'continue_succeeded'
        else : return 'continue_aborted'

        if count_placeObject == 2:
            return 'continue_SUCCEEDED'
        else:
            return 'continue_Navigate_object'


if __name__ == '__main__':
    #content in the main function bruh bruh bruh
    rospy.init_node('serving_breakfast')

    ###############################
    # cereal is on table
    CEREAL_FUR = "table1"
    MILK_FUR = "bed"
    FLAT_SURFACE = "flat_surface"
    PLACE_TABLE = "table1"
    ###############################

    #declare the global variable
    count_object = 0
    count_placeObject = 0
    count_location = 0

    image_pub = rospy.Publisher("/blob/image_blob", Image, queue_size=1)
    navigation = go_to_Navigation()

    rs2 = Realsense()
    rs2.wait() # wait for camera intrinsics

    
    sm_top = smach.StateMachine(outcomes=['SUCCEEDED', 'ABORTED'])
    # initial userdata Pose()
    sm_top.userdata.sm_pose = Pose()
    # userdata from GetObjectBBX
    sm_top.userdata.bbx_list = []
    # userdata from GetObjectProperties
    sm_top.userdata.object_pose_list = []
    sm_top.userdata.object_size_list = []

    ed = EnvironmentDescriptor("../config/fur_data.yaml")
    
    with sm_top:
        smach.StateMachine.add('Start_signal', Start_signal(),
                               transitions={'continue_Navigate_object':'Navigate_object'})
        smach.StateMachine.add('Navigate_object', Navigate_object(),
                               transitions={'continue_Get_pose':'Get_pose',
                                            'continue_ABORTED':'ABORTED'})
        smach.StateMachine.add('Get_pose', Get_pose(),
                               transitions={'continue_Pick':'Pick','continue_Navigate_object':'Navigate_object'},
                               remapping={'Get_pose_in':'sm_pose',
                                          'Get_pose_out':'sm_pose'})
        smach.StateMachine.add('Pick', Pick(),
                               transitions={'continue_Navigate_table':'Navigate_table'},
                               remapping={'Pick_in':'sm_pose'})
        smach.StateMachine.add('Navigate_table', Navigate_table(),
                               transitions={'continue_GetObjectBBX':'GetObjectBBX'})
        smach.StateMachine.add('GetObjectBBX', GetObjectBBX(), 
                                transitions={'continue_GetObjectProperties':'GetObjectProperties'},
                                remapping=   {'ListBBX_output': 'bbx_list'})
        smach.StateMachine.add('GetObjectProperties', GetObjectProperties(),
                                transitions={'continue_Place_object':'Place_object'},
                                remapping=   {'ListBBX_input'           : 'bbx_list',
                                              'ObjectPoseList_output'   : 'object_pose_list',
                                              'ObjectSizeList_output'   : 'object_size_list'})
        smach.StateMachine.add('Place_object', Place_object(),
                               transitions={'continue_Navigate_object':'Navigate_object',
                                            'continue_SUCCEEDED':'SUCCEEDED'},
                                            remapping = {'object_pose_list_input':'object_pose_list'})

        sis = smach_ros.IntrospectionServer('Service_name', sm_top, '/ServingBreakfast')
        sis.start()

        outcomes = sm_top.execute()

        rospy.spin()
        sis.stop()