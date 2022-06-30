#!/usr/bin/env python

"""
roslaunch cr3_moveit_control manipulation_pipeline_real.launch
"""

import rospy
import smach
import smach_ros

# msg for transfer information 
from geometry_msgs.msg import Pose, Point, Vector3



from tf.transformations import quaternion_from_euler
import time

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
from custom_socket import CustomSocket
import numpy as np

# transform to from cameralink to base link
import tf2_ros
import tf2_geometry_msgs

class GetObjectBBX(smach.State):
    def __init__(self):
        rospy.loginfo('Initiating state GetObjectBBX')
        smach.State.__init__(self, outcomes=['continue_GetProperties', 'continue_ABORTED'], 
                                output_keys=['ListBBX_output', 'CameraIntrinsics_output', 'DepthImage_output'])
        # initiate variables
        self.bbx_pixel_list = [] # [(xm1, ym1, xM1, yM1, class), (xm2, ym2, xM2, yM2, class), ...] in pixels
        self.intrinsics = None
        self.bridge = CvBridge()
        self.frame = None

        # connect to CV server
        host = "192.168.8.99"
        port = 10001
        self.c = CustomSocket(host, port)
        self.c.clientConnect()
        rospy.loginfo("connected object detection server")

    def execute(self, userdata):
        rospy.loginfo('Executing state GetObjectPose')

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
            # send frame to server and recieve the result
            result = self.c.req(self.frame)
            self.frame = check_image_size_for_ros(self.frame)
            rospy.loginfo("result {}".format(result))
            if result['n'] == 0:
                return None 
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

                (xmin_pixel,ymin_pixel) = rescale_pixel(bbox[0], bbox[1])
                (xmax_pixel,ymax_pixel) = rescale_pixel(bbox[2], bbox[3])
                object_id = 1 # TODO change to object tracker

                self.bbx_pixel_list.append((xmin_pixel,ymin_pixel,xmax_pixel,ymax_pixel, object_id))

                # visualize purpose
                x_pixel = int(bbox[0] + (bbox[2]-bbox[0])/2)
                y_pixel = int(bbox[1] + (bbox[3]-bbox[1])/2)
                (x_pixel, y_pixel) = rescale_pixel(x_pixel, y_pixel)
                
                self.frame = cv2.circle(self.frame, (x_pixel, y_pixel), 5, (0, 255, 0), 2)
                self.frame = cv2.rectangle(self.frame, (xmin_pixel,ymin_pixel), (xmax_pixel,ymax_pixel), (0, 0, 255), 2)
                                    
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(self.frame, "bgr8"))

            if not self.intrinsics:
                rospy.logerr("no camera intrinsics")
                return None

            self.image_sub.unregister()
            self.depth_sub.unregister()

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
                # scale image incase image size donot match cv server
                self.frame = check_image_size_for_cv(self.frame)
            except CvBridgeError as e:
                print(e)

        def depth_callback(frame):
            try:
                # if self.tf_stamp is not None:
                #     # rospy.loginfo("publishing tf")
                #     self.tf_stamp.header.stamp = rospy.Time.now()
                #     self.pub_tf.publish(tf2_msgs.msg.TFMessage([self.tf_stamp]))

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

        # run_once function
        run_once()
        while not rospy.is_shutdown():
            command = raw_input("Press Enter :")
            if command == 'q':
                break
            rospy.loginfo("------ Running 3D detection ------")
            reset()
            detect()
            userdata.ListBBX_output = self.bbx_pixel_list
            userdata.CameraIntrinsics_output = self.intrinsics
            userdata.DepthImage_output = self.depth_image
            return 'continue_GetProperties'
        return 'continue_ABORTED'

class GetObjectProperties(smach.State):
    def __init__(self):
        rospy.loginfo('Initiating state GetObjectProperties')
        smach.State.__init__(self, outcomes=['continue_Place'], 
                             input_keys=['ListBBX_input', 'CameraIntrinsics_input', 'DepthImage_input'], 
                             output_keys=['ObjectPoseList_output', 'ObjectSizeList_output', 'ObjectStanList_output'])
        
        # initiate variables from GetObjectBBX()
        self.bbx_pixel_list = [] # [(xm1, ym1, xM1, yM1, class), (xm2, ym2, xM2, yM2, class), ...] in pixels
        self.intrinsics = None
        self.depth_image = None

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
           
        def GetStan(objsize_list):
            # objsize_list = [size1, size2, ...]
            # using Rfactor to compare posture of object using length/hight
            # if l/h >= Rfactor the object will have more probability to be parallel to camera
            # else the object will have more probability to be perpendicular to camera
            #
            #           parallel                     perpendicular 
            #         ------------ obj             ||||||||||||||||| obj 
            #         ============ cam             ================= cam
            #
            # this Rfactor = 0.6 is using for cerial box only, get from experimental

            Rfactor = 0.6

            posture = ("parallel","perpendicular")
            print(objsize.y)
            print(objsize.z)

            objstance_list = []
            for objsize in objsize_list:
                if objsize.y/objsize.z >= Rfactor:
                    objstance_list.append(posture[0])
                else:
                    objstance_list.append(posture[1])
            
            return objstance_list
                
        # ----------------------------------------------start-----------------------------------------------------
        # init data from input
        self.bbx_pixel_list = userdata.ListBBX_input
        self.intrinsics     = userdata.CameraIntrinsics_input
        self.depth_image    = userdata.DepthImage_input

        # start runing bounding box in list
        for bbx in self.bbx_pixel_list:
            # init each pixel of bouding box
            # (xmin,ymin)----------.        corner11-------corner21
            # |                    |        |                     |
            # |                    |        |                     |
            # |                    |        |                     |
            # |                    |        |                     |
            # '----------(xmax,ymax)        corner12-------corner22

            objname = bbx[4]
            xmin_pixel = int(bbx[0])
            ymin_pixel = int(bbx[1])
            xmax_pixel = int(bbx[2])
            ymax_pixel = int(bbx[3])
            xcen_pixel = int(bbx[0] + (bbx[2]-bbx[0])/2)
            ycen_pixel = int(bbx[1] + (bbx[3]-bbx[1])/2)

            rospy.loginfo("found {}".format(xcen_pixel,ycen_pixel))
            depth = self.depth_image[ycen_pixel,  xcen_pixel] # [y, x] for numpy array

            # [x, y] for realsense lib
            center00_result = rs2.rs2_deproject_pixel_to_point(self.intrinsics, [xcen_pixel, ycen_pixel], depth)
            corner11_result = rs2.rs2_deproject_pixel_to_point(self.intrinsics, [xmin_pixel, ymin_pixel], depth)
            corner12_result = rs2.rs2_deproject_pixel_to_point(self.intrinsics, [xmin_pixel, ymax_pixel], depth)
            corner21_result = rs2.rs2_deproject_pixel_to_point(self.intrinsics, [xmax_pixel, ymin_pixel], depth)
            corner22_result = rs2.rs2_deproject_pixel_to_point(self.intrinsics, [xmax_pixel, ymax_pixel], depth)

            all_result = (center00_result, corner11_result, corner12_result, corner21_result, corner22_result)

            # depth of camera to plane
            z = center00_result[2]/1000

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

            self.bbxc_point_list.append((checkpoint,objname))
        
        self.ObjPoseList = GetPose(self.bbxc_point_list)
        self.ObjSizeList = GetSize(self.bbxc_point_list)
        self.ObjStanList = GetStan(self.ObjSizeList)

        rospy.loginfo("Get all Object Properties, sending to Place")
        userdata.ObjectPoseList_output = self.ObjPoseList
        userdata.ObjectSizeList_output = self.ObjSizeList
        userdata.ObjectStanList_output = self.ObjStanList


        return 'continue_Place'

class Place(smach.State):
    def __init__(self):
        rospy.loginfo('Initiating state GetObjectProperties')
        smach.State.__init__(self, outcomes=['continue_SUCCEEDED'],
                             input_keys=['ObjectPoseList_input', 'ObjectSizeList_input', 'ObjectStanList_input']) 
    
    def execute(self, userdata):
        rospy.loginfo(userdata.ObjectPoseList_input)
        rospy.loginfo(userdata.ObjectSizeList_input)
        rospy.loginfo(userdata.ObjectStanList_input)
        return 'continue_SUCCEEDED'

def main():
    rospy.init_node('smach_pick_state_machine')
    sm = smach.StateMachine(outcomes=['SUCCEEDED', 'ABORTED'])

    # userdata from GetObjectBBX
    sm.userdata.intrinsics = None
    sm.userdata.depth_image = None
    sm.userdata.bbx_list = []

    # userdata from GetObjectProperties
    sm.userdata.object_pose_list = []
    sm.userdata.object_size_list = []
    sm.userdata.object_stan_list = []

    with sm:
        smach.StateMachine.add('GetObjectBBX', GetObjectBBX(),
                                transitions= {'continue_GetProperties'  : 'GetObjectProperties',
                                              'continue_ABORTED'        : 'ABORTED'},
                                remapping=   {'ListBBX_output'          : 'bbx_list',
                                              'CameraIntrinsics_output' : 'intrinsics',
                                              'DepthImage_output'       : 'depth_image'})

        smach.StateMachine.add('GetObjectProperties', GetObjectProperties(),
                                transitions= {'continue_Place'          : 'Place'},
                                remapping=   {'ListBBX_input'           : 'bbx_list',
                                              'CameraIntrinsics_input'  : 'intrinsics',
                                              'DepthImage_input'        : 'depth_image',
                                              'ObjectPoseList_output'   : 'object_pose_list',
                                              'ObjectSizeList_output'   : 'object_size_list',
                                              'ObjectStanList_output'   : 'object_stan_list'})
        
        smach.StateMachine.add('Place', Place(),
                                transitions= {'continue_SUCCEEDED'      : 'SUCCEEDED'},
                                remapping=   {'ObjectPoseList_input'    : 'object_pose_list',
                                              'ObjectSizeList_input'    : 'object_size_list',
                                              'ObjectStanList_input'    : 'object_stan_list'})
    # outcome = sm.execute()

    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('server_name', sm, '/Place')
    sis.start()

    # Execute the state machine
    outcome = sm.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()


if __name__ == "__main__":
    main()