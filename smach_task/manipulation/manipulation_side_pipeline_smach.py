#!/usr/bin/env python

"""
roslaunch realsense2_camera rs_rgbd.launch align_depth:=true color_width:=1280 color_height:=720 color_fps:=30 depth_width:=1280 depth_height:=720 depth_fps:=30 filters:=pointcloud
kill -9 $(lsof -t -i:10001)
roslaunch cr3_moveit_control manipulation_pipeline_real.launch
"""

import rospy
import smach
import smach_ros
import time

from cr3_moveit_control.srv import PickWithSide
# msg for transfer information 
from geometry_msgs.msg import Pose, Point, Vector3

# # tf for visualization
import tf
from geometry_msgs.msg import TransformStamped

# transform to from cameralink to base link
import tf2_msgs
import tf2_ros
import tf2_geometry_msgs

# realsense
import pyrealsense2 as rs2
import cv2
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError

# # computer vision
# import socket
from custom_socket import CustomSocket
import numpy as np

class GetObjectName(smach.State):
    def __init__(self):
        rospy.loginfo('Initiating state GetObjectName')
        smach.State.__init__(self, outcomes=['continue_GetObjectBBX'], output_keys=[
                             'objectname_output'])

    def execute(self, userdata):
        rospy.loginfo('Executing state GetObjectName')
        # sending object name to GetobjectName state (change string right here)
        userdata.objectname_output = "Cereal"
        return 'continue_GetObjectBBX'

class GetObjectBBX(smach.State):
    def __init__(self):
        rospy.loginfo('Initiating state GetObjectBBX')
        smach.State.__init__(self, outcomes= ['continue_GetProperties', 'continue_ABORTED'], 
                                input_keys= ['objectname_input'],
                                output_keys= ['ListBBX_output'])
        # initiate variables
        self.object_name = ""
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
            if result['n'] == 0:
                return False

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
                if object_name != self.object_name:
                    continue
                else:
                    x_pixel = int(bbox[0] + (bbox[2]-bbox[0])/2)
                    y_pixel = int(bbox[1] + (bbox[3]-bbox[1])/2)

                    (xcen_pixel, ycen_pixel) = rescale_pixel(x_pixel, y_pixel)
                    (xmin_pixel, ymin_pixel) = rescale_pixel(bbox[0], bbox[1])
                    (xmax_pixel, ymax_pixel) = rescale_pixel(bbox[2], bbox[3])

                    object_id = 1 # TODO change to object tracker

                    # visualize purpose
                    self.frame = cv2.circle(self.frame, (xcen_pixel, ycen_pixel), 5, (0, 255, 0), 2)
                    self.frame = cv2.rectangle(self.frame, (xmin_pixel, ymin_pixel), (xmax_pixel, ymax_pixel), (0, 255, 0), 2)

                    all_bbx_result = pixel_2_meter_BBX(xmin_pixel, xmax_pixel, xcen_pixel,
                                                        ymin_pixel, ymax_pixel, ycen_pixel)
                    
                    self.bbxA_list.append((all_bbx_result,object_name,object_id))
            
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(self.frame, "bgr8"))

            

            self.image_sub.unregister()
            self.depth_sub.unregister()
            rospy.loginfo("Object found!")
            return True
        
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

        # recieving object name from GetObjectName state
        self.object_name = userdata.objectname_input
        rospy.loginfo(self.object_name)

        # run_once function
        run_once()
        while not rospy.is_shutdown():
            command = raw_input("Press Enter :")
            if command == 'q':
                break
            rospy.loginfo("------ Running 3D detection ------")
            reset()
            not_abort = detect()
            if not_abort == True:
                userdata.ListBBX_output = self.bbxA_list
                return 'continue_GetProperties'
            else:
                return 'continue_ABORTED'
        return 'continue_ABORTED'


class GetObjectProperties(smach.State):
    def __init__(self):
        rospy.loginfo('Initiating state GetObjectProperties')
        smach.State.__init__(self, outcomes=['continue_Pick','continue_ABORTED'], 
                             input_keys=['ListBBX_input'], 
                             output_keys=['ObjectStan_output','ObjectPose_output'])
        
        # initiate variables from GetObjectBBX()
        # self.bbx_all_list :
        # [((center00, corner11, corner12, corner21, corner22), class, id), ...]
        # which corner is (x,y,z) in meter
        self.bbx_all_list = []

        # init all object properties
        self.bbxc_point_list = []

        self.ObjPoseList = []
        self.ObjClosest = int()
        self.ObjSize = Vector3()
        self.ObjStanLi = ''
    
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

        def GetSize(bbxc_point):
            # bbxc_point_list = [((center00, corner11, corner12, corner21, corner22), objname),...] as tuple of point, str
            # corner11-------corner21      y:= length z:= hight x:= depth
            # |                     |
            # |                     |                Save in size as Vector3:         
            # |                     |       z           size.x = depth (from camara to center of object surface)    
            # |                     |       |           size.y = length(of bounding box )
            # corner12-------corner22   y--*x(in)       size.z = hight (of bounding box )

            
            bbxc = bbxc_point
            print (bbxc)
            corner11 = bbxc[0][1]
            corner12 = bbxc[0][2]
            corner21 = bbxc[0][3]
            corner22 = bbxc[0][4]

            bbx_size = Vector3()
            bbx_size.x = corner22.x
            bbx_size.y = abs(corner21.y - corner11.y)
            bbx_size.z = abs(corner11.z - corner12.z)

            return bbx_size
           
        def GetStan(objsize):
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

            posture = ("parallel","perpendicular","nosize")
            
            print(objsize.y)
            print(objsize.z)
            if objsize.z == 0:
                return posture[2]
            else:
                if objsize.y/objsize.z >= Rfactor:
                    return posture[0]
                else:
                    return posture[1]
            

        def find_closest_object(object_pose_list):
            idx = 0
            min_idx = 0
            z_min = 10000000000
            print(object_pose_list)
            for object_pose in object_pose_list:
                if 0.5 <= object_pose.position.z < z_min:
                    min_idx = idx
                idx += 1

            # for obj_idx in len(object_pose_list):
            #     print(obj_idx)
            #     print(object_pose_list[obj_idx])
            #     print(object_pose_list[obj_idx].position.z)
            #     if 0.5 <= object_pose_list[obj_idx].position.z < z_min:
            #         min_pose = object_pose_list[obj_idx]
  
            try:
                return min_idx
            except:
                return None

        # ----------------------------------------------start-----------------------------------------------------
        self.pub_tf = rospy.Publisher(
            "/tf", tf2_msgs.msg.TFMessage, queue_size=1)
        
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
        
        if len(self.bbxc_point_list) == 0:
            return 'continue_ABORTED'
        else:
            print(self.bbxc_point_list)
            self.ObjPoseList = GetPose(self.bbxc_point_list)
            print(self.ObjPoseList)
            self.ObjClosest  = find_closest_object(self.ObjPoseList)
            if self.ObjClosest is None:
                return 'continue_ABORTED'
            self.ObjSize = GetSize(self.bbxc_point_list[self.ObjClosest])
            self.ObjStan = GetStan(self.ObjSize)

            objpose = self.ObjPoseList[self.ObjClosest]

            self.tf_stamp = TransformStamped()
            self.tf_stamp.header.frame_id = "/camera_link"
            self.tf_stamp.header.stamp = rospy.Time.now()
            self.tf_stamp.child_frame_id = "/object_frame"
            # self.tf_stamp.child_frame_id = "/object_frame_{}".format(self.bbxc_point_list[self.ObjClosest[1]][1]) # object_id
            self.tf_stamp.transform.translation.x = objpose.position.x
            self.tf_stamp.transform.translation.y = objpose.position.y
            self.tf_stamp.transform.translation.z = objpose.position.z
            self.tf_stamp.transform.rotation.x = objpose.orientation.x
            self.tf_stamp.transform.rotation.y = objpose.orientation.y
            self.tf_stamp.transform.rotation.z = objpose.orientation.z
            self.tf_stamp.transform.rotation.w = objpose.orientation.w

            self.pub_tf.publish(tf2_msgs.msg.TFMessage([self.tf_stamp]))

            rospy.loginfo("Get all Object Properties, sending to Place")
            userdata.ObjectPose_output = objpose
            userdata.ObjectStan_output = self.ObjStan
            print(self.ObjStan)
            print(objpose)

            return 'continue_Pick'

class Pick(smach.State):
    def __init__(self):
        rospy.loginfo('Initiating state Pick')
        smach.State.__init__(self, outcomes=['continue_SUCCEEDED', 'continue_ABORTED'], 
                                   input_keys=['objectpose_input','objectstan_input'])
        self.success = False

    def execute(self, userdata):
        rospy.loginfo('Executing state Pick')
        # recieving Pose() from GetObjectPose state
        recieved_pose = userdata.objectpose_input
        rospy.loginfo('------ Position ------')
        rospy.loginfo('x = %s', recieved_pose.position.x)
        rospy.loginfo('x = %s', recieved_pose.position.x)
        rospy.loginfo('y = %s', recieved_pose.position.y)
        rospy.loginfo('z = %s', recieved_pose.position.z)
        rospy.loginfo('------ Orientation ------')
        rospy.loginfo('x = %s', recieved_pose.orientation.x)
        rospy.loginfo('y = %s', recieved_pose.orientation.y)
        rospy.loginfo('z = %s', recieved_pose.orientation.z)
        rospy.loginfo('w = %s', recieved_pose.orientation.w)

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

        transformed_pose = transform_pose(
            recieved_pose, "camera_link", "base_link")
        rospy.loginfo(transformed_pose.position.x)
        transformed_pose.orientation.x = 0
        transformed_pose.orientation.y = 0
        transformed_pose.orientation.z = 0
        transformed_pose.orientation.w = 1

        self.stance = userdata.objectstan_input
        if self.stance == 'parallel':
            self.success = pick_service(transformed_pose, 'side')
        else:
            self.success = pick_service(transformed_pose, 'front')

        

        if self.success == True:
            return 'continue_SUCCEEDED'
        else:
            return 'continue_ABORTED'


def main():
    rospy.init_node('smach_object_detection_state_machine')
    sm = smach.StateMachine(outcomes=['SUCCEEDED', 'ABORTED'])

    sm.userdata.string_name = ""

    # userdata from GetObjectBBX
    sm.userdata.bbx_list = []

    # userdata from GetObjectProperties
    sm.userdata.object_pose = Pose()
    sm.userdata.object_stan = ''

    with sm:
        smach.StateMachine.add('GetObjectName', GetObjectName(),
                               transitions={
                                   'continue_GetObjectBBX': 'GetObjectBBX'},
                               remapping={'objectname_output': 'string_name'})

        smach.StateMachine.add('GetObjectBBX', GetObjectBBX(),
                                transitions= {'continue_GetProperties'  : 'GetObjectProperties',
                                              'continue_ABORTED'        : 'ABORTED'},
                                remapping=   {'objectname_input': 'string_name',
                                                'ListBBX_output'          : 'bbx_list'})

        smach.StateMachine.add('GetObjectProperties', GetObjectProperties(),
                                transitions= {'continue_Pick'          : 'Pick',
                                                'continue_ABORTED'        : 'ABORTED'},
                                remapping=   {'ListBBX_input'           : 'bbx_list',
                                              'ObjectPose_output'   : 'object_pose',
                                              'ObjectStan_output'   : 'object_stan'})
        
        smach.StateMachine.add('Pick', Pick(),
                               transitions={'continue_SUCCEEDED': 'SUCCEEDED',
                                            'continue_ABORTED': 'ABORTED'},
                               remapping={'objectpose_input': 'object_pose',
                                            'objectstan_input': 'object_stan'})

    # outcome = sm.execute()

    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('server_name', sm, '/buhbubu')
    sis.start()

    # Execute the state machine
    outcome = sm.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()
    return


if __name__ == "__main__":
    main()