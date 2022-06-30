import roslib
import rospy
import smach
import smach_ros
from geometry_msgs.msg import Pose, Point, TransformStamped
import tf
from tf.transformations import quaternion_from_euler
import math
import time

# realsense
import pyrealsense2 as rs2
import cv2
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError

# # tf for visualization
import tf2_msgs

# # computer vision
# import socket
from custom_socket import CustomSocket
import numpy as np

# transform to from cameralink to base link
import tf2_ros
import tf2_geometry_msgs

from visualization_msgs.msg import Marker , MarkerArray

from cr3_moveit_control.srv import cr3_place

# /home/tanas/smach/walkie2_ws/src/cr3_moveit_control/srv/cr3_place.srv
class Input(smach.State):
    def __init__(self):
        rospy.loginfo('initiating input state')
        smach.State.__init__(self, outcomes = ['continue_GetObjectPoseList'])

    def execute(self, userdata):
        return 'continue_GetObjectPoseList'

class GetObjectPoseList(smach.State):
    def __init__(self):
        rospy.loginfo('initiating get object pose list state')
        smach.State.__init__(self, outcomes=['continue_Place', 'continue_aborted'])
    
    def execute(self, userdata):
        return 'continue_Place'

class Place(smach.State):
    def __init__(self) :
        rospy.loginfo('initiating place state')
        smach.State.__init__(self, outcomes = ['continue_aborted','continue_succeeded'])

    def execute(self, userdata):

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

        raw_input("Press enter")

        point_goal11 = Point()
        point_goal12 = Point()
        point_goal21 = Point()
        point_goal22 = Point()

        point_goal11.x = -0.99
        point_goal12.x = -0.99
        point_goal21.x = -0.35
        point_goal22.x = -0.35

        point_goal11.y = -0.45
        point_goal12.y = 0.42
        point_goal21.y = -0.45
        point_goal22.y = 0.42

        # high = abs(table - base_link)
        high = Point()
        high.z = 0.20


        c1 = Pose()
        c2 = Pose()
        c3 = Pose()

        c1.position.x = -0.45
        c1.position.y = -0.30

        c2.position.x = -0.45
        c2.position.y = -0.15

        collision_object_pos = [c1, c2]

        # rospy.loginfo(point_goal11, point_goal12, point_goal21, point_goal22, high, collision_object_pos)
        success = place_service(point_goal11, point_goal12, point_goal21, point_goal22, high, collision_object_pos)
        print(success)

        return 'continue_succeeded'


def main():
    
    rospy.init_node('smach_sm_place_state_machine')

    sm = smach.StateMachine(outcomes = ['succeeded', 'aborted'])

    with sm:
        smach.StateMachine.add('INPUT', Input(),
                                transitions = {'continue_GetObjectPoseList':'GETOBJECTPOSELIST'})
        
        smach.StateMachine.add('GETOBJECTPOSELIST', GetObjectPoseList(),
                                transitions = {'continue_Place':'PLACE',
                                               'continue_aborted':'aborted'})

        smach.StateMachine.add('PLACE', Place(),
                                transitions = {'continue_aborted':'aborted',
                                                'continue_succeeded':'succeeded'})

    sis = smach_ros.IntrospectionServer('sm_place_server', sm, '/PLACE')
    sis.start()
    
    outcome = sm.execute()

    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()


# class GetObjectPoseList(smach.State):
#     def __init__(self):
#         rospy.loginfo('initiating get object pose list state')
#         smach.State.__init__(self, outcomes=['continue_Place', 'continue_aborted']
#                                  , input_keys=['objectname_input', 'objectpose_output']
#                                  , output_keys=['objectlist_output'])
#         # initiate variables
#         # self.object_name = ""
#         self.center_pixel_list = [] # [(x1, y1, id), (x2, y2, id), ...] in pixels
#         self.object_pose_list = [] # [(x1, y1, z1, id), (x1, y1, z1, id), ...] im meters
#         self.intrinsics = None
#         self.bridge = CvBridge()
#         self.frame = None
#         self.object_pose = Pose()
#         self.tf_stamp = None

#         # connect to CV server
#         host = "192.168.8.99"
#         port = 10001
#         self.c = CustomSocket(host, port)
#         self.c.clientConnect()
#         rospy.loginfo("connected object detection server")

#     def execute(self, userdata):
#         rospy.loginfo('Executing state GetObjectPose')

#         def run_once():
#             while self.intrinsics is None:
#                 time.sleep(0.1)
#             rospy.loginfo("realsense image width, height = ({}, {})".format(self.intrinsics.width, self.intrinsics.height))
#             self.c.req(np.random.randint(255, size=(720, 1280, 3), dtype=np.uint8))

#         def reset():
#             rospy.loginfo("Reseting the value")
#             self.frame = None
#             rospy.sleep(0.1)
#             rospy.loginfo("Finished reseting")
#             self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, yolo_callback, queue_size=1, buff_size=52428800)
#             self.depth_sub = rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, depth_callback, queue_size=1, buff_size=52428800)

#         def detect():
#             rospy.loginfo("Start detecting")
#             # send frame to server and recieve the result
#             result = self.c.req(self.frame)
#             self.frame = check_image_size_for_ros(self.frame)
#             rospy.loginfo("result {}".format(result))
#             if result['n'] == 0:
#                 return None 
#             # object detection bounding box 2d
#             for bbox in result['bbox_list']:
                
#                 # receive xyxy
#                 x_pixel = int(bbox[0] + (bbox[2]-bbox[0])/2)
#                 y_pixel = int(bbox[1] + (bbox[3]-bbox[1])/2)
#                 (x_pixel, y_pixel) = rescale_pixel(x_pixel, y_pixel)
#                 object_id = 1 # TODO change to object tracker
#                 self.center_pixel_list.append((x_pixel, y_pixel, object_id))
#                 # visualize purpose
#                 self.frame = cv2.circle(self.frame, (x_pixel, y_pixel), 5, (0, 255, 0), 2)
#                 self.frame = cv2.rectangle(self.frame, rescale_pixel(bbox[0], bbox[1]), rescale_pixel(bbox[2], bbox[3]), (0, 255, 0), 2)
            
#             self.image_pub.publish(self.bridge.cv2_to_imgmsg(self.frame, "bgr8"))

#             # 3d pose
#             if not self.intrinsics:
#                 rospy.logerr("no camera intrinsics")
#                 return None
#             for center_pixel in self.center_pixel_list:
#                 rospy.loginfo("found {}".format(center_pixel))
#                 depth = self.depth_image[center_pixel[1], center_pixel[0]] # [y, x] for numpy array
#                 result = rs2.rs2_deproject_pixel_to_point(self.intrinsics, [center_pixel[0], center_pixel[1]], depth) # [x, y] for realsense lib
#                 x_coord, y_coord, z_coord = result[0]/1000, result[1]/1000, result[2]/1000

#                 # filter only object with more than 50 cm
#                 if z_coord >= 0.5:
#                     rospy.sleep(0.1)
#                     self.object_pose_list.append((x_coord, y_coord, z_coord, center_pixel[2]))

#                     self.tf_stamp = TransformStamped()
#                     self.tf_stamp.header.frame_id = "/camera_link"
#                     self.tf_stamp.header.stamp = rospy.Time.now()
#                     self.tf_stamp.child_frame_id = "/object_frame_{}".format(center_pixel[2]) # object_id
#                     self.tf_stamp.transform.translation.x = z_coord
#                     self.tf_stamp.transform.translation.y = -x_coord
#                     self.tf_stamp.transform.translation.z = -y_coord

#                     quat = tf.transformations.quaternion_from_euler(
#                         float(0), float(0), float(0))

#                     self.tf_stamp.transform.rotation.x = quat[0]
#                     self.tf_stamp.transform.rotation.y = quat[1]
#                     self.tf_stamp.transform.rotation.z = quat[2]
#                     self.tf_stamp.transform.rotation.w = quat[3]

#             self.image_sub.unregister()
#             self.depth_sub.unregister()
#             rospy.loginfo("Object found!")
#             # self.object_pose = find_closest_object()

#         # function used in callback functions
#         def check_image_size_for_cv(frame):
#             if frame.shape[0] != 720 and frame.shape[1] != 1280:
#                 frame = cv2.resize(frame, (1280, 720))
#             return frame

#         def check_image_size_for_ros(frame):
#             if frame.shape[0] != self.intrinsics.height and frame.shape[1] != self.intrinsics.width:
#                 frame = cv2.resize(frame, (self.intrinsics.width, self.intrinsics.height))
#             return frame

#         def rescale_pixel(x, y):
#             x = int(x*self.intrinsics.width/1280)
#             y = int(y*self.intrinsics.height/720)
#             return (x, y)

#         def find_closest_object():
#             object_pose_z_min = None
#             z_min = 10000000000
#             for object_pose in self.object_pose_list:
#                 if object_pose[2] < z_min:
#                     object_pose_z_min = object_pose
#                     z_min = object_pose[2]
                    
#             return xyz_to_pose(object_pose_z_min[0], object_pose_z_min[1], object_pose_z_min[2])

#         def xyz_to_pose(x, y, z):
#             """
#             transform xyz in realsense coord to camera_link coord
#             """
#             # set object pose
#             object_pose = Pose()
#             object_pose.position.x = z
#             object_pose.position.y = -x
#             object_pose.position.z = -y
#             object_pose.orientation.x = 0
#             object_pose.orientation.y = 0
#             object_pose.orientation.z = 0
#             object_pose.orientation.w = 1
#             return object_pose

#         # all call_back functions
#         def info_callback(cameraInfo):
#             try:
#                 if self.intrinsics:
#                     return
#                 self.intrinsics = rs2.intrinsics()
#                 self.intrinsics.width = cameraInfo.width
#                 self.intrinsics.height = cameraInfo.height
#                 self.intrinsics.ppx = cameraInfo.K[2]
#                 self.intrinsics.ppy = cameraInfo.K[5]
#                 self.intrinsics.fx = cameraInfo.K[0]
#                 self.intrinsics.fy = cameraInfo.K[4]
#                 if cameraInfo.distortion_model == 'plumb_bob':
#                     self.intrinsics.model = rs2.distortion.brown_conrady
#                 elif cameraInfo.distortion_model == 'equidistant':
#                     self.intrinsics.model = rs2.distortion.kannala_brandt4
#                 self.intrinsics.coeffs = [i for i in cameraInfo.D]
#             except CvBridgeError as e:
#                 print(e)
#                 return

#         def yolo_callback(data):
#             try:
#                 # change subscribed data to numpy.array and save it as "frame"
#                 self.frame = self.bridge.imgmsg_to_cv2(data, 'bgr8')
#                 # scale image incase image size donot match cv server
#                 self.frame = check_image_size_for_cv(self.frame)
#             except CvBridgeError as e:
#                 print(e)

#         def depth_callback(frame):
#             """
#                                 +Z           
#             -y   realsense frame|                
#             | +z                |    
#             |/                  |      
#             o---> +x            |  +X    
#                                 | / 
#             +Y -----------------o camera_link frame tf/

#             """
#             try:
#                 if self.tf_stamp is not None:
#                     # rospy.loginfo("publishing tf")
#                     self.tf_stamp.header.stamp = rospy.Time.now()
#                     self.pub_tf.publish(tf2_msgs.msg.TFMessage([self.tf_stamp]))

#                 self.depth_image = self.bridge.imgmsg_to_cv2(frame, frame.encoding)
#                 # rescale pixel incase pixel donot match
#                 self.depth_image = check_image_size_for_ros(self.depth_image)

#             except CvBridgeError as e:
#                 print(e)
#                 return
#             except ValueError as e:
#                 return
#             pass

#         # ----------------------------------------------start-----------------------------------------------------
#         # subscribe topics
#         rospy.Subscriber(
#             "/camera/aligned_depth_to_color/camera_info", CameraInfo, info_callback)
#         self.image_sub = rospy.Subscriber(
#             "/camera/color/image_raw", Image, yolo_callback, queue_size=1, buff_size=52428800)
#         self.depth_sub = rospy.Subscriber(
#             "/camera/aligned_depth_to_color/image_raw", Image, depth_callback, queue_size=1, buff_size=52428800)
#         self.image_pub = rospy.Publisher(
#             "/blob/image_blob", Image, queue_size=1)
#         self.pub_tf = rospy.Publisher(
#             "/tf", tf2_msgs.msg.TFMessage, queue_size=1)

#         # recieving object name from GetObjectName state
#         # self.object_name = userdata.objectname_input
#         # rospy.loginfo(self.object_name)

#         # run_once function
#         run_once()
#         while not rospy.is_shutdown():
#             command = raw_input("Press Enter :")
#             if command == 'q':
#                 break
#             rospy.loginfo("------ Running 3D detection ------")
#             reset()
#             detect()
#             userdata.objectlist_output = self.object_pose_list
#             # userdata.objectpose_output = self.object_pose
#             return 'continue_Pick'
#         return 'continue_ABORTED'

# class Place :

    # def __init__(self, corner_table11, corner_table12, corner_table21, corner_table22, high, current_collision_object_pos) :
    #     self.corner_table11 = corner_table11
    #     self.corner_table12 = corner_table12
    #     self.corner_table21 = corner_table21
    #     self.corner_table22 = corner_table22
    #     self.high = high
    #     self.current_collision_object_pos = current_collision_object_pos

    #     rospy.loginfo('initiating place state')
    #     smach.State.__init__(self, outcomes =['continue_aborted','continue_succeeded'])

    # def execute(self) :

    #     def place_service() :
    #         rospy.wait_for_service('cr3_place')
    #         try:
    #             place = rospy.ServiceProxy('cr3_place', cr3_place)
    #             res = place(self.corner_table11, self.corner_table12, self.corner_table21, self.corner_table22, self.high, self.current_collision_object_pos)
    #             return res.success_place
    #         except rospy.ServiceException as e:
    #             print("Service call failed: %s"%e)

    #     rospy.loginfo(self.corner_table11, self.corner_table12, self.corner_table21, self.corner_table22, self.high, self.current_collision_object_pos)
    #     success = place_service()
    #     print(success)

    #     if success :
    #         return "continue_aborted"
    #     else :
    #         return "continue_succeeded"








    # class GetPoseEnvironment(smach.State):
    # def __init__(self):
    #     rospy.loginfo('initiating create environment state')
    #     smach.State.__init__(self, outcomes = ['continue_GetObjectPoseList']
    #                              , output_keys = ['corner11_output', 'corner12_output', 'corner21_output', 'corner22_output', 'high_output'])
    #     self.sub_line_min_dist = None
    #     self.corner11 = Pose()
    #     self.corner12 = Pose()
    #     self.corner21 = Pose()
    #     self.corner22 = Pose()
    #     self.high = Pose()

    # def execute(self, userdata):

    #     def transform_pose(input_pose, from_frame, to_frame):
    #         # **Assuming /tf2 topic is being broadcasted
    #         tf_buffer = tf2_ros.Buffer()
    #         listener = tf2_ros.TransformListener(tf_buffer)

    #         pose_stamped = tf2_geometry_msgs.PoseStamped()
    #         pose_stamped.pose = input_pose
    #         pose_stamped.header.frame_id = from_frame
    #         # pose_stamped.header.stamp = rospy.Time.now()

    #         try:
    #             # ** It is important to wait for the listener to start listening. Hence the rospy.Duration(1)
    #             output_pose_stamped = tf_buffer.transform(pose_stamped, to_frame, rospy.Duration(1))
    #             return output_pose_stamped.pose

    #         except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
    #             raise

    #     def cb_furniture(marker_array) :
    #         self.corner11.position = marker_array.markers[0].points[0]
    #         self.corner12.position = marker_array.markers[0].points[1]
    #         self.corner21.position = marker_array.markers[0].points[3]
    #         self.corner22.position = marker_array.markers[0].points[2]
    #         self.high.position = marker_array.markers[0].points[0].z

    #         userdata.corner11_output = transform_pose(self.corner11, "world", "base_link")
    #         userdata.corner12_output = transform_pose(self.corner12, "world", "base_link")
    #         userdata.corner21_output = transform_pose(self.corner21, "world", "base_link")
    #         userdata.corner22_output = transform_pose(self.corner22, "world", "base_link")
    #         userdata.high_output = self.high

    #     self.sub_line_min_dist = rospy.Subscriber('furniture_marker', MarkerArray, cb_furniture)
        
    #     return 'continue_GetObjectPoseList'
