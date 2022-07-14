#!/usr/bin/env python

"""
roslaunch walkie2_bringup walkie2_bringup.launch
roslaunch wakie2_bringup cr3_bringup.launch
ssh to pi and $ ./mega.sh
rviz
python /home/eic/ros/smach/smach_task/manipulation/manipulation_pipeline_smach.py
"""

import roslib
import rospy
import smach
import smach_ros
from geometry_msgs.msg import Pose, PoseStamped
from tf.transformations import quaternion_from_euler
import math
import time

# pick service
from cr3_moveit_control.srv import PickWithSide

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

# # output
from geometry_msgs.msg import Pose

# transform to from cameralink to base link
import tf2_ros
import tf2_geometry_msgs

# lift
from std_msgs.msg import Bool

# cr3 command
import moveit_commander

# useful function
def lift_cb(data) :
    while (not data.data) : pass

def lift_command(cmd) :
    """
    lift command
    up is True
    down is False
    """
    lift_pub = rospy.Publisher('lift_command', Bool, queue_size=1)
    time.sleep(1)
    lift_pub.publish(cmd)

    rospy.Subscriber("done", Bool, lift_cb)


class GetObjectName(smach.State):
    def __init__(self):
        rospy.loginfo('Initiating state GetObjectName')
        smach.State.__init__(self, outcomes=['continue_GetObjectPose'], output_keys=[
                             'objectname_output'])

    def execute(self, userdata):
        rospy.loginfo('Executing state GetObjectName')
        # sending object name to GetobjectName state (change string right here)
        userdata.objectname_output = OBJECT_NAME
        return 'continue_GetObjectPose'


class GetObjectPose(smach.State):
    def __init__(self):
        rospy.loginfo('Initiating state GetObjectPose')
        smach.State.__init__(self, outcomes=['continue_Pick', 'continue_ABORTED'], input_keys=[
                             'objectname_input', 'objectpose_output'], output_keys=['objectpose_output'])
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
        host = "0.0.0.0"
        port = 10008
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
            # scale image incase image size donot match cv server
            self.frame = check_image_size_for_cv(self.frame)
            # send frame to server and recieve the result
            result = {"n":0}
            while result['n'] == 0:
                result = self.c.req(self.frame)
            self.frame = check_image_size_for_ros(self.frame)
            rospy.loginfo("result {}".format(result))
            if result['n'] == 0:
                return None 
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

        # arm sethome
        set_home_walkie()

        # lift down
        lift_command(False)

        # run_once function
        run_once()
        while not rospy.is_shutdown():
            command = raw_input("Press Enter :")
            if command == 'q':
                break
            rospy.loginfo("------ Running 3D detection ------")
            reset()
            detect()
            userdata.objectpose_output = self.object_pose
            rospy.loginfo(self.object_pose)
            return 'continue_Pick'
        return 'continue_ABORTED'


class Pick(smach.State):
    def __init__(self):
        rospy.loginfo('Initiating state Pick')
        smach.State.__init__(self, outcomes=['continue_SUCCEEDED', 'continue_ABORTED'], 
                                   input_keys=['objectpose_input'])
        self.success = False
        self.tf_buffer =  tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)

    def execute(self, userdata):
        rospy.loginfo('Executing state Pick')
        # recieving Pose() from GetObjectPose state
        recieved_pose = userdata.objectpose_input

        # tune coordinate
        recieved_pose.position.x -= 0.05
        recieved_pose.position.y += 0.03
        recieved_pose.position.z += 0.05

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

        # lift up
        lift_command(True)

        transformed_pose = transform_pose(
            recieved_pose, "realsense_pitch", "cr3_base_link")
        rospy.loginfo(transformed_pose.position.x)
        transformed_pose.orientation.x = 0
        transformed_pose.orientation.y = 0
        transformed_pose.orientation.z = 0
        transformed_pose.orientation.w = 1

        self.success = pick_service(transformed_pose, 'front')

        if self.success == True:
            return 'continue_SUCCEEDED'
        else:
            return 'continue_ABORTED'


def main():
    rospy.init_node('smach_pick_state_machine')
    sm = smach.StateMachine(outcomes=['SUCCEEDED', 'ABORTED'])
    sm.userdata.string_name = ""
    sm.userdata.object_pose = Pose()
    with sm:
        smach.StateMachine.add('GetObjectName', GetObjectName(),
                               transitions={
                                   'continue_GetObjectPose': 'GetObjectPose'},
                               remapping={'objectname_output': 'string_name'})
        smach.StateMachine.add('GetObjectPose', GetObjectPose(),
                               transitions={'continue_Pick': 'Pick',
                                            'continue_ABORTED': 'ABORTED'},
                               remapping={'objectname_input': 'string_name',
                                          'objectpose_output': 'object_pose',
                                          'objectpose_output': 'object_pose'})
        smach.StateMachine.add('Pick', Pick(),
                               transitions={'continue_SUCCEEDED': 'SUCCEEDED',
                                            'continue_ABORTED': 'ABORTED'},
                               remapping={'objectpose_input': 'object_pose'})
    outcome = sm.execute()


if __name__ == "__main__":
    OBJECT_NAME = "Waterbottle"
    main()
