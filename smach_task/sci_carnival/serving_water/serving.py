#!/usr/bin/env python

import roslib
import rospy
import smach
import smach_ros
from geometry_msgs.msg import Pose, PoseStamped, Twist, Quaternion
from tf.transformations import quaternion_from_euler
import math

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

import moveit_planning_scene_interface

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

# useful function
def lift_cb(data) :
    while (not data.data) : pass

def lift_command(cmd) :
    """
    lift command
    up is True
    down is False
    """
    
    time.sleep(1)
    lift_pub.publish(cmd)

    rospy.Subscriber("done", Bool, lift_cb)

def add_table(table_name, scene):
    global ed
    cxl = []
    cyl = []

    table_pose = PoseStamped()
    table_pose.header.frame_id = "map"
    ed_point = ed.get_center_point(table_name)
    table_pose.pose.position.x = ed_point.x
    table_pose.pose.position.y = ed_point.y
    table_pose.pose.position.z = ed_point.z

    height = ed.get_height()
    corner_list = ed.get_corner_list()
    for corner in corner_list:
        cxl.append(corner.x)
        cyl.append(corner.y)
    
    length_x = max(cxl) - min(cxl)
    length_y = max(cyl) - min(cyl)



    table_pose.pose.orientation.w = 1.0
    scene.add_box(table_name, table_pose, size=(length_x, length_y, height))
    rospy.sleep(3)
    return True


    
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

            raw_input("press enter to move to home position:")
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

    move_group.set_planner_id("RRT*")
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
    

    raw_input("using %s ,press enter:" %move_group.get_planner_id())

    move_group.execute(plan, wait=True)

def catesian_go(goal_pose = Pose(),move_group = moveit_commander.MoveGroupCommander("arm"),robot = moveit_commander.RobotCommander()):
    waypoints = []
    waypoints.append(goal_pose)

    plan, fraction = move_group.compute_cartesian_path(waypoints, 0.01, 0.0)

    display_trajectory = DisplayTrajectory()
    display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory.append(plan)
    display_trajectory_publisher.publish(display_trajectory)

    raw_input("using catesian path with fraction %f ,press enter:" % fraction)
    move_group.execute(plan, wait=True)

# state
class GetObjectName(smach.State):
    def __init__(self):
        rospy.loginfo('Initiating state GetObjectName')
        smach.State.__init__(self, outcomes=['continue_GetObjectPose'], output_keys=['objectname_output'])

    def execute(self, userdata):
        rospy.loginfo('Executing state GetObjectName')
        # sending object name to GetobjectName state (change string right here)
        userdata.objectname_output = OBJECT_NAME
        return 'continue_GetObjectPose'


class GetObjectPose(smach.State):
    def __init__(self):
        global object_detection
        rospy.loginfo('Initiating state GetObjectPose')
        smach.State.__init__(self, outcomes=['continue_Pick', 'continue_ABORTED'], input_keys=['objectname_input', 'objectpose_output'], output_keys=['objectpose_output'])
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
            command = raw_input("Press Enter :")
            if command == 'q':
                break
            rospy.loginfo("------ Running 3D detection ------")
            reset()
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
            group_name = "arm"
            move_group = moveit_commander.MoveGroupCommander(group_name)
            scene = moveit_commander.PlanningSceneInterface()
            robot = moveit_commander.RobotCommander()

            add_table("dinner_table",scene)

            pose_goal = Pose()
            # q = quaternion_from_euler(3.14,-1.57,0)
            # pose_goal.orientation = Quaternion(*q)
            pose_goal.position.x = pose.position.x
            pose_goal.position.y = pose.position.y-0.05
            pose_goal.position.z = pose.position.z+0.05
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

            grasp_pose .position.x    = pose_goal.position.x - 0.02
            grasp_pose .position.y    = pose_goal.position.y
            grasp_pose .position.z    = pose_goal.position.z
            grasp_pose .orientation.x = pose_goal.orientation.x
            grasp_pose .orientation.y = pose_goal.orientation.y
            grasp_pose .orientation.z = pose_goal.orientation.z
            grasp_pose .orientation.w = pose_goal.orientation.w

            pregrasp_pose.position.x    = pose_goal.position.x - 0.15
            pregrasp_pose.position.y    = pose_goal.position.y
            pregrasp_pose.position.z    = pose_goal.position.z
            pregrasp_pose.orientation.x = pose_goal.orientation.x
            pregrasp_pose.orientation.y = pose_goal.orientation.y
            pregrasp_pose.orientation.z = pose_goal.orientation.z
            pregrasp_pose.orientation.w = pose_goal.orientation.w

            lift_pose.position.x    = pose_goal.position.x - 0.15
            lift_pose.position.y    = pose_goal.position.y
            lift_pose.position.z    = pose_goal.position.z + 0.15
            lift_pose.orientation.x = pose_goal.orientation.x
            lift_pose.orientation.y = pose_goal.orientation.y
            lift_pose.orientation.z = pose_goal.orientation.z
            lift_pose.orientation.w = pose_goal.orientation.w

            print(grasp_pose)
            print(pregrasp_pose)
            print(lift_pose)
            
            go_to_pose_goal(pregrasp_pose, move_group, robot)
            rospy.sleep(3)
            raw_input("enter to open gripper")
            gripper_publisher.publish(False)
            rospy.sleep(3)
            catesian_go(grasp_pose, move_group, robot)
            raw_input("enter to close gripper")
            gripper_publisher.publish(True)
            rospy.sleep(3)
            catesian_go(lift_pose, move_group, robot)
            rospy.sleep(3)
            set_home_walkie(move_group)

            return True

            

        # lift up
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
            return 'continue_navigate_volunteer'
        else:
            return 'continue_ABORTED'


# main
if __name__ == "__main__":

    rospy.init_node('smach_example_state_machine')

    ##############################################
    OBJECT_NAME = "waterbottle"
    # OBJECT_NAME = "bubble_tea"
    ############################################
    
    #Environment Descriptor for going to person position
    ed = EnvironmentDescriptor("/home/eic/ros/smach/smach_task/config/fur_data_onsite.yaml")
    
    #navigation manager
    # navigation = go_to_Navigation()


    #Nlp and cv server
    host = '0.0.0.0'
    port_object = 10008
    object_detection = CustomSocket(host=host, port=port_object)
    object_detection.clientConnect()

    # Flask nlp server
    stt = SpeechToText("nlp")
    t = threading.Thread(target = stt.run ,name="nlp")
    t.start()

    # publisher and subscriber
    lift_pub                     = rospy.Publisher('lift_command', Bool, queue_size=1)
    a                            = rospy.Publisher("posem",Marker,queue_size=1)
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',DisplayTrajectory, queue_size=1)
    gripper_publisher            = rospy.Publisher('/cr3_gripper_command',Bool,queue_size=1)

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['SUCCEEDED','ABORTED'])
    sm.userdata.string_name = ""
    sm.userdata.object_pose = Pose()
    # Open the container
    with sm:
        # ------------------------------ Pick Object --------------------------------------
        smach.StateMachine.add('GetObjectName', GetObjectName(),
                               transitions={'continue_GetObjectPose': 'GetObjectPose'},
                               remapping={'objectname_output': 'string_name'})
        smach.StateMachine.add('GetObjectPose', GetObjectPose(),
                               transitions={'continue_Pick': 'Pick',
                                            'continue_ABORTED': 'ABORTED'},
                               remapping={'objectname_input': 'string_name',
                                          'objectpose_output': 'object_pose',
                                          'objectpose_output': 'object_pose'})
        smach.StateMachine.add('Pick', Pick(),
                               transitions={'continue_navigate_volunteer': 'SUCCEEDED',
                                            'continue_ABORTED': 'ABORTED'},
                               remapping={'objectpose_input': 'object_pose'})
        # ----------------------------------------------------------------------------------

    # Execute SMACH plan
    outcome = sm.execute()
