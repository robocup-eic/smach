    #!/usr/bin/env python

import roslib
import rospy
import smach
import smach_ros
from client.custom_socket import CustomSocket
# navigation
import tf2_ros
from nav_msgs.msg import Odometry
from math import pi
import tf
import tf2_msgs
# ros pub sub
from std_msgs.msg import Bool
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo

import threading
# realsense and computer vision

import cv2
import numpy as np
import pyrealsense2.pyrealsense2 as rs2
from geometry_msgs.msg import PoseStamped, Twist ,Vector3, TransformStamped,Point
import socket
from util.custom_socket import CustomSocket
# import yaml reader
from util.guest_name_manager import GuestNameManager
from util.environment_descriptor import EnvironmentDescriptor
from util.realsense import Realsense

import tf2_geometry_msgs
from geometry_msgs.msg import Pose

class Navigation(smach.State):
    def __init__(self):
        rospy.loginfo('Initiating Navigation state')
        smach.State.__init__(self,outcomes=['continue_No_seat','continue_Seat'])
        global gm
        # init list of seat
        self.chair_list = gm.get_chair_poses()
        self.person_list = []
        self.bridge = CvBridge()

    def execute(self,userdata):
        rospy.loginfo('Executing Navigation state')
        global static_broadcaster, dynamic_broadcaster, tf_listener, ed

        def detect(frame):
            # scale image incase image size donot match cv server
            frame = rs.check_image_size_for_cv(frame)
            # send frame to server and recieve the result
            result = personTrack.req(frame)
            # rescale pixel incase pixel donot match
            frame = rs.check_image_size_for_ros(frame)

            # init person_list
            person_list = []

            print(result)

            # if there is no person just skip
            if len(result["result"]) == 0:
                image_pub.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))
                return

            for person in result["result"]:
                person_id = person[0]
                self.x_pixel = int((person[2][0]+person[2][2])/2)
                self.y_pixel = int((person[2][1]+person[2][3])/2)
                self.x_pixel, self.y_pixel = rs.rescale_pixel(self.x_pixel, self.y_pixel)

                # visualize purpose
                frame = cv2.circle(frame, (self.x_pixel, self.y_pixel), 5, (0, 255, 0), 2)
                frame = cv2.rectangle(frame, rs.rescale_pixel(person[2][0], person[2][1]), rs.rescale_pixel(person[2][2], person[2][3]), (0, 255, 0), 2)
                frame = cv2.putText(frame, str(person_id), rs.rescale_pixel(person[2][0], person[2][1] + 10), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
                
                # get 3d person point
                person_pose = Pose()
                person_pose.position.x, person_pose.position.y, person_pose.position.z = rs.get_coordinate(self.x_pixel, self.y_pixel, ref=(frame.shape[1], frame.shape[0]))
                person_pose.orientation.x, person_pose.orientation.y, person_pose.orientation.z, person_pose.orientation.w = 0,0,0,1
                person_list.append((person_id, person_pose))

            image_pub.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))

            # data from comuter vision realsense x is to the right, y is to the bottom, z is toward.
            return person_list

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

        def tf_all_people(person_list):
            if len(person_list) == 0:
                return False
            for person in person_list:
                person_id      = str(person[0])
                person_point   = person[1]

                dynamic_tf_stamp                         = TransformStamped()
                dynamic_tf_stamp.header.stamp            = rospy.Time.now()
                dynamic_tf_stamp.header.frame_id         = "camera_link"
                dynamic_tf_stamp.child_frame_id          = person_id
                dynamic_tf_stamp.transform.translation.x = person_point.x
                dynamic_tf_stamp.transform.translation.y = person_point.y
                dynamic_tf_stamp.transform.translation.z = person_point.z
                dynamic_tf_stamp.transform.rotation.x    = 0
                dynamic_tf_stamp.transform.rotation.y    = 0
                dynamic_tf_stamp.transform.rotation.z    = 0
                dynamic_tf_stamp.transform.rotation.w    = 1

                dynamic_broadcaster.sendTransform(dynamic_tf_stamp)

            rospy.loginfo('Broadcate all Person in tf-tree')
            return True

        def tf_all_chair(chair_list):
            for chair in chair_list:
                point = ed.get_center_point(chair)

                static_tf_stamp                         = TransformStamped()
                static_tf_stamp.header.stamp            = rospy.Time.now()
                static_tf_stamp.header.frame_id         = "map"
                static_tf_stamp.child_frame_id          = chair
                static_tf_stamp.transform.translation.x = point.x 
                static_tf_stamp.transform.translation.y = point.y
                static_tf_stamp.transform.translation.z = point.z
                static_tf_stamp.transform.rotation.x    = 0
                static_tf_stamp.transform.rotation.y    = 0
                static_tf_stamp.transform.rotation.z    = 0
                static_tf_stamp.transform.rotation.w    = 1

                static_broadcaster.sendTransform(static_tf_stamp)

            rospy.loginfo('Broadcate all Seat in tf-tree')
            return True

        def avaliable_seat_list():
            avaliable_seat = self.chair_list.copy()

            for person in self.person_list:
                person_id      = str(person[0])
                min_distance = 10000000
                person_pose = person[1]
                # compare with chair

                for chair in self.chair_list: ## TODO ed get chair list
                    chair_pose = chair["pose"]
                    print(chair_pose)
                    distance = float(((chair_pose.position.x - person_pose.position.x)**2 + (chair_pose.position.y - person_pose.position.y)**2)**0.5)

                    if distance < min_distance:
                        min_distance = distance
                        bond_chair = chair
                avaliable_seat.remove(bond_chair)
            
            return avaliable_seat
        
        #===============================================start=============================================
        # start person tracker
        rospy.sleep(0.5)
        result_person_list = []


        for i in range(10):
            result_person_list = detect(rs.get_image())
            if result_person_list is not None:
                for result in result_person_list:
                    # if there are no
                    if result[0] != [r[0] for r in self.person_list]:
                        self.person_list.append(result)


        # for i in range(9):
        #     result_person_list = detect(rs.get_image())
        #     for result in result_person_list:
        #         # 
        #         if result[0] in [r[0] for r in self.person_list]:
        #             self.person_list.append(result)

        #     if len(result_person_list) != 0:
        #         self.person_list = result_person_list
        # print(self.person_list)

        if tf_all_people(self.person_list) and tf_all_chair(self.chair_list):
            avaliable_seat = avaliable_seat_list()
            print(avaliable_seat)
        
        if len(avaliable_seat) == 0:
            return 'continue_No_seat'
        else:
            return 'continue_Seat'

if __name__ == '__main__':
    rospy.init_node('receptionist_task')

    tf_Buffer = tf2_ros.Buffer()
    static_broadcaster = tf2_ros.StaticTransformBroadcaster()
    dynamic_broadcaster = tf2_ros.TransformBroadcaster()
    tf_listener = tf2_ros.TransformListener(tf_Buffer)

    ed = EnvironmentDescriptor("../config/fur_data.yaml")
    gm = GuestNameManager("../config/receptionist_database.yaml")
    gm.reset()
    person_count = 0

    rs = Realsense()
    rs.wait() # wait for camera intrinsics

    image_pub = rospy.Publisher("/blob/image_blob", Image, queue_size=1)

    # connect to server
    host = "192.168.8.99"
    # host = socket.gethostname()

    # face recognition model
    port_faceRec = 10006
    faceRec = CustomSocket(host,port_faceRec)
    faceRec.clientConnect()

    # person tracker model
    port_personTrack = 11000
    personTrack = CustomSocket(host,port_personTrack)
    personTrack.clientConnect()

    # Create a SMACH state machine
    sm_top = smach.StateMachine(outcomes=['No_seat','Seat'])
    # Open the container
    with sm_top:
        # Add states to the container
        smach.StateMachine.add('Navigation', Navigation(),
                               transitions={'continue_No_seat':'No_seat',
                                            'continue_Seat':'Seat'})

    sis = smach_ros.IntrospectionServer('Server_name', sm_top, '/Receptionist')
    sis.start()

    # Execute SMACH plan
    outcome = sm_top.execute()
    rospy.spin()
    sis.stop()









