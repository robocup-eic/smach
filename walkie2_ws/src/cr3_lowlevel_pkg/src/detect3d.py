#!/usr/bin/env python

"""
run
$ roslaunch realsense2_camera rs_rgbd.launch align_depth:=true  color_width:=848 color_height:=480 color_fps:=30 filters:=pointcloud
$ rosrun cr3_lowlevel_pkg detect3d.py

publish image to
    /blob/image_blob
"""
import rospy

# realsense
import pyrealsense2 as rs2
import cv2
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError

# tf for visualization
import tf2_ros
import tf2_msgs
import tf
from geometry_msgs.msg import TransformStamped

# computer vision
import socket
from custom_socket import CustomSocket
import numpy as np

# output
from geometry_msgs.msg import Pose

def simple_detect_bbox(img, color="blue"):
    # simple object detection using color
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    # lower_bound : red [136, 87, 111], green [25, 52, 72], blue [94, 80, 2]
    # upper_bound : red [180, 255, 255], green [102, 255, 255], [120, 255, 255]
    if color == "red":
        lower_bound = np.array([136, 87, 111])
        upper_bound = np.array([180, 255, 255])
    elif color == "blue":
        lower_bound = np.array([94, 80, 2])
        upper_bound = np.array([120, 255, 255])
    elif color == "green":
        lower_bound = np.array([25, 52, 72])
        upper_bound = np.array([102, 255, 255])

    mask = cv2.inRange(hsv, lower_bound, upper_bound)

    # define kernel size
    kernel = np.ones((7, 7), np.uint8)
    # Remove unnecessary noise from mask
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    # Find contours from the mask
    items = cv2.findContours(
        mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    # print(contours)
    contours = items[0] if len(items) == 2 else items[1]

    x, y, w, h = 0, 0, 0, 0
    # find biggest contours
    if len(contours) != 0:
        c = max(contours, key = cv2.contourArea)
        area = cv2.contourArea(c)
        if (area > 300):
            x, y, w, h = cv2.boundingRect(c)
            img = cv2.rectangle(img, (x, y),
                                (x + w, y + h),
                                (0, 0, 255), 2)

            # cv2.putText(img, "object", (x, y),
            #             cv2.FONT_HERSHEY_SIMPLEX, 1.0,
            #             (0, 0, 255))
                        
    return img, x, y, w, h

class GetObjectPose():
    def __init__(self):

        rospy.loginfo('Initialize state GetObjectPose')
        image_sub = rospy.Subscriber("/camera/color/image_raw", Image , self.yolo_callback)
        depth_sub = rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image , self.depth_callback)
        depth_info_sub = rospy.Subscriber("/camera/aligned_depth_to_color/camera_info", CameraInfo , self.info_callback)
        self.image_pub = rospy.Publisher("/blob/image_blob",Image,queue_size=1)

        # connect to server
        # host = socket.gethostname()
        host = "https://e0cf-182-232-62-93.ap.ngrok.io"
        port = 10001
        self.c = CustomSocket(host,port)
        self.c.clientConnect()

        self.x_pixel = None
        self.y_pixel = None  
        self.intrinsics = None

        self.pub_tf = rospy.Publisher("/tf", tf2_msgs.msg.TFMessage, queue_size=1)
        self.tfm = tf2_msgs.msg.TFMessage()

        self.bridge = CvBridge()

        self.is_trigger = False
        self.is_done = False
        self.object_pose = Pose()
        self.frame = None

    def detect(self):
        self.is_trigger = True
        while not self.is_done:
            pass
        self.is_trigger = False
        self.is_done = False
        return self.object_pose
            
    
    def info_callback(self, cameraInfo):
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
        
    def depth_callback(self, frame):
        """
        +Z           
        |         +y   realsense frame
        |         | +z   o object
        |         |/
        |  +X     o---> +x
        | / 
        0/-----------------> -Y camera_link frame tf

        """
        try:
            if not self.is_trigger:
                self.pub_tf.publish(self.tfm)
                return
            elif not ((self.x_pixel is None) or (self.y_pixel is None)):
                depth_image = self.bridge.imgmsg_to_cv2(frame, frame.encoding)
                # pick one pixel among all the pixels with the closest range:
                pix = (int(self.x_pixel), int(self.y_pixel))
                # line = '\rDepth at pixel(%3d, %3d): %7.1f(mm).' % (pix[0], pix[1], cv_image[pix[1], pix[0]])
                if self.intrinsics:
                    depth = depth_image[pix[1], pix[0]]
                    result = [0,0,0]
                    result = rs2.rs2_deproject_pixel_to_point(self.intrinsics, [pix[0], pix[1]], depth)
                    x_coord, y_coord, z_coord = result[0]/1000, result[1]/1000, result[2]/1000
                    # line += '  Coordinate: %8.2f %8.2f %8.2f.' % (result[0], result[1], result[2])
                    if z_coord >= 0.5:

                        # line += '\r'
                        # print(line)
                        rospy.sleep(0.1)

                        t = TransformStamped()
                        t.header.frame_id = "/camera_link"
                        t.header.stamp = rospy.Time.now()
                        t.child_frame_id = "/object_frame"
                        t.transform.translation.x = z_coord
                        t.transform.translation.y = -x_coord
                        t.transform.translation.z = -y_coord

                        quat = tf.transformations.quaternion_from_euler(float(0),float(0),float(0))

                        t.transform.rotation.x = quat[0]
                        t.transform.rotation.y = quat[1]
                        t.transform.rotation.z = quat[2]
                        t.transform.rotation.w = quat[3]

                        # set tranfrom message to be visualize
                        self.tfm = tf2_msgs.msg.TFMessage([t])

                        # set object pose
                        self.object_pose.position.x = z_coord
                        self.object_pose.position.y = -x_coord
                        self.object_pose.position.z = -y_coord
                        self.object_pose.orientation.x = 0
                        self.object_pose.orientation.y = 0
                        self.object_pose.orientation.z = 0
                        self.object_pose.orientation.w = 0

                        self.is_done = True

        except CvBridgeError as e:
            print(e)
            return
        except ValueError as e:
            return
        pass

    def yolo_callback(self, data): 
        if not self.is_trigger:
            try:
                if self.frame is not None:
                    self.image_pub.publish(self.bridge.cv2_to_imgmsg(self.frame, "bgr8"))
            except CvBridgeError as e:
                print(e)
            return
        else:
            # change subscribed data to numpy.array and save it as "frame"
            self.frame = self.bridge.imgmsg_to_cv2(data,'bgr8')

            self.frame = cv2.resize(self.frame, (720, 1080))
            # 2d object detection
            # send frame to server and recieve the result      
            result = self.c.req(self.frame)
            print(result)

            # self.frame, x, y, w, h = simple_detect_bbox(self.frame, "blue")
            # self.x_pixel = x
            # self.y_pixel = y
            



if __name__ == '__main__':
    rospy.init_node('visual_servo', anonymous=True)
    detector = GetObjectPose()
    while not rospy.is_shutdown():
        command = raw_input("command: ")
        if command == "a":
            object_pose = detector.detect()

        print(object_pose)
    # rospy.spin()