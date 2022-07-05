#!/usr/bin/env python

import rospy
from cv_bridge import CvBridge, CvBridgeError
import cv2
import pyrealsense2 as rs2
from sensor_msgs.msg import Image, CameraInfo
import time

class Realsense():
    def __init__(self):
    
        self.intrinsics = None
        self.depth_image = None
        self.frame = None
        self.bridge = CvBridge()
        self.ref = (1280,720)
        
        rospy.Subscriber("/camera/aligned_depth_to_color/camera_info", CameraInfo, self.info_callback)
        rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, self.depth_callback, queue_size=1, buff_size=52428800)
        rospy.Subscriber("/camera/color/image_raw", Image , self.image_callback)

    def rescale_pixel(self, x, y):
            x = int(x*self.intrinsics.width/self.ref[0])
            y = int(y*self.intrinsics.height/self.ref[1])
            return (x, y)

    def check_image_size_for_ros(self, frame):
        if frame.shape[0] != self.intrinsics.height and frame.shape[1] != self.intrinsics.width:
            frame = cv2.resize(frame, (self.intrinsics.width, self.intrinsics.height))
        return frame

    def check_image_size_for_cv(self, frame):
        if frame.shape[0] != 720 and frame.shape[1] != 1280:
            frame = cv2.resize(frame, (1280, 720))
        return frame

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
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(frame, frame.encoding)
        except CvBridgeError as e:
            print(e)
            return
        except ValueError as e:
            return
        pass

    def image_callback(self, data):
        try:
            # change subscribed data to numpy.array and save it as "frame"
            self.frame = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        except CvBridgeError as e:
            print(e)

    def get_coordinate(self, x, y, ref=(1280,720)):
        """
        pixel = (x, y) in pixels relative to (1280, 720) from cv server
        ref = (width, height)
        return (x, y, z) in meters
        """
        self.ref = ref
        x_pixel, y_pixel = self.rescale_pixel(x, y)
        self.depth_image = self.check_image_size_for_ros(self.depth_image)
        depth = self.depth_image[y_pixel,  x_pixel] # [y, x] for numpy array
        result = rs2.rs2_deproject_pixel_to_point(self.intrinsics, [x_pixel, y_pixel], depth) # [x, y] for realsense builtin function
        return result[0]/1000, result[1]/1000, result[2]/1000
    
    def get_image(self):
        return self.frame

    def wait(self):
        while self.intrinsics is None or self.depth_image is None or self.frame is None:
            rospy.loginfo("no camera intrinsics received")
            time.sleep(0.1)

if __name__ == "__main__":
    rospy.init_node('realsense_module')
    rs = Realsense()
    rs.wait()
    while not rospy.is_shutdown():
        print(rs.get_coordinate(640, 360))
    # rospy.spin()