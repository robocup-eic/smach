#!/usr/bin/env python

"""

   0------------------> x (cols) Image Frame
   |
   |        c    Camera frame
   |         o---> x
   |         |
   |         V y
   |
   V y (rows)


SUBSCRIBES TO:
    /raspicam_node/image: Source image topic
    
PUBLISHES TO:
    /blob/image_blob : image with detected blob and search window
    /blob/point_blob : blob position in adimensional values wrt. camera frame

"""


#--- Allow relative importing
if __name__ == '__main__' and __package__ is None:
    from os import sys, path
    sys.path.append(path.dirname(path.dirname(path.abspath(__file__))))
    
import sys
import rospy
import cv2
import numpy as np
import time

from std_msgs.msg           import String
from sensor_msgs.msg        import Image
from geometry_msgs.msg      import Point
from cv_bridge              import CvBridge, CvBridgeError


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

class SimpleColorDetector:

    def __init__(self):
        
        self._t0 = time.time()
        self.blob_point = Point()
    
        print (">> Publishing image to topic image_blob")
        self.image_pub = rospy.Publisher("/blob/image_blob",Image,queue_size=1)
        print (">> Publishing position to topic point_blob")
        self.blob_pub  = rospy.Publisher("/blob/point_blob",Point,queue_size=1)

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw",Image,self.callback)
        print ("<< Subscribed to topic /usb_cam/image_raw")
        
        
    def callback(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        if cv_image is not None :
            (rows,cols,channels) = cv_image.shape
            #--- Detect blobs
            cv_image, x, y, w, h = simple_detect_bbox(cv_image, "blue")

            # calculate delta x, y
            delta_x = ((x+w/2) - cols / 2)
            delta_y = -1 * ((y+h/2) - rows / 2)

            self.blob_point.x = delta_x
            self.blob_point.y = delta_y
            self.blob_pub.publish(self.blob_point)

            # visualize
            # center of object
            cv2.circle(cv_image, (int(x+w/2), int(y+h/2)), 7, (0, 0, 255), -1)
            # center of image
            cv2.circle(cv_image, (int(cols/2), int(rows/2)), 7, (255, 0, 0), -1)
            cv2.putText(cv_image, "({},{})".format(delta_x, delta_y), (x, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0,
                    (0, 0, 255))
            
            try:
                self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
            except CvBridgeError as e:
                print(e)
                    
            fps = 1.0/(time.time()-self._t0)
            self._t0 = time.time()
            

if __name__ == '__main__':
    
    rospy.init_node('blob_detector', anonymous=True)
    detector = SimpleColorDetector()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    
    cv2.destroyAllWindows()
