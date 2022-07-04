import cv2
import pyrealsense2
# from realsense_depth import *

# Initialize Camera Intel Realsense
dc = DepthCamera()
while True:
    ret, depth_frame, color_frame = dc.get_frame()
    # Show distance for a specific point
    point = (400,300)
    distance = depth_frame[point[1], point[0]]
    print(distance)