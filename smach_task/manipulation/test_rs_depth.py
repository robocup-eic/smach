#!/usr/bin/env python
import rospy
import pyrealsense2 as rs2

pipe = rs2.pipeline()


pipe.start()
dpt_frame = pipe.wait_for_frames().get_depth_frame().as_depth_frame()

width = dpt_frame.get_width()
height = dpt_frame.get_height()

raw_input()
pixel_distance_in_meters = dpt_frame.get_distance(width / 2, height / 2)
print(pixel_distance_in_meters)
