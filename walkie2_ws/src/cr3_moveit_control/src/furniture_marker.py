#!/usr/bin/env python

import rospy
import yaml
from visualization_msgs.msg import Marker , MarkerArray
from geometry_msgs.msg import Point

yaml_data = None 
with open("/home/tanas/robocup-manipulation/cr3_ws/src/cr3_moveit_control/config/fur_data.yaml", "r") as f:
    try:
        yaml_data = yaml.safe_load(f)
    except yaml.YAMLError as exc:
        print(exc)

rospy.init_node('furniture')
pub_line_min_dist = rospy.Publisher('furniture_marker', MarkerArray, queue_size=100)
rospy.loginfo('Publishing furniture marker')

marker_list = MarkerArray()    
marker_id = 1

for t in yaml_data:

    name = t['name']
    frame_id = t['frame_id']
    shape = t['shape']
    robot_pose = t['robo_pose'] #if circle obj doesn't use this MOVE IN RECT

    marker = Marker()
    marker.id = marker_id
    marker.header.frame_id = frame_id
    marker.header.stamp = rospy.Time.now()
    marker.action = Marker.ADD

    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0

    marker.color.r = 0
    marker.color.g = 0.5
    marker.color.b = 1.0
    marker.color.a = 1.0 #transparency

    if shape == "rectangle":

        corner1 = t['corner1']
        corner2 = t['corner2']
        corner3 = t['corner3']
        corner4 = t['corner4']
        height = t['heights']
        print("rect name: " + name)
        print("frame id: " + frame_id)
        print("height:" + str(height))
        print("robot pose" + str(robot_pose))
        print("corner1" + str(corner1))
        print("corner2" + str(corner2))
        print("corner3" + str(corner3))
        print("corner4" + str(corner4))

        marker.type = Marker.LINE_STRIP
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0

        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.points = []
        corner1_point = Point()
        corner1_point.x = corner1['x']
        # corner1_point.y = corner1['y']
        corner1_point.z = corner1['z']
        marker.points.append(corner1_point)
        corner2_point = Point()
        corner2_point.x = corner2['x']
        corner2_point.y = corner2['y']
        corner2_point.z = corner2['z']
        marker.points.append(corner2_point)
        corner3_point = Point()
        corner3_point.x = corner3['x']
        corner3_point.y = corner3['y']
        corner3_point.z = corner3['z']
        marker.points.append(corner3_point)
        corner4_point = Point()
        corner4_point.x = corner4['x']
        corner4_point.y = corner4['y']
        corner4_point.z = corner4['z']
        marker.points.append(corner4_point)
        marker.points.append(corner1_point)

    elif t['shape'] == "circle":

        radius = float(t['radius'])
        position = t['position']    
        print("circle name: " + name)
        print("frame id: " + frame_id)
        print("radius:" + str(radius))
        print("robot pose:" + str(robot_pose))
        print("position" + str(position))

        marker.type = Marker.CYLINDER
        marker.scale.x = radius * 2
        marker.scale.y = radius * 2  
        marker.scale.z = 1.0

        marker.pose.position.x = position['x']
        marker.pose.position.y = position['y']
        marker.pose.position.z = position['z']

    else:
        print("unidentify object")
                                                                                                                               
    marker_list.markers.append(marker)                                                                                                                                               
    marker_id = marker_id + 1

# Publish the Marker
while True:
    pub_line_min_dist.publish(marker_list)

rospy.spin()