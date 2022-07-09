#!/usr/bin/env python
import yaml
import rospy
from geometry_msgs.msg import Pose, Point
from visualization_msgs.msg import Marker

class EnvironmentDescriptor:
    def __init__(self, yaml_path = "../../config/fur_data.yaml"):
        self.yaml_path = yaml_path
        self.data_yaml = self.read_yaml()
        # for visualization
        self.marker_pub = rospy.Publisher('robot_point_visual', Marker , queue_size=1)

    def read_yaml(self):
        with open(self.yaml_path, "r") as f:
            try:
                yaml_data = yaml.safe_load(f)
                yaml_data.remove(None)
                return yaml_data
            except yaml.YAMLError as exc:
                print(exc)
                return None
    def get_robot_pose(self, name):
        for data in self.data_yaml:
            if data["name"] == name:
                robot_pose = Pose()
                robot_pose.position.x = data["robot_pose"]["position"]["x"]
                robot_pose.position.y = data["robot_pose"]["position"]["y"]
                robot_pose.position.z = data["robot_pose"]["position"]["z"]
                robot_pose.orientation.x = data["robot_pose"]["orientation"]["x"]
                robot_pose.orientation.y = data["robot_pose"]["orientation"]["y"]
                robot_pose.orientation.z = data["robot_pose"]["orientation"]["z"]
                robot_pose.orientation.w = data["robot_pose"]["orientation"]["w"]
                return robot_pose
    def get_robot_pose(self, name):
        for data in self.data_yaml:
            if data["name"] == name:
                robot_pose = Pose()
                robot_pose.position.x = data["robot_pose"]["position"]["x"]
                robot_pose.position.y = data["robot_pose"]["position"]["y"]
                robot_pose.position.z = data["robot_pose"]["position"]["z"]
                robot_pose.orientation.x = data["robot_pose"]["orientation"]["x"]
                robot_pose.orientation.y = data["robot_pose"]["orientation"]["y"]
                robot_pose.orientation.z = data["robot_pose"]["orientation"]["z"]
                robot_pose.orientation.w = data["robot_pose"]["orientation"]["w"]
                return robot_pose
    def get_chair_poses(self):
        chair_poses = []
        for data in self.data_yaml:
            if "chair" in data["name"]:
                chair_pose = Pose()
                chair_pose.position.x = data["position"]["x"]
                chair_pose.position.y = data["position"]["y"]
                chair_pose.position.z = data["position"]["z"]
                chair_pose.orientation.x = 0
                chair_pose.orientation.y = 0
                chair_pose.orientation.z = 0
                chair_pose.orientation.w = 1
                chair_poses.append(chair_pose)
        return chair_poses
    def get_corner_list(self, name):
        corner_list = []
        for data in self.data_yaml:
            if data["name"] == name:
                for i in range(4):
                    corner_list.append(data["corner{}".format(i+1)])
        return corner_list
    def get_height(self, name):
        for data in self.data_yaml:
            if data["name"] == name:
                return data["height"]
    def get_center_point(self, name):
        for data in self.data_yaml:
            if data["name"] == name:
                center_point = Point()
                center_point.x = data["position"]["x"]
                center_point.y = data["position"]["y"]
                center_point.z = data["position"]["z"]
                return center_point
    
    def visual_robotpoint(self):
        point_list = []
        # name_list = []
        for data in self.data_yaml:
            if data.has_key("robot_pose"):
                robot_pose = Point()
                robot_pose.x = data["robot_pose"]["position"]["x"]
                robot_pose.y = data["robot_pose"]["position"]["y"]
                robot_pose.z = data["robot_pose"]["position"]["z"]
                point_list.append(robot_pose)
                # name_list.append(data["name"])
            

        point_marker = Marker()
        point_marker.header.frame_id = "map"
        point_marker.header.stamp = rospy.Time().now()
        point_marker.id = 99
        point_marker.type = Marker.POINTS
        point_marker.action = Marker.ADD
        point_marker.points = point_list
        point_marker.scale.x = 0.1
        point_marker.scale.y = 0.1
        point_marker.color.a = 1
        point_marker.color.r = 0
        point_marker.color.g = 1
        point_marker.color.b = 1

        def wait_for_sub():
            while True:
                if self.marker_pub.get_num_connections()>0:
                    break

        wait_for_sub() 
        self.marker_pub.publish(point_marker)

if __name__ == "__main__":
    ed = EnvironmentDescriptor("../../config/fur_data.yaml")
    print(ed.get_robot_pose("table1"))
    print(ed.get_chair_poses())