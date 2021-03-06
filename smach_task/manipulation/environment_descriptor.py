import yaml
from geometry_msgs.msg import Pose, Point

class EnvironmentDescriptor:
    def __init__(self, yaml_path = "../config/fur_data.yaml"):
        self.yaml_path = yaml_path
        self.data_yaml = self.read_yaml()

    def read_yaml(self):
        with open(self.yaml_path, "r") as f:
            try:
                yaml_data = yaml.safe_load(f)
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

    def get_corner_list(self, name):
        corner_list = []
        for data in self.data_yaml:
            if data is None:
                continue
            if data["name"] == name:
                for i in range(4):
                    corner = Point()
                    corner.x = data["corner{}".format(i+1)]["x"]
                    corner.y = data["corner{}".format(i+1)]["y"]
                    corner.z = data["corner{}".format(i+1)]["z"]
                    corner_list.append(corner)
        return corner_list

    def get_height(self, name):
        for data in self.data_yaml:
            if data["name"] == name:
                return data["height"]

if __name__ == "__main__":
    ed = EnvironmentDescriptor("../../config/fur_data.yaml")
    # print(ed.data_yaml)
    # print(ed.get_height("table1"))

    print(ed.get_corner_list("table1"))