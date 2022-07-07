import yaml
from geometry_msgs.msg import Pose

class GuestNameManager:
    def __init__(self, yaml_path):
        self.yaml_path = yaml_path
        self.data_yaml = self.read_yaml()

    def read_yaml(self):
        with open(self.yaml_path, "r") as f:
            try:
                yaml_data = yaml.safe_load(f)
                if None in yaml_data:
                    yaml_data.remove(None)
                return yaml_data
            except yaml.YAMLError as exc:
                print(exc)
                return None
                
    def write_yaml(self, data):
        with open(self.yaml_path, 'w') as file:
            yaml.dump(data, file, encoding=('utf-8'))

    def get_guest_name(self,role):
        for data in self.data_yaml:
            if data["role"] == role:
                return data["name"]

    def get_guest_fav_drink(self,role):
        for data in self.data_yaml:
            if data["role"] == role:
                return data["fav_drink"]

    def add_guest_name(self, role, name):
        for data in self.data_yaml:
            if data["role"] == role:
                # already have this guest
                return None
        guest = {"name":str(name), "role":role} # there are no this guest yet
        self.data_yaml.append(guest)
        # print(yaml.dump(self.data_yaml))
        self.write_yaml(self.data_yaml)

    def add_guest_fav_drink(self, role, drink):
        for i, data in enumerate(self.data_yaml):
            if data["role"] == role:
                data["fav_drink"] = str(drink)
                self.data_yaml[i] = data
        self.write_yaml(self.data_yaml)

    def add_guest_location(self, role, location):
        """
        add gueset location in Pose tf with map frame
        """
        for i, data in enumerate(self.data_yaml):
            if data["role"] == role:
                position = {"x": location.position.x, "y": location.position.y, "z":location.position.z}
                orientation = {"x": location.orientation.x, "y": location.orientation.y, "z": location.orientation.z, "w":location.orientation.w}
                data["location"] = {}
                data["location"]["position"] = position
                data["location"]["orientation"] = orientation
                self.data_yaml[i] = data
        self.write_yaml(self.data_yaml)

    def get_guest_location(self, role):
        for data in self.data_yaml:
            if data["role"] == role:
                location = Pose()
                location.position.x = data["location"]["position"]["x"]
                location.position.y = data["location"]["position"]["y"]
                location.position.z = data["location"]["position"]["z"]
                location.orientation.x = data["location"]["orientation"]["x"]
                location.orientation.y = data["location"]["orientation"]["y"]
                location.orientation.z = data["location"]["orientation"]["z"]
                location.orientation.w = data["location"]["orientation"]["w"]
                return location

    def reset(self):
        host = {}
        for data in self.data_yaml:
            if data["role"] == "host":
                host = data
                self.data_yaml = [host]
                self.write_yaml(self.data_yaml)
        
if __name__ == "__main__":
    gm = GuestNameManager("../../config/receptionist_database.yaml")
    gm.add_guest_name("guest_1","patter")
    # gm.add_guest_fav_drink("guest_3", "jack daniel")
    # print(gm.get_guest_fav_drink("guest_3"))

    person_pose = Pose()
    person_pose.position.x = 0.1
    person_pose.position.y = 0.2
    person_pose.position.z = 0.3
    person_pose.orientation.x = 0
    person_pose.orientation.y = 0
    person_pose.orientation.w = 0
    person_pose.orientation.w = 1
    # gm.add_guest_location("guest_1", person_pose)
    print(gm.get_guest_location("guest_1"))
    # gm.reset()