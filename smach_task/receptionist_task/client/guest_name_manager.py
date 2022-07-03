import yaml

class GuestNameManager:
    def __init__(self, yaml_path = "../../config/fur_data.yaml"):
        self.yaml_path = yaml_path
        self.data_yaml = self.read_yaml()

    def read_yaml(self):
        with open(self.yaml_path, "r") as f:
            try:
                yaml_data = yaml.safe_load(f)
                print(yaml_data)
                return yaml_data
            except yaml.YAMLError as exc:
                print(exc)
                return None

    def get_guest_name(self,role):
        for data in self.data_yaml:
            if data["role"] == role:
                return data["name"]

    def get_guest_fav_drink(self,role):
        for data in self.data_yaml:
            if data["role"] == role:
                return data["fav_drink"]

if __name__ == "__main__":
    ed = GuestNameManager("../../config/receptionist_database.yaml")
    print(ed.get_guest_fav_drink("guest_1"))