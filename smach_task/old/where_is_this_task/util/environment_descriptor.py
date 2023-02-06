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
        center_point = Point()
        # print(self.data_yaml)
        for data in self.data_yaml:
            if data["name"] == name:
                if data["shape"]== "rectangle":
                    xc1 = data["corner1"]["x"]
                    xc2 = data["corner2"]["x"]
                    xc3 = data["corner3"]["x"]
                    xc4 = data["corner4"]["x"]
                    yc1 = data["corner1"]["y"]
                    yc2 = data["corner2"]["y"]
                    yc3 = data["corner3"]["y"]
                    yc4 = data["corner4"]["y"]

                    center_point.x = (xc1+xc2+xc3+xc4)/4
                    center_point.y = (yc1+yc2+yc3+yc4)/4
                    center_point.z = 0
                
                elif data["shape"] == "circle":
                    center_point.x = data["position"]["x"]
                    center_point.y = data["position"]["y"]
                    center_point.z = data["position"]["z"]
                else:
                    return None
                    
        return center_point
    
    def out_of_areana(self,robot_pose):
        for data in self.data_yaml:
            if data["name"] == "arena":
                xc1 = data["corner1"]["x"]
                xc2 = data["corner2"]["x"]
                xc3 = data["corner3"]["x"]
                xc4 = data["corner4"]["x"]
                yc1 = data["corner1"]["y"]
                yc2 = data["corner2"]["y"]
                yc3 = data["corner3"]["y"]
                yc4 = data["corner4"]["y"]

                min_x = min((xc1,xc2,xc3,xc4))
                max_x = max((xc1,xc2,xc3,xc4))
                min_y = min((yc1,yc2,yc3,yc4))
                max_y = max((yc1,yc2,yc3,yc4))

                robot_x = robot_pose.position.x
                robot_y = robot_pose.position.y

                if (min_x <= robot_x <= max_x) and (min_y <= robot_y <= max_y):
                    return False
                else:
                    return True
    
    def out_of_area(self,robot_pose, area):
        for data in self.data_yaml:
            if data["name"] == area:
                xc1 = data["corner1"]["x"]
                xc2 = data["corner2"]["x"]
                xc3 = data["corner3"]["x"]
                xc4 = data["corner4"]["x"]
                yc1 = data["corner1"]["y"]
                yc2 = data["corner2"]["y"]
                yc3 = data["corner3"]["y"]
                yc4 = data["corner4"]["y"]

                min_x = min((xc1,xc2,xc3,xc4))
                max_x = max((xc1,xc2,xc3,xc4))
                min_y = min((yc1,yc2,yc3,yc4))
                max_y = max((yc1,yc2,yc3,yc4))

                robot_x = robot_pose.position.x
                robot_y = robot_pose.position.y

                if (min_x < robot_x <= max_x) and (min_y < robot_y <= max_y):
                    return False
                else:
                    return True

    def current_area(self,robot_pose):
        areas = ['livingroom_area','office_area','kitchen_area','bedroom_area']
        for area in areas:

            data = [d for d in self.data_yaml if d['name'] == area][0]
            
            xc1 = data["corner1"]["x"]
            xc2 = data["corner2"]["x"]
            xc3 = data["corner3"]["x"]
            xc4 = data["corner4"]["x"]
            yc1 = data["corner1"]["y"]
            yc2 = data["corner2"]["y"]
            yc3 = data["corner3"]["y"]
            yc4 = data["corner4"]["y"]

            min_x = min((xc1,xc2,xc3,xc4))
            max_x = max((xc1,xc2,xc3,xc4))
            min_y = min((yc1,yc2,yc3,yc4))
            max_y = max((yc1,yc2,yc3,yc4))

            robot_x = robot_pose.position.x
            robot_y = robot_pose.position.y

            if (min_x < robot_x <= max_x) and (min_y < robot_y <= max_y):
                return area

        return None


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
        point_marker.header.stamp = rospy.Time.now()
        point_marker.id = 99
        point_marker.type = Marker.POINTS
        point_marker.action = Marker.ADD
        point_marker.points = point_list
        point_marker.scale.x = 0.1
        point_marker.scale.y = 0.1
        point_marker.color.a = 1
        point_marker.color.r = 1
        point_marker.color.g = 0
        point_marker.color.b = 0

        start = rospy.Time.now()
        print(start)

        rospy.logwarn("please add marker topic:= /robot_point_visual to see robot_pose point -*60 SEC REMAIN*-")
        while True:

            if self.marker_pub.get_num_connections()>0:
                break

            if rospy.Time.now() - start >= rospy.Duration(60):
                break

        self.marker_pub.publish(point_marker)

    def readablename(self,unreadable_name):
        readable_name = ''
        nn = unreadable_name.split('_')
        for n in nn:
            readable_name += n + ' '
        return readable_name[:-1]

    def unreadablename(self,readable_name):
        unreadable_name = ''
        nn = readable_name.split(' ')
        for n in nn:
            unreadable_name += n + '_'
        return unreadable_name[:-1]
    
    def furlist(self, room):
        livingroom = ['couch_table', 'side_table', 'book_shelf', 'TV', 'sofa', 'houseplant']
        kitchen = ['dinner_table', 'kitchen_shelf', 'pantry', 'sink', 'fridge', 'washing_machine', 'kitchen_bin']
        bedroom = ['bed', 'small_shelf', 'cupboard', 'big_shelf' ]
        office = ['desk', 'office_shell', 'show_rack', 'bin' ]
        if room == 'livingroom_area':
            return livingroom
        elif room == 'kitchen_area':
            return kitchen
        elif room == 'bedroom_area':
            return bedroom
        elif room == 'office_area':
            return office
        elif room == 'all':
            return livingroom + kitchen + bedroom + office
        else:
            return []

    def get_room_path(self, current_room, target_room):
        room_dict = {'livingroom':['kitchen','office','bedroom'],'kitchen':['bedroom','livingroom','office'], 'office':['bedroom','livingroom','kitchen'],'bedroom':['office','kitchen','livingroom']}
        room_path = [current_room]
        tmp = current_room
        # print(room_path)

        if target_room == current_room:
            return None

        while target_room not in room_dict[tmp][:2]:
            tmp = np.random.choice(room_dict[tmp][:2])
            # print(tmp)
            room_path.append(tmp)

        room_path.append(target_room)
        return room_path

    def get_exit_pos(self, room):
        room_data = [d for d in self.data_yaml if d['name']==room][0]
        result = Pose()
        result.position.x = room_data['position']['x']
        result.position.y = room_data['position']['y']
        result.position.z = room_data['position']['z']

        return result

    def get_closest_fur(self, pose, room=None):

        if room:
            fur_list = self.furlist(room)
        else:
            fur_list = self.furlist(self.current_area(pose)[:-5])

        fur_points = [self.get_center_point(fur) for fur in fur_list]
        dist = [math.sqrt((pose.position.x-p.x)**2+(pose.position.y-p.y)**2) for p in fur_points] 

        dist_name = [(n, d, p) for n,d, p in zip(fur_list, dist, fur_points)]
        dist_name = sorted(dist_name, key=lambda x: x[1])

        return dist_name[0]


    def get_directions(self, current_pose, furniture):

        
        current_room = self.current_area(current_pose)
        fur_exits = []

        p = Pose()
        p.position = self.get_center_point(furniture)
        target_room = self.current_area(p)

        room_path = self.get_room_path(current_room[:-5], target_room[:-5])
        prev_room = current_room[:-5]
        
        all_names = [d['name'] for d in self.data_yaml]

        for room in room_path[1:]:
            exit = '{}_2_{}'.format(prev_room, room)
            if not exit in all_names:
                exit = '{}_2_{}'.format(room, prev_room)

            fur_exit = self.get_closest_fur(self.get_exit_pos(exit), room=prev_room)
            fur_exits.append((prev_room))

            prev_room = room

            new_list = [i for i in fur_exits]
            new_list.append(target_room[:-5])

        return new_list

#!/usr/bin/env python
import yaml
import rospy
from geometry_msgs.msg import Pose, Point
from visualization_msgs.msg import Marker
from move_base_msgs.msg import *
import numpy as np
import math

if __name__=='__main__':
    ed = EnvironmentDescriptor("/home/arch/smach/smach_task/config/fur_data_onsite.yaml")
    test = Pose()
    test.position.x, test.position.y, test.position.z = 3.67554497719, 5.29867649078, 0.00
    path = ed.get_directions(test, 'sink')
    print(path)
    