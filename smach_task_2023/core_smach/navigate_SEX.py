'''
Simple Environment eXporters
has ability to read/write yaml file for Pose msg's
'''
import yaml
from geometry_msgs.msg import Pose
from interactive_markers.interactive_marker_server import *
from visualization_msgs.msg import *
import rospkg
from simple_environment_exporter._import import *
from simple_environment_exporter.Simple_Environment_eXporter import Simple_Environment_eXporter as sex
from copy import deepcopy

# class Navi_SEX():
#     # create an interactive marker server on the topic namespace simple_marker
#     server = InteractiveMarkerServer("whatever the fuck this is")

#     def __init__(self,pkg:str="simple_environment_exporter"):
#         rospy.init_node('test')
#         self.pkg = pkg
#         self.mesh_folder = "/mesh/"
#         self.frame_id = "world"
#         self.frame = ""
#     def read_yaml(self, id) -> Pose:
#         '''
#         read yaml file from path, then convert the data inside into Pose msg
#         '''
#         with open('example_sex.yaml', 'r') as f:
#             data = yaml.load(f, Loader=yaml.Loader)

#         for x in data['Station']:
#             if x['id'] == id: # if id exist
#                 return x['pose']
#         return None

#     def write_yaml(self, path, data: Pose) -> None:
#         '''
#         write Pose msg into yaml file
#         '''
#         with open(path, 'w') as f:
#             yaml.dump(data, f)

#     # Function for the interactive marker server.
#     def __make_Header(self,frame_id:str):
#         header = Header()
#         header.frame_id = frame_id
#         header.stamp = rospy.Time.now()
#         return header

#     # Function for the interactive marker server.
#     def create_mesh_marker(self,mesh:str):

#         def mesh_marker(mesh_file:str):
#             marker               = Marker()
#             marker.type          = Marker.MESH_RESOURCE

#             marker.scale.x       = 1
#             marker.scale.y       = 1
#             marker.scale.z       = 1

#             marker.color.r       = 255/255
#             marker.color.g       = 0
#             marker.color.b       = 127/255
#             marker.color.a       = 0.8

#             marker.mesh_resource = mesh_file
#             marker.mesh_use_embedded_materials = False
#             return marker

#         mesh_file = "package://"+self.pkg+self.mesh_folder+mesh
#         marker = mesh_marker(mesh_file)
#         marker.header = self.__make_Header(self.frame_id)
#         return marker

#     def create_interactive_marker(self,mesh:str,name:str,sx,sy,sz) -> None:

#         def marker_control(name,x,y,z,mode):
#             control = InteractiveMarkerControl()
#             control.name = name
#             control.interaction_mode = mode
#             control.orientation.x = x
#             control.orientation.y = y
#             control.orientation.z = z
#             control.orientation.w = 1
#             return control

#         int_marker = InteractiveMarker()
#         int_marker.header.frame_id = "world"
#         int_marker.name = name
#         int_marker.description = name

#         obj_control = InteractiveMarkerControl()
#         obj_control.always_visible = True
#         # if mesh == "no":
#         #     obj_control.markers.append(self.create_box_marker(sx,sy,sz))
#         # else:

#         # Always will be mesh
#         obj_control.markers.append(self.create_mesh_marker(mesh))

#         x_control = marker_control("x_control",1,0,0,InteractiveMarkerControl.MOVE_AXIS)
#         y_control = marker_control("y_control",0,1,0,InteractiveMarkerControl.MOVE_AXIS)
#         z_control = marker_control("z_control",0,0,1,InteractiveMarkerControl.MOVE_AXIS)
#         rz_control = marker_control("rz_control",0,1,0,InteractiveMarkerControl.ROTATE_AXIS)

#         int_marker.controls.append(obj_control)
#         int_marker.controls.append(x_control)
#         int_marker.controls.append(y_control)
#         int_marker.controls.append(z_control)
#         int_marker.controls.append(rz_control)

#         # add the interactive marker to our collection &
#         # tell the server to call processFeedback() when feedback arrives for it
#         self.server.insert(int_marker, self.processFeedback)
#         self.server.applyChanges()


class Navi_SEX():
    def __init__(self, filepath):
        self.filepath = filepath
        with open('example_sex.yaml', 'r') as f:
            self.data = yaml.load(f, Loader=yaml.Loader)

    def read_yaml(self, id) -> Pose:
        for x in self.data['Station']:
            if x['id'] == id:  # if id exist
                return x['pose']
        return None

    def write_yaml(self, id: str, type: str, pose: Pose) -> None:
        self.data['Station'].append({'id': id, 'type': type, 'pose': pose})
        with open(self.filepath, 'w') as f:
            yaml.dump(self.data, f)


if __name__ == "__main__":
    # Don't forget to initialize node
    sex = Navi_SEX()

    r = sex.read_yaml('living room')
    # print(r)
    # sex.create_interactive_marker("no", "test", 1, 1, 1)

    # print(isinstance(r,Pose))

    # p = Pose(Point(1,2,3), Quaternion(1,2,3,4))
    # sex.write_yaml(path='test.yaml', data=p)
