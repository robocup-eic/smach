import rospy

from interactive_markers.interactive_marker_server import *
from visualization_msgs.msg import *
from simple_environment_exporter._import import *
from simple_environment_exporter.Simple_Environment_eXporter import Simple_Environment_eXporter as sex
from copy import deepcopy

class yaml_maker:
    rospy.init_node("yaml_maker")
    # create an interactive marker server on the topic namespace simple_marker
    server = InteractiveMarkerServer("yaml_maker")
    isex = sex()

    def __init__(self,pkg:str="simple_environment_exporter"):
        self.pkg = pkg
        
        
        rospy.loginfo("Define init param:--------------")
        self.name = input("Name: ")
        self.file = RosPack().get_path(pkg)+"/save/"+self.name+".yaml"
        self.frame = input("Frame: ")
        mesh_folder = input("Mesh_Folder: /mesh/ ?")
        if mesh_folder:
            self.mesh_folder = mesh_folder
        else:
            self.mesh_folder = "/mesh/"
    def __make_Header(self,frame_id:str):
            header = Header()
            header.frame_id = frame_id
            header.stamp = rospy.Time.now()
            return header
    
    def create_mesh_marker(self,mesh:str):
        
        def mesh_marker(mesh_file:str):
            marker               = Marker()
            marker.type          = Marker.MESH_RESOURCE

            marker.scale.x       = 1
            marker.scale.y       = 1
            marker.scale.z       = 1
         
            marker.color.r       = 255/255
            marker.color.g       = 0
            marker.color.b       = 127/255
            marker.color.a       = 0.8

            marker.mesh_resource = mesh_file
            marker.mesh_use_embedded_materials = False
            return marker
        
        mesh_file = "package://"+self.pkg+self.mesh_folder+mesh
        marker = mesh_marker(mesh_file)
        marker.header = self.__make_Header(self.frame)
        return marker
    
    def create_box_marker(self,x,y,z):
        def box_marker(x,y,z):
            marker               = Marker()
            marker.type          = Marker.CUBE

            marker.scale.x       = x
            marker.scale.y       = y
            marker.scale.z       = z
         
            marker.color.r       = 255/255
            marker.color.g       = 0
            marker.color.b       = 127/255
            marker.color.a       = 0.8
            return marker
        
        marker = box_marker(x,y,z)
        marker.header = self.__make_Header(self.frame)
        return marker
        
    def processFeedback(self,feedback:InteractiveMarkerFeedback):
        self.posenow = feedback.pose


    def create_interactive_marker(self,mesh:str,name:str,sx,sy,sz):
        
        def marker_control(name,x,y,z,mode):
            control = InteractiveMarkerControl()
            control.name = name
            control.interaction_mode = mode
            control.orientation.x = x
            control.orientation.y = y
            control.orientation.z = z
            control.orientation.w = 1
            return control
        
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = self.frame
        int_marker.name = name
        int_marker.description = name

        obj_control = InteractiveMarkerControl()
        obj_control.always_visible = True
        if mesh == "no":
            obj_control.markers.append(self.create_box_marker(sx,sy,sz))
        else:
            obj_control.markers.append(self.create_mesh_marker(mesh))

        x_control = marker_control("x_control",1,0,0,InteractiveMarkerControl.MOVE_AXIS)
        y_control = marker_control("y_control",0,1,0,InteractiveMarkerControl.MOVE_AXIS)
        z_control = marker_control("z_control",0,0,1,InteractiveMarkerControl.MOVE_AXIS)
        rz_control = marker_control("rz_control",0,1,0,InteractiveMarkerControl.ROTATE_AXIS)




        int_marker.controls.append(obj_control)
        int_marker.controls.append(x_control)
        int_marker.controls.append(y_control)
        int_marker.controls.append(z_control)
        int_marker.controls.append(rz_control)

        # add the interactive marker to our collection &
        # tell the server to call processFeedback() when feedback arrives for it
        self.server.insert(int_marker, self.processFeedback)
        self.server.applyChanges()

    def run(self):
        olist = []
        dlist = []
        v_pub = self.isex.Visual_publisher
        rospy.loginfo("SECTION: ADD OBJECT BY INTERSCTIVE MARKER")
        while not rospy.is_shutdown():
            obj_count = len(olist)
            rospy.loginfo("object: "+str(obj_count))
            obj_id = input("object id: ")
            if input("have mesh [Y/n]") != "n":
                mesh = input("mesh stl : ")
                self.create_interactive_marker(mesh,obj_id,0,0,0)
                sx = 1
                sy = 1
                sz = 1
                typee = "mesh"
            else:
                sx = float(input("scale x: "))
                sy = float(input("scale y: "))
                sz = float(input("scale z: "))
                self.create_interactive_marker("no",obj_id,sx,sy,sz)
                typee = "box"
                
            rospy.loginfo("move marker")
            input("set location")
            try:
                pose:Pose = self.posenow
                s = True
            except:
                rospy.logerr("try to move marker!!")
                s = False

            if s == True:
                print("shape")
                print(pose)
                if typee == "mesh":

                    ddata = {"id":obj_id,
                                "type": typee,
                                "shape":{"x":sx,
                                        "y":sy,
                                        "z":sz},
                                "pose":{"position":{"x":pose.position.x,
                                                    "y":pose.position.y,
                                                    "z":pose.position.z},
                                        "orientation":{"x":pose.orientation.x,
                                                        "y":pose.orientation.y,
                                                        "z":pose.orientation.z,
                                                        "w":pose.orientation.w}},
                                "mesh_source":mesh}
                else:
                    ddata = {"id":obj_id,
                                "type": typee,
                                "shape":{"x":sx,
                                        "y":sy,
                                        "z":sz},
                                "pose":{"position":{"x":pose.position.x,
                                                    "y":pose.position.y,
                                                    "z":pose.position.z},
                                        "orientation":{"x":pose.orientation.x,
                                                        "y":pose.orientation.y,
                                                        "z":pose.orientation.z,
                                                        "w":pose.orientation.w}},
                                }
                        
                if input("are yoy sure? [y]") == "y":
                    emarker = self.isex.add_object_visual(self.isex.read_object_data(ddata),obj_count)
                    marker = deepcopy(emarker)
                    marker.header= self.__make_Header(self.frame)
                    olist.append(marker)
                    dlist.append(ddata)
                    

            self.server.clear()
            v_pub.publish(olist)

            if input("exit [y]: ") == "y":
                break

        print(olist)
        return dlist
    
    def save(self,dlist:list):
        flist = {"Name":self.name,"Frame":self.frame,"Mesh_Folder":self.mesh_folder,"Furniture":dlist}
        with open(self.file,"w") as stream:
            yaml.dump(flist,stream,default_flow_style=False)
        rospy.logwarn("Finishing making YAML")
        rospy.signal_shutdown("finishing")



if __name__=="__main__":
    
    a = yaml_maker()
    ol = a.run()
    a.save(ol)

    rospy.spin()