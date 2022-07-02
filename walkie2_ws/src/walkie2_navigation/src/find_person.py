from client.custom_socket import CustomSocket
import socket
import rospy
from geometry_msgs.msg      import Point
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String

# connect to server
host = '192.168.8.99'
port = 11000
personTrack = CustomSocket(host,port)
personTrack.clientConnect()

#CV2 related libraries
from cv_bridge import CvBridge, CvBridgeError
import cv2
import pyrealsense2.pyrealsense2 as rs2

class FindPerson:

    def __init__(self):
        rospy.loginfo('Initiating Find Person')

        # computer vision socker
        global personTrack
        self.personTrack = personTrack

        # image
        self.frame = None
        self.depth_image = None
        self.x_pixel = None
        self.y_pixel = None
        self.intrinsics = None
        self.bridge = CvBridge()
        self.person_id = -1
        self.detected=False

        self.rel_point = Point()
        self.abs_point = Point()

    def __call__(self):
        rospy.loginfo("")

        def check_image_size_for_cv(frame):
            if frame.shape[0] != 720 and frame.shape[1] != 1280:
                frame = cv2.resize(frame, (1280, 720))
            return frame

        def check_image_size_for_ros(frame):
            if frame.shape[0] != self.intrinsics.height and frame.shape[1] != self.intrinsics.width:
                frame = cv2.resize(frame, (self.intrinsics.width, self.intrinsics.height))
            return frame

        def rescale_pixel(x, y):
            x = int(x*self.intrinsics.width/1280)
            y = int(y*self.intrinsics.height/720)
            return (x, y)
            
        def detect():
            # print(self.frame)
            if self.frame is None:
                rospy.loginfo("no frame receive")
                return
            # scale image incase image size donot match cv server
            self.frame = check_image_size_for_cv(self.frame)

            # send frame to server and recieve the result
            result = self.personTrack.req(self.frame)
            if len(result["result"]) == 0:
                self.image_pub.publish(self.bridge.cv2_to_imgmsg(self.frame, "bgr8"))
                return

            if not self.intrinsics:
                rospy.loginfo("no camera intrinsics")
                return

            # rescale pixel incase pixel donot match
            self.frame = check_image_size_for_ros(self.frame)

            # not found person yet
            if self.person_id == -1:
                center_pixel_list = []
                for track in result["result"]:
                    self.depth_image = check_image_size_for_ros(self.depth_image)
                    x_pixel = int((track[2][0]+track[2][2])/2)
                    y_pixel = int((track[2][1]+track[2][3])/2)
                    x_pixel, y_pixel = rescale_pixel(x_pixel, y_pixel)
                    depth = self.depth_image[y_pixel, x_pixel] # numpy array
                    center_pixel_list.append((x_pixel, y_pixel, depth, track[0])) # (x, y, depth, perons_id)
                self.person_id = min(center_pixel_list, key=lambda x: x[2])[3] # get person id with min depth
                rospy.loginfo("Found person ID : {}".format(self.person_id)) 
            for track in result["result"]:
                # track : [id, class_name, [x1,y1,x2,y2]]
                # rospy.loginfo("Track ID: {} at {}".format(track[0],track[2]))
                if track[0] == self.person_id:
                    self.detected=True
                    self.x_pixel = int((track[2][0]+track[2][2])/2)
                    self.y_pixel = int((track[2][1]+track[2][3])/2)
                    self.x_pixel, self.y_pixel = rescale_pixel(self.x_pixel, self.y_pixel)
                    # visualize purpose
                    self.frame = cv2.circle(self.frame, (self.x_pixel, self.y_pixel), 5, (0, 255, 0), 2)
                    self.frame = cv2.rectangle(self.frame, rescale_pixel(track[2][0], track[2][1]), rescale_pixel(track[2][2], track[2][3]), (0, 255, 0), 2)
                    self.frame = cv2.putText(self.frame, str(self.person_id), rescale_pixel(track[2][0], track[2][1] + 10), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)

                else:
                    self.detected = False
            
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(self.frame, "bgr8"))

            # 3d pose
            # rescale pixel incase pixel donot match
            self.depth_image = check_image_size_for_ros(self.depth_image)
            pix = (self.x_pixel, self.y_pixel)
            depth = self.depth_image[pix[1], pix[0]] # [y, x] for numpy array
            rospy.loginfo("FOUND PERSON AT: ({},{}), Depth: {}".format(self.x_pixel, self.y_pixel, depth))
            result = rs2.rs2_deproject_pixel_to_point(self.intrinsics, [pix[0], pix[1]], depth)  # [x, y] for realsense lib
            
            # data from comuter vision realsense x is to the right, y is to the bottom, z is toward.
            x_coord, y_coord, z_coord = result[0]/1000, result[1]/1000, result[2]/1000

            if self.detected:

                rospy.loginfo("Publishing person at real-world : {}, {}, {}".format(x_coord, y_coord, z_coord))

                self.abs_point.x = x_coord
                self.abs_point.y = y_coord
                self.abs_point.z = z_coord

                self.rel_point.x = self.x_pixel - self.frame.shape[1]/2
                self.rel_point.y = self.y_pixel - self.frame.shape[0]/2

                self.abs_pub.publish(self.abs_point)
                self.rel_pub.publish(self.rel_point)

                return True
            
            return False


        rospy.loginfo('Start finding person')
        # run person detection constantly
        # wait untill the robot finds a person then continue to the next state
        # before continue to the next state count the number of person

        # start person tracker
        image_sub = rospy.Subscriber("/camera/color/image_raw", Image , self.yolo_callback)
        depth_sub = rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image , self.depth_callback)
        depth_info_sub = rospy.Subscriber("/camera/aligned_depth_to_color/camera_info", CameraInfo , self.info_callback)

        self.image_pub = rospy.Publisher("/blob/image_blob", Image, queue_size=1)
        self.rel_pub  = rospy.Publisher("/human/rel_coor", Point, queue_size=1)
        self.abs_pub  = rospy.Publisher("/human/abs_coor", Point, queue_size=1)
        
        rospy.sleep(0.5)

        try:
            while True:
               detect()

        except KeyboardInterrupt:
            image_sub.unregister()
            depth_sub.unregister()
            depth_info_sub.unregister()
            return 


    def yolo_callback(self, data):
        try:
            # change subscribed data to numpy.array and save it as "frame"
            self.frame = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        except CvBridgeError as e:
            print(e)

    def depth_callback(self, frame):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(frame, frame.encoding)
        except CvBridgeError as e:
            print(e)
            return
        except ValueError as e:
            return

    def info_callback(self, cameraInfo):
        try:
            if self.intrinsics:
                return
            self.intrinsics = rs2.intrinsics()
            self.intrinsics.width = cameraInfo.width
            self.intrinsics.height = cameraInfo.height
            self.intrinsics.ppx = cameraInfo.K[2]
            self.intrinsics.ppy = cameraInfo.K[5]
            self.intrinsics.fx = cameraInfo.K[0]
            self.intrinsics.fy = cameraInfo.K[4]
            if cameraInfo.distortion_model == 'plumb_bob':
                self.intrinsics.model = rs2.distortion.brown_conrady
            elif cameraInfo.distortion_model == 'equidistant':
                self.intrinsics.model = rs2.distortion.kannala_brandt4
            self.intrinsics.coeffs = [i for i in cameraInfo.D]
        except CvBridgeError as e:
            print(e)
            return
    
if __name__ == '__main__':

    rospy.init_node('receptionist_task')

    person_finder = FindPerson()
    person_finder()

    # rospy.spin()