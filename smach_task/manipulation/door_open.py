#!/usr/bin/env python

import rospy
import smach
import smach_ros

import cv2
import pyrealsense2 as rs2
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError

class open_or_close(smach.State):
    def __init__(self):
        rospy.loginfo('Initiating state open_or_clos')
        smach.State.__init__(self, outcomes= ['continue_A'])

        self.intrinsics = None
        self.frame = None
        
    
    def execute(self, userdata):
        
        def rescale_pixel(x, y):
            x = int(x*self.intrinsics.width/1280)
            y = int(y*self.intrinsics.height/720)
            return (x, y)
            
        def info_callback(cameraInfo):
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

        def check_image_size_for_ros(frame):
            if frame.shape[0] != self.intrinsics.height and frame.shape[1] != self.intrinsics.width:
                frame = cv2.resize(frame, (self.intrinsics.width, self.intrinsics.height))
            return frame

        def rescale_pixel(x, y):
            x = int(x*self.intrinsics.width/1280)
            y = int(y*self.intrinsics.height/720)
            return (x, y)
        
        def depth_callback(frame):
            try:
                # if self.tf_stamp is not None:
                    # rospy.loginfo("publishing tf")
                    # self.tf_stamp.header.stamp = rospy.Time.now()
                    # self.pub_tf.publish(tf2_msgs.msg.TFMessage([self.tf_stamp]))

                self.depth_image = self.bridge.imgmsg_to_cv2(frame, frame.encoding)
            
            except CvBridgeError as e:
                print(e)
                return
            except ValueError as e:
                return
            pass

        #-----------------------------------------------------------------------------------------
        rospy.Subscriber(
            "/camera/aligned_depth_to_color/camera_info", CameraInfo, info_callback)
        self.depth_sub = rospy.Subscriber(
            "/camera/aligned_depth_to_color/image_raw", Image, depth_callback, queue_size=1, buff_size=52428800)
        
        x_pixel = int(self.intrinsics.width/2)
        y_pixel = int(self.intrinsics.height/2)

        (xcen_pixel, ycen_pixel) = rescale_pixel(x_pixel, y_pixel)

        self.depth_image = check_image_size_for_ros(self.depth_image)
        depth = self.depth_image[ycen_pixel,  xcen_pixel] # [y, x] for numpy array

        while not rospy.is_shutdown:
            centerframe_result = rs2.rs2_deproject_pixel_to_point(self.intrinsics, [xcen_pixel, ycen_pixel], depth)
            print(centerframe_result)
            if centerframe_result > 0.3:
                print("dsd")
                break
        
        return 'continue_A'

if __name__ == "__main__":
    rospy.init_node('check_door')
    sm = smach.StateMachine(outcomes=['SUCCEEDED'])

    with sm:
        smach.StateMachine.add('open_or_close', open_or_close(),
                                transitions= {'continue_A'  : 'SUCCEEDED'})

    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('server_name', sm, '/door')
    sis.start()

    # Execute the state machine
    outcome = sm.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()

        

