#!/usr/bin/env python
"""
Gets the position of the blob and it commands to steer the wheels

Subscribes to 
    /cr3_arm_command
    /cr3_gripper_command
    /blob/point_blob
    
Publish to
    /joint_states

Sevice client of
    /visual_servo_service

"""
import math
import time
import threading
import numpy as np
import copy

# ros library
import rospy
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from cr3_lowlevel_pkg.msg import JointCommand

# cr3 library
from dobot_api import dobot_api_dashboard, dobot_api_feedback, MyType

# service
from cr3_lowlevel_pkg.srv import VisualServoResponse, VisualServo

Kp = 0.05

def rad_to_deg(angle):
    return angle * 180/ math.pi

def arm_cb(data):
    global client_feedback
    rospy.loginfo("moving arm")
    com = [rad_to_deg(j) for j in list(data.joint_commands)]
    client_feedback.ServoJ(com[0], com[1], com[2], com[3], com[4], com[5])
    time.sleep(0.01) 

def gripper_cb(data):
    global client_feedback, cr3_joint8, cr3_joint9
    is_close = data.data
    # True is closing gripper
    if is_close:
        #close gripper
        client_dashboard.DO(2,0)
        client_dashboard.DO(1,1)
        cr3_joint8, cr3_joint9 = 0,0
        time.sleep(0.5)
    else:
        #open gripper 
        client_dashboard.DO(2,1)
        client_dashboard.DO(1,0)
        cr3_joint8, cr3_joint9 = math.pi/2, math.pi/2
        time.sleep(0.5)


def on_shutdown():
    global client_feedback, client_dashboard
    client_dashboard.DisableRobot()
    client_dashboard.close()
    client_feedback.close()

# The feedback information about port 30003 is displayed
def cr3_feedback():
    global client_feedback, dobot_enable
    global cr3_joint
    global cr3_endpoint
    while dobot_enable:
        time.sleep(0.05)
        all = client_feedback.socket_feedback.recv(10240)
        data = all[0:1440]
        a = np.frombuffer(data, dtype=MyType)
        try:
            if hex((a['test_value'][0]))[:13] == '0x123456789ab':
                print("============== Feed Back ===============")
                cr3_endpoint =  np.around(a['tool_vector_actual'], decimals=4)[0]
                print("cr3_endpoint: [x:{0}] , [y:{1}] , [z:{2}] , [rx:{3}] , [ry:{4}] , [rz:{5}]".format(cr3_endpoint[0],cr3_endpoint[1],cr3_endpoint[2],cr3_endpoint[3],cr3_endpoint[4],cr3_endpoint[5]))                            
                cr3_joint[:6] = np.around(a['q_actual'], decimals=4)[0]
                print("cr3_joint: [j1:{0}] , [j2:{1}] , [j3:{2}] , [j4:{3}] , [j5:{4}] , [j6:{5}]".format(cr3_joint[0],cr3_joint[1],cr3_joint[2],cr3_joint[3],cr3_joint[4],cr3_joint[5]))
                print("robot_mode: {}\n safety_mode: {}\nprogram_state: {}\n safety_status: {}".format(a['robot_mode'],a['safety_mode'], a['program_state'], a['safety_status']))
                print("========================================")

                # check robot alarm
                if int(a['robot_mode'][0]) == 9:
                    rospy.logerr("Some error with robot, clearing alarm")
                    client_dashboard.ClearError()
                    time.sleep(0.5)
                    client_dashboard.EnableRobot()
                    time.sleep(0.5)
                elif int(a['robot_mode'][0]) == 7:
                    rospy.loginfo("robot moving...")
                elif int(a['robot_mode'][0]) == 5:
                    rospy.loginfo("robot standby")

        except Exception as e:
            rospy.logerr(e)

def handle_visual_servo(req):
    robot = VisualServoServer()
    robot.is_operating = req.trigger
    rospy.loginfo("is_operating : {}".format(robot.is_operating))

    start_time = time.time()

    # timeout of 10 seconds
    while robot.is_operating and time.time() -  start_time < 10:
        try:
            robot.arm_command()
            # Delay execution to match rate

        except KeyboardInterrupt:
            dobot_enable = False
            break
        # using 20 hz
        time.sleep(0.05)

    return VisualServoResponse(not robot.is_operating)

class VisualServoServer():
    def __init__(self):
        global cr3_endpoint, cr3_joint
        self.blob_x         = 0.0
        self.blob_y         = 0.0
        self._time_detected = 0.0
        
        self.sub_center = rospy.Subscriber("/blob/point_blob", Point, self.update_position)
        rospy.loginfo("Subscribers set")
        
        self._time_steer        = 0
        self._steer_sign_prev   = 0
        self.current_pose = copy.copy(cr3_endpoint)
        self.start_joint = copy.copy(cr3_joint)

        self.is_operating, self.is_x_stop, self.is_y_stop = False, False, False
    @property
    def is_detected(self): return(time.time() - self._time_detected < 1.0)
        
    def update_position(self, message):
        self.blob_x = message.x
        self.blob_y = message.y
        self._time_detected = time.time()
        # rospy.loginfo("Ball detected: %.1f  %.1f "%(self.blob_x, self.blob_y))


    def arm_command(self):
        """
        +Z           
        |         +y   Camera frame
        |         |     o object
        |         |
        |         o---> +x
        |
        0------------------> +Y Robot Base Frame

        """
        global client_feedback, cr3_endpoint

        # rospy.loginfo("commanding for x,y = {},{}".format(self.blob_x, self.blob_y))
        x_tolerance = 10
        y_tolerance = 10

        if self.is_detected:

            # safty check the endpoint coordinates
            # x_eef, y_eef, z_eef, roll_eef, pitch_eef, yaw_eef = cr3_endpoint[0], cr3_endpoint[1], cr3_endpoint[2], cr3_endpoint[3], cr3_endpoint[4], cr3_endpoint[5]
            x_eef, y_eef, z_eef, roll_eef, pitch_eef, yaw_eef = self.current_pose[0], self.current_pose[1], self.current_pose[2], self.current_pose[3], self.current_pose[4], self.current_pose[5]
            rospy.loginfo("cr3_endpoint: [x:{0}] , [y:{1}] , [z:{2}]".format(x_eef, y_eef, z_eef))
            if not (-200 < y_eef < 200 and 100 < z_eef < 500 and -700 <x_eef< -400) :
                rospy.logerr("cr3 out of workspace! moving to {},{},{}".format(self.start_joint[0], self.start_joint[1], self.start_joint[2]))
                client_feedback.JointMovJ(self.start_joint[0], self.start_joint[1], self.start_joint[2], self.start_joint[3], self.start_joint[4], self.start_joint[5])
                time.sleep(10)
                self.current_pose = copy.copy(cr3_endpoint)
            else:
                # TODO use pid
                if self.blob_x > x_tolerance:
                    # rospy.loginfo("moving +x power {}".format(self.blob_x * Kp))
                    # y_eef += abs(self.blob_x * Kp)
                    rospy.loginfo("moving +x")
                    y_eef += 0.5
                elif self.blob_x < -1*x_tolerance:
                    # rospy.loginfo("moving -x power {}".format(self.blob_x * Kp))
                    # y_eef -= abs(self.blob_x * Kp)
                    rospy.loginfo("moving -x")
                    y_eef -= 0.5
                else:
                    rospy.loginfo("stop x")
                    self.is_x_stop = True

                if self.blob_y > y_tolerance:
                    # rospy.loginfo("moving +z power {}".format(self.blob_y * Kp))
                    # z_eef += abs(self.blob_y * Kp)
                    rospy.loginfo("moving +y")
                    z_eef += 0.5
                elif self.blob_y < -1*y_tolerance:
                    # rospy.loginfo("moving -z power {}".format(self.blob_y * Kp))
                    # z_eef -= abs(self.blob_y * Kp)
                    rospy.loginfo("moving -y")
                    z_eef -= 0.5
                else:
                    rospy.loginfo("stop y")
                    self.is_y_stop = True

                client_feedback.ServoP(x_eef, y_eef, z_eef, roll_eef, pitch_eef, yaw_eef)
                # update current_pose
                self.current_pose = x_eef, y_eef, z_eef, roll_eef, pitch_eef, yaw_eef
        else:
            rospy.loginfo("stop")

        # check are x and y axis stop, set is_operating to false
        self.is_operating = not (self.is_x_stop and self.is_y_stop)
        

            
if __name__ == "__main__":

    rospy.init_node('visual_servo_service', anonymous=True)
    rospy.Subscriber("/cr3_arm_command", JointCommand, arm_cb)
    rospy.Subscriber("/cr3_gripper_command", Bool, gripper_cb)
    s = rospy.Service("visual_servo_service", VisualServo, handle_visual_servo)
    pub = rospy.Publisher("/joint_states", JointState, queue_size=10)
    rate = rospy.Rate(20)
    rospy.on_shutdown(on_shutdown)

    # Enable threads on ports 29999 and 30003
    client_dashboard = dobot_api_dashboard('192.168.5.6', 29999)
    client_feedback = dobot_api_feedback('192.168.5.6', 30003)

    
    client_dashboard.DisableRobot()
    time.sleep(1)

    # Remove alarm
    client_dashboard.ClearError()
    time.sleep(0.5)

    # Description The upper function was enabled successfully
    client_dashboard.EnableRobot()
    time.sleep(0.5)
    # Select user and Tool coordinate system 0
    client_dashboard.User(0)
    client_dashboard.Tool(0)

    # initiate variable

    cr3_joint = [0,0,0,0,0,0,0,0]
    cr3_endpoint = [0,0,0,0,0,0]

    msg = JointState()
    msg.header.frame_id = ""
    msg.name = ["joint1","joint2","joint3","joint4","joint5","joint6", "joint8", "joint9"]

    dobot_enable = True

    # run feedback in Background
    t1 = threading.Thread(name="thread1", target=cr3_feedback)
    t1.start()
    time.sleep(1)
    
    # publish joint state
    while not rospy.is_shutdown():
        try:
            # Initialize the time of publishing
            msg.header.stamp = rospy.Time.now()
            # Joint angle values
            msg.position = list(map(lambda x: x*math.pi / 180, cr3_joint))
            # Publish message
            pub.publish(msg)
            # Increase sequence
            msg.header.seq += 1
            # Change angle value
            # Delay execution to match rate
            rate.sleep()

        except KeyboardInterrupt:
            dobot_enable = False
            break
