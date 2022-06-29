#!/usr/bin/python

import rospy
from sensor_msgs.msg import JointState
import math
from dobot_api import dobot_api_dashboard, dobot_api_feedback, MyType
import thread
import time
import numpy as np

dobot_Enable = True
cr3_joint1 = 0
cr3_joint2 = 0
cr3_joint3 = 0
cr3_joint4 = 0
cr3_joint5 = 0
cr3_joint6 = 0

# Initialize the node
rospy.init_node("joint_publish_gui")
# Publisher object
publisherObject = rospy.Publisher("/joint_states", JointState, queue_size=10)

CR3_ip = rospy.get_param('~ip_cr3') # get parameter from launch agrument

# Rate controller
rateController = rospy.Rate(100)


# Enable threads on ports 29999 and 30003
client_dashboard = dobot_api_dashboard(CR3_ip, 29999)
client_feedback = dobot_api_feedback(CR3_ip, 30003)

# The feedback information about port 30003 is displayed
def CR3_feedback():
    time.sleep(0.05)
    all = client_feedback.socket_feedback.recv(10240)
    data = all[0:1440]
    a = np.frombuffer(data, dtype=MyType)
    try:
        if hex((a['test_value'][0])) == '0x123456789abcdef':
            print("============== Feed Back ===============")
            CR3_endpoint =  np.around(a['Tool_vector_target'], decimals=4)[0]
            print("CR3_endpoint: [x:{0}] , [y:{1}] , [z:{2}] , [rx:{3}] , [ry:{4}] , [rz:{5}]".format(CR3_endpoint[0],CR3_endpoint[1],CR3_endpoint[2],CR3_endpoint[3],CR3_endpoint[4],CR3_endpoint[5]))                            
            CR3_joint = np.around(a['q_target'], decimals=4)[0]
            print("CR3_joint: [j1:{0}] , [j2:{1}] , [j3:{2}] , [j4:{3}] , [j5:{4}] , [j6:{5}]".format(CR3_joint[0],CR3_joint[1],CR3_joint[2],CR3_joint[3],CR3_joint[4],CR3_joint[5]))
            print("========================================")
    except:
        print("Some error with robot.")


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

thread.start_new_thread(CR3_feedback, ("Thread-1",))

# Variables
msg = JointState()
msg.header.frame_id = ""
msg.name = ["joint1","joint2","joint3","joint4","joint5","joint6"]       # Joint name (array assignment)

try:
    #client_dashboard.DisableRobot()
         # Main loop
    while(not rospy.is_shutdown()) and dobot_Enable == True:
        joint1 = cr3_joint1*math.pi / 180
        joint2 = cr3_joint2*math.pi / 180
        joint3 = cr3_joint3*math.pi / 180
        joint4 = cr3_joint4*math.pi / 180
        joint5 = cr3_joint5*math.pi / 180
        joint6 = cr3_joint6*math.pi / 180
        # Initialize the time of publishing
        msg.header.stamp = rospy.Time.now()
        # Joint angle values
        msg.position = [joint1, joint2, joint3, joint4, joint5, joint6]
        # Publish message
        publisherObject.publish(msg)
        # Increase sequence
        msg.header.seq += 1
        # Change angle value
        # Delay execution to match rate
        rateController.sleep()
    
except KeyboardInterrupt or rospy.ROSInternalException:
    dobot_Enable = False
    client_dashboard.DisableRobot()
    rospy.logfatal("Node crashed due to an internal exception")
        
    
client_dashboard.close()
client_feedback.close()
print("END program")
