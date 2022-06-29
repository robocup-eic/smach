#!/usr/bin/env python

# ros library
import rospy
from std_msgs.msg import Bool
from sensor_msgs.msg import JointState
from cr3_lowlevel_pkg.msg import JointCommand

# cr3 library
from dobot_api import dobot_api_dashboard, dobot_api_feedback, MyType
import time

# other library
import threading
import numpy as np
import math


def rad_to_deg(angle):
    return angle * 180 / math.pi


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
        # close gripper
        client_dashboard.DO(2, 0)
        client_dashboard.DO(1, 1)
        cr3_joint8, cr3_joint9 = 0, 0
        time.sleep(0.5)
    else:
        # open gripper
        client_dashboard.DO(2, 1)
        client_dashboard.DO(1, 0)
        cr3_joint8, cr3_joint9 = math.pi/2, math.pi/2
        time.sleep(0.5)


def on_shutdown():
    global client_feedback, client_dashboard
    client_dashboard.DisableRobot()
    client_dashboard.close()
    client_feedback.close()

# The feedback information about port 30003 is displayed


def CR3_feedback():
    global client_feedback, dobot_enable
    global cr3_joint1, cr3_joint2, cr3_joint3, cr3_joint4, cr3_joint5, cr3_joint6
    while dobot_enable:
        time.sleep(0.05)
        all = client_feedback.socket_feedback.recv(10240)
        data = all[0:1440]
        a = np.frombuffer(data, dtype=MyType)
        try:
            if hex((a['test_value'][0]))[:13] == '0x123456789ab':
                print("============== Feed Back ===============")
                CR3_endpoint = np.around(
                    a['tool_vector_actual'], decimals=4)[0]
                print("CR3_endpoint: [x:{0}] , [y:{1}] , [z:{2}] , [rx:{3}] , [ry:{4}] , [rz:{5}]".format(
                    CR3_endpoint[0], CR3_endpoint[1], CR3_endpoint[2], CR3_endpoint[3], CR3_endpoint[4], CR3_endpoint[5]))
                CR3_joint = np.around(a['q_actual'], decimals=4)[0]
                print("CR3_joint: [j1:{0}] , [j2:{1}] , [j3:{2}] , [j4:{3}] , [j5:{4}] , [j6:{5}]".format(
                    CR3_joint[0], CR3_joint[1], CR3_joint[2], CR3_joint[3], CR3_joint[4], CR3_joint[5]))
                print("robot_mode: {}\n safety_mode: {}\nprogram_state: {}\n safety_status: {}".format(
                    a['robot_mode'], a['safety_mode'], a['program_state'], a['safety_status']))
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
                    rospy.loginfo("robot standy")

                cr3_joint1 = CR3_joint[0]
                cr3_joint2 = CR3_joint[1]
                cr3_joint3 = CR3_joint[2]
                cr3_joint4 = CR3_joint[3]
                cr3_joint5 = CR3_joint[4]
                cr3_joint6 = CR3_joint[5]

        except Exception as e:
            rospy.logerr(e)


if __name__ == '__main__':

    rospy.init_node('cr3_controller', anonymous=True)
    rospy.Subscriber("/cr3_arm_command", JointCommand, arm_cb)
    rospy.Subscriber("/cr3_gripper_command", Bool, gripper_cb)
    pub = rospy.Publisher("/joint_states", JointState, queue_size=1000)
    rate = rospy.Rate(20)  # 10hz
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

    cr3_joint1 = 0
    cr3_joint2 = 0
    cr3_joint3 = 0
    cr3_joint4 = 0
    cr3_joint5 = 0
    cr3_joint6 = 0
    cr3_joint8 = 0
    cr3_joint9 = 0

    msg = JointState()
    msg.header.frame_id = ""
    msg.name = ["joint1", "joint2", "joint3", "joint4",
                "joint5", "joint6", "joint8", "joint9"]

    dobot_enable = True

    # run feedback in Background
    t1 = threading.Thread(name="thread1", target=CR3_feedback)
    t1.start()

    while not rospy.is_shutdown():
        try:
            joint1 = cr3_joint1*math.pi / 180
            joint2 = cr3_joint2*math.pi / 180
            joint3 = cr3_joint3*math.pi / 180
            joint4 = cr3_joint4*math.pi / 180
            joint5 = cr3_joint5*math.pi / 180
            joint6 = cr3_joint6*math.pi / 180
            joint8 = cr3_joint8*math.pi / 180
            joint9 = cr3_joint9*math.pi / 180
            # Initialize the time of publishing
            msg.header.stamp = rospy.Time.now()
            # Joint angle values
            msg.position = [joint1, joint2, joint3,
                            joint4, joint5, joint6, joint8, joint9]
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
