#!/usr/bin/python2

from dobot_api import dobot_api_dashboard, dobot_api_feedback, MyType
import time
import numpy as np

dobot_Enable = True

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


try:
    while dobot_Enable == True:
        # Call the JointMovJ directive
        
        client_feedback.MovJ(-72.97, 245.71, 94.13, 90.01, 5.01, 172.32)
        time.sleep(15)
        print("Factory position reached")
    
except KeyboardInterrupt:
    dobot_Enable = False
    client_dashboard.DisableRobot()
    
client_dashboard.close()
client_feedback.close()
print("END program")

