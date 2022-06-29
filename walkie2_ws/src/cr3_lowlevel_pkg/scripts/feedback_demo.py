#!/usr/bin/python3

from dobot_api import dobot_api_dashboard, dobot_api_feedback, MyType
import _thread
import time
import numpy as np

dobot_Enable = True

# The feedback information about port 30003 is displayed
def CR3_feedback(threadName):
    global dobot_Enable,client_feedback
    while dobot_Enable == True:
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
            
    client_dashboard.DisableRobot()
    client_feedback.close()
    print('!!!!!! client_feedback END !!!!!!')

# Enable threads on ports 29999 and 30003
client_dashboard = dobot_api_dashboard('192.168.1.6', 29999)
client_feedback = dobot_api_feedback('192.168.1.6', 30003)


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

_thread.start_new_thread(CR3_feedback, ("Thread-1",))


try:
    while dobot_Enable == True:
        # Call the JointMovJ directive
        
        #client_feedback.MovL(-262.72,-133.91,373.85,179.24,0.64,85.17)
        time.sleep(5)
        #client_feedback.MovL(-454.27,148.69,284.94,-173.31,1.56,52.81)
        time.sleep(5)
        
        """
        client_feedback.JointMovJ(10,20,0,0,0,0)
        time.sleep(3)
        client_feedback.JointMovJ(-10,20,0,0,0,0)
        time.sleep(3)
        """
        
    
except KeyboardInterrupt:
    dobot_Enable = False
    client_dashboard.DisableRobot()
    
client_dashboard.close()
client_feedback.close()
print("END program")

