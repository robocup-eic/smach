#!/usr/bin/python

import math
from dobot_api import dobot_api_dashboard, dobot_api_feedback, MyType
import time
import numpy as np

dobot_Enable = True

def rad_to_deg(rad):
    return rad*180/math.pi


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

client_dashboard.AccL(10)
client_dashboard.SpeedL(10)

try:
    while dobot_Enable == True:
        # Call the JointMovJ directive
        client_dashboard.ClearError()
        time.sleep(0.5)

        # client_feedback.JointMovJ(0,0.0,0,0,0,0)
        
        # home walkie 1
        # client_feedback.JointMovJ(rad_to_deg(0), rad_to_deg(0), rad_to_deg(2.267), rad_to_deg(-2.267), rad_to_deg(-1.507), -45)
        
        # home walkie 2
        client_feedback.JointMovJ(rad_to_deg(0), rad_to_deg(0), rad_to_deg(2.267), rad_to_deg(0.875), rad_to_deg(1.507), rad_to_deg(2.355))

        print("setting home")

        time.sleep(2)
    
except KeyboardInterrupt:
    dobot_Enable = False
    client_dashboard.DisableRobot()
    
client_dashboard.close()
client_feedback.close()
print("END program")

