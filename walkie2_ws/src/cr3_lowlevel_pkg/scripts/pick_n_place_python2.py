#!/usr/bin/python


from itertools import count
from dobot_api import dobot_api_dashboard, dobot_api_feedback, MyType
import time
        

def pick():
        #open gripper
        client_dashboard.DO(2,1)
        client_dashboard.DO(1,0)
        time.sleep(3)
        # Call the JointMovJ directive     
        client_feedback.RelMovL(0, 0, -145)
        time.sleep(3) 
        #close gripper
        client_dashboard.DO(2,0)
        client_dashboard.DO(1,1)
        time.sleep(3)
        client_feedback.RelMovL(0, 0, 145)
        time.sleep(3)

def place():
        #move relative to y 
        client_feedback.RelMovL(0,100,0)
        time.sleep(3)
        #move relative to Z down  
        client_feedback.RelMovL(0,0,-145)
        time.sleep(3)
        #open gripper 
        client_dashboard.DO(2,1)
        client_dashboard.DO(1,0)
        time.sleep(3)
        #move relative to z up   
        client_feedback.RelMovL(0,0,145)
        time.sleep(3) 
        #close gripper
        client_dashboard.DO(2,0)
        client_dashboard.DO(1,1)
        time.sleep(3)

def set_init_pos():
        client_feedback.JointMovJ(0.39,-0.19,89.83,0.51,-90.42,-44.89)
        time.sleep(3)

def place():
        #move relative to y 
        client_feedback.RelMovL(0,100,0)
        time.sleep(3)
        #move relative to Z down  
        client_feedback.RelMovL(0,0,-150)
        time.sleep(3)
        #open gripper 
        client_dashboard.DO(2,1)
        client_dashboard.DO(1,0)
        time.sleep(3)
        #move relative to z up   
        client_feedback.RelMovL(0,0,150)
        time.sleep(3) 
        #close gripper
        client_dashboard.DO(2,0)
        client_dashboard.DO(1,1)
        time.sleep(3)

def set_init_pos():
    client_feedback.JointMovJ(0.39,-0.19,89.83,0.51,-90.42,-44.89)
    time.sleep(3)
        
def move():
    set_init_pos()   
    pick()
    place()
                
    client_dashboard.close()
    client_feedback.close()
    print("END program")


if __name__ == "__main__":
    
    dobot_Enable = True
    start = time.time()

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

    LOOPING = 2
    count = 0

    while count < LOOPING:

        try:

            set_init_pos()   
            pick()
            place()
            count += 1
        except KeyboardInterrupt:
            break

    client_dashboard.close()
    client_feedback.close()
    print("END program")






