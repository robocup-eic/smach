
""" 

Guidelines for writing a state:

Consult example_smach_state.py for an ideal state
--------------------------------------------
--MUST HAVE: 

1. OUTCOME INFORMATION: Have to declear all
    outcomes=['out1','out2','loop','undo','fail'] # out2 is optional, a good practice should have loop & undo

2. INPUT/OUTPUT DATA: Have to declare all
    input_keys=['data1', 'data2'],
    output_keys=['data1', 'data2']

2. LOGGING: EACH STEP MUST HAVE LOGGING TO ROSPY
    rospy.loginfo('Executing state State1')
    rospy.loginfo(f'({name of class state}}): Executing..')
    rospy.loginfo(f'(AddPerson): {p.name} added to people_list. {p.__dict__}')

3. REMAPPING: ADD TO LIST FOR CONSTRUCTOR 
    # Will be added to the state machine by inputs and outputs
    remappings = {'data1': 'data1', 'data2': 'data2'}  # Use actual remappings

4. EXCEPTION HANDLING: return loop if exception or undo

5. FIX DATATYPE: NO DYNAMIC TYPE
    # Fix datatype to string, int, float, bool, list, dict
    log : bool = False # have default value
    # Raise exceptions if any entity parameter is not of type bool
    if not isinstance(intent, bool):
        raise ValueError("Argument 'intent' must be of type bool")
    if None then raise exception for variable CANNOT BE NONE

--------------------------------------------
--OPTIONAL:  

"""

import rospy
import smach
import nlp_client
import threading
from ratfin import *
import socket
import cv2
try:
    from custom_socket import CustomSocket
except:
    from core_cv.custom_socket import CustomSocket # when running from main
import os 
import os
import sys      
import time
sys.path.append("smach_task_2023/core_nlp/") # specific to cv folder
from emerStop import EmergencyStop, StopEnd

os.environ["QT_LOGGING_RULES"] = "*=false"

# # Model Smach States
class RegisterFace(smach.State):

    def __init__(self, 
                 log : bool = False,
                 nlp : bool = True,
                 timeout_tries: int = 0 # 0 means infinite tries
                 ):
        """ 
        input_keys=['name'],
        output_keys=['name']
        TemplateVersion 1.1.0 
        """
        self.log = log
        self.nlp = nlp
        
        # Raise exceptions if any entity parameter is not of type bool
        if not isinstance(log, bool):
            raise ValueError("Argument 'log' must be of type bool")
        if not isinstance(nlp, bool):
            raise ValueError("Argument 'nlp' must be of type bool")
        
        # Initialize the state
        smach.State.__init__(self, 
                             outcomes=['out1','undo'],
                            #  outcomes=['out1','out2','loop','undo','timeout'],
                             input_keys=['name'],
                             output_keys=['name'])
        
        # timout configuration, (don't change)
        if timeout_tries == 0:
            self.timeout_bool = False
        else:
            self.timeout_bool = True
        self.tries_counter = int(0) # counter for the number of tries
        self.timeout_tries = timeout_tries
    

    def execute(self, userdata):
        try:
            # Log the execution stage
            rospy.loginfo(f'(RegisterFace): Executing..')

            # Userdata verification
            rospy.loginfo(f'(RegisterFace): Checking userdata..')
            rospy.loginfo(f'(RegisterFace): using userdata.name: {userdata.name}.')

            # Do something
            if userdata.name == "":
                raise Exception("No name given")
            
            # print(userdata)
            try: 

                ## Register
                
                # Log the execution stage
                rospy.loginfo(f'(RegisterFace): Camera init...')

                # Setup cap
                cap = cv2.VideoCapture(0)
                cap.set(4, 480)
                cap.set(3, 640)

                # Setup socket
                host = socket.gethostname()
                port = 12304
                c = CustomSocket(host, port)
                c.clientConnect()

                # Log the execution stage
                rospy.loginfo(f'(RegisterFace): looping...')
                if self.nlp:
                        time.sleep(1.5)
                        nlp_client.speak(f"capturing in 3        2          1")
                while cap.isOpened():

                    ret, frame = cap.read()
                    cv2.imshow("client_cam", frame)

                    res = c.register(frame, userdata.name )
                    # {'feedback': 'Game registered'}
                    if res != {}:
                        
                        # Log thee execution stage
                        rospy.loginfo(f'(RegisterFace): {userdata.name} registered')
                        if self.nlp:
                            nlp_client.speak(f"{userdata.name} face registered")
                        
                        if self.log:
                            print(res)
                        rospy.loginfo(f'(RegisterFace): exiting...')
                        cap.release()
                    break


                cv2.destroyAllWindows()
            except:
                cap.release()
                return "undo"

            # if something goes wrong, raise exception
            if False:
                raise Exception(
                    "(RegisterFace): No attribute detected in the timeout period")
            
            return "out1"
        except Exception as e:
            printclr(e, "red")
            return "undo"


# Model Smach States
class DetectFace(smach.State):
    """ 
    TemplateVersion 1.1.0 
    """
    def __init__(self, 
                 log : bool = False,
                 timeout_tries: int = 0 # 0 means infinite tries
                 ):
        self.log = log
        
        # Raise exceptions if any entity parameter is not of type bool
        if not isinstance(log, bool):
            raise ValueError("Argument 'log' must be of type bool")
        
        # Initialize the state
        smach.State.__init__(self, 
                             outcomes=['out1','undo'],
                            #  outcomes=['out1','out2','loop','undo','timeout'],
                             input_keys=['name'],
                             output_keys=['name'])
        
        # timout configuration, (don't change)
        if timeout_tries == 0:
            self.timeout_bool = False
        else:
            self.timeout_bool = True
        self.tries_counter = int(0) # counter for the number of tries
        self.timeout_tries = timeout_tries


    def execute(self, userdata):
        try:
            # Receive and return list of names

            # Log the execution stage
            rospy.loginfo(f'(DetectFace): Camera init...')

            ## Setup cap 
            cap = cv2.VideoCapture(0)
            cap.set(4, 480)
            cap.set(3, 640)

            # Setup socket
            host = socket.gethostname()
            port = 12304
            c = CustomSocket(host, port)
            c.clientConnect()

            # Log the execution stage
            rospy.loginfo(f'(DetectFace): Entering Loop..')
            while cap.isOpened():
                # Read from Camera
                ret, frame = cap.read()
                cv2.imshow("client_cam", frame)
                
                # Post to Socket
                res = c.detect(frame)
                # {'name': 'Game'}
                # if detect a known person

                #TODO: Add the ability to add the person to be found 
                if res != {}:

                    name = res["name"]
                    # Log the execution stage
                    rospy.loginfo(f'(DetectFace): Found a person named {name}')
                    if self.nlp:
                        nlp_client.speak(f"Found {name}")
                    if self.log:
                            print(res)

                    cap.release()
                    # Log the execution stage
                    rospy.loginfo(f'(RegisterFace): exiting...')
                    
                    return "out1"

                key = cv2.waitKey(1)
                if key == ord("q"):
                    cap.release()
                    return "out1"

            cv2.destroyAllWindows()
        except Exception as e:
            print(e)
            cap.release()
            return "undo"

def main():
    log = False
    # Initialize the node
    NODE_NAME = "smach_cv_face_recognition"
    rospy.init_node(NODE_NAME)
    
    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['out1'])

    # Declear Variables for top-state
    sm.userdata.name = ""

    with sm:
        smach.StateMachine.add('REGISTER_FACE',
                            RegisterFace(log=log),
                            remapping={'name': 'name'},
                            transitions={'out1': 'DETECT_FACE',
                                         'undo': 'out1',}
                                        )
        smach.StateMachine.add('DETECT_FACE',
                            DetectFace(log=log),
                            remapping={'name': 'name'},
                            transitions={'out1': 'out1',
                                         'undo': 'out1',}
                                        )
        # smach.StateMachine.add('STOP_END',
        #                     StopEnd(),
        #                     remapping={'stop': 'stop'},
        #                     transitions={'out1': 'out1'}
        #                                 )

    # Create a thread to execute the smach container
    # Execute SMACH plan
    smach_thread = threading.Thread(target=sm.execute)
    smach_thread.start()

    


    # es = EmergencyStop()
    # es_thread = threading.Thread(target=es.execute)
    # es_thread.start()
    # es.emer_stop_handler(sm.userdata)

    
if __name__ == '__main__':
    main()




