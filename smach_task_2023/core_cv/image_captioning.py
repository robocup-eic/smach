
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
import sys
sys.path.append('../')
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
import json
import time
import os
os.environ["QT_LOGGING_RULES"] = "*=false"

# from core_nlp.emerStop import listen_for_kill_command


# General CV Funcitons

def list_available_cam(max_n):
    list_cam = []
    for n in range(max_n):
        cap = cv2.VideoCapture(n)
        ret, _ = cap.read()

        if ret:
            list_cam.append(n)
        cap.release()
    
    if len(list_cam) == 1:
        return list_cam[0]
    else:
        print(list_cam)
        return int(input("Cam index: "))







# Model Smach States
class ImageCaption(smach.State):
    """ 
    TemplateVersion 1.1.0 
    >>> python image_captioning.py
    {
        'age': '20', 
        'gender': 'male', 
        'race': 'asian', 
        'hair color': 'brown', 
        'shirt color': 'white', 
        'glasses': 'no', 
        'answer': 'He is a man. He is asian. His apparent age is 20 years old. His race is asian. His hair color is brown. His shirt color is white. He is not wearing glasses. '}
    """

    def __init__(self, 
                 log : bool = False,
                 timeout_tries: int = 0, # 0 means infinite tries
                 nlp : bool = False
                 ):
        self.log = log
        
        # Raise exceptions if any entity parameter is not of type bool
        if not isinstance(log, bool):
            raise ValueError("Argument 'log' must be of type bool")
        if not isinstance(nlp, bool):
            raise ValueError("Argument 'nlp' must be of type bool")
        
        # Initialize the state
        smach.State.__init__(self, 
                             outcomes=['out1','undo'],
                            #  outcomes=['out1','out2','loop','undo','timeout'],
                             output_keys=['age','gender','race','hair_color','shirt_color','wearing_glasses'])
        
        # timout configuration, (don't change)
        if timeout_tries == 0:
            self.timeout_bool = False
        else:
            self.timeout_bool = True
        self.tries_counter = int(0) # counter for the number of tries
        self.timeout_tries = timeout_tries
        self.nlp = nlp

    def execute(self, userdata):
        try:
            # Log the execution stage
            rospy.loginfo(f'(ImageCaption): Executing..')

            # Userdata verification
            rospy.loginfo(f'(ImageCaption): Checking userdata..')
            
            #TODO handle userdata here

            try:
                # Log the execution stage
                rospy.loginfo(f'(ImageCaption): Cameria init...')

                # Setup CV Capture
                cap = cv2.VideoCapture(0)
                cap.set(4, 720)
                cap.set(3, 1280)

                # Setup socket
                host = socket.gethostname()
                port = 12303

                c = CustomSocket(host, port, log=True)
                c.clientConnect()

                # Log the execution stage
                rospy.loginfo(f'(ImageCaption): looping...')
                if self.nlp:
                        nlp_client.speak(f"SAY SOMETHING IDK BRO")

                while cap.isOpened():
                    
                    ret, frame = cap.read()
                    cv2.imshow("client_cam", frame)

                    if not ret:
                        print("Ignoring empty camera frame.")
                        continue

                   
                    res = c.req(frame)
                    # print(res)
                    if res != {}:
                        rospy.loginfo(f'(ImageCaption): Captioned a person')
                        if self.log:
                            print(res)
                        if res['age']:
                            userdata.age = res['age']
                        if res['gender']:
                            userdata.gender = res['gender']
                        if res['race']:
                            userdata.race = res['race']
                        if res['hair color']:
                            userdata.hair_color = res['hair color']
                        if res['shirt color']:
                            userdata.shirt_color = res['shirt color']
                        if res['glasses']:
                            userdata.wearing_glasses = res['glasses']
                        if res['answer'] and self.nlp:
                            
                            nlp_client.speak(res['answer'])
                        cap.release()
                    break
                cv2.destroyAllWindows()

            except Exception as e:
                printclr(e, "red")
                cap.release()
                return "undo"
            # if something goes wrong, raise exception
            if False:
                raise Exception(
                    "(ExampleState): No attribute detected in the timeout period")
            
            return "out1"
        except Exception as e:
            printclr(e, "red")
            return "undo"
        

    # def execute(self, userdata):
    #     try:
    #         # Log the execution stage
    #         rospy.loginfo(f'(ImageCaption): Executing..')

    #         # Userdata verification
    #         rospy.loginfo(f'(ImageCaption): Checking userdata..')

    #         # Do something
    #         print(userdata)
    #         try: 
    #             rospy.loginfo(f'(ImageCaption): Starting CV..')
                
    #             while self.cap.isOpened():
                    
    #                 ret, frame = self.cap.read()
    #                 if not ret:
    #                     print("Ignoring empty camera frame.")
    #                     continue

    #                 # Sent to sever
                    

    #                 # Show client Frame
    #                 cv2.imshow("client_cam", frame)

    #                 key = cv2.waitKey(1)
    #                 if key == ord("q"):
    #                     self.cap.release()
    #                 if key == ord("c"): # capture part
    #                     msg = self.c.req(frame)
    #                     print(msg)
    #                     cv2.waitKey()

    #             self.cap.release()
    #             cv2.destroyAllWindows()
    #         except:
    #             self.cap.release()
    #             self.cap.release()
    #             return "undo"


    #         # if something goes wrong, raise exception
    #         if False:
    #             raise Exception(
    #                 "(ExampleState): No attribute detected in the timeout period")
            
    #         return "out1"
    #     except Exception as e:
    #         printclr(e, "red")
    #         return "undo"



def main():
    # Initialize the node
    NODE_NAME = "smach_cv_image_captioning"
    rospy.init_node(NODE_NAME)
    
    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['out1'])

    # Declear Variables for top-state
    sm.userdata.name = ""

    with sm:
        smach.StateMachine.add('IMAGE_CAPTION',
                            ImageCaption(log=True),
                            remapping={'name': 'name'},
                            transitions={'out1': 'out1',
                                         'undo': 'out1',}
                                        )
        

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




