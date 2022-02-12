#!/usr/bin/env python

import roslib
import rospy
import smach
import smach_ros
import time
import threading
import cv2
import requests
import socket
from sensor_msgs.msg import Image
from custom_socket import CustomSocket
from cv_bridge import CvBridge
import matplotlib.pyplot as plt
from std_msgs.msg import Bool
from nlp_server import SpeechToText



class Standby(smach.State):
    def __init__(self):
        global stt
        smach.State.__init__(self, outcomes=['continue_follow','continue_pointing'])
        self.stt = stt
        rospy.loginfo('Initiate state Stnadby')
    def execute(self, userdata):
        rospy.loginfo('Executing state Standby')

        while True:
            # debug
            if self.stt.body["intent"] is not None:
                print(self.stt.body["intent"])

            if self.stt.body["intent"] == "follow_people":
                self.stt.body["intent"] = None
                return "continue_follow"
        
            if self.stt.body["intent"] == "what_is_that":
                self.stt.body["intent"] = None
                return "continue_pointing"

            time.sleep(0.01)


class Stop_command(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['continue_stop','continue_find_person'])
        self.check = True
        rospy.loginfo('Initiate state Stop_command')
    def execute(self, userdata):
        rospy.loginfo('Executing state Stop_command')
        if self.check == True:
            return 'continue_find_person'
        else:
            return 'continue_stop'
class Follow_person(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['continue_stop','continue_find_person'])
        self.check = True
        rospy.loginfo('Initiate state Follow_person')
    def execute(self, userdata):
        rospy.loginfo('Executing state Follow_person')
        if self.check == True:
            return 'continue_find_person'
        else:
            return 'continue_stop'
class Get_bounding_box(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['continue_stop','continue_find_person'])
        self.has_box = True
        rospy.loginfo('Initiate state Get_bounding_box')
    def execute(self, userdata):
        rospy.loginfo('Executing state Get_bounding_box')
        if self.has_box == True:
            return 'continue_find_person'
        else:
            return 'continue_stop'
class Rotate(smach.State):
    def __inti__(self):
        smach.State.__init__(self, outcomes=['outcome'])
        rospy.loginfo('Initiate state Rotate')
    def execute(self, userdata):
        rospy.loginfo('Executing state Rotate')
        return 'outcome'
class Check_bounding_box(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['continue_found_person','continue_stop_rotate'])
        self.has_box = False
        rospy.loginfo('Initiate state Check_bounding_box')
    def execute(self, userdata):
        rospy.loginfo('Executing state Check_bounding_box')
        if self.has_box == True:
            return 'continue_found_person'
        else:
            return 'continue_stop_rotate'

class Pose(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['continue_text_to_speech'],
                                    input_keys=['pose_input'],
                                    output_keys=['pose_output'])
        rospy.loginfo('Initiate state Pose')
        self.bridge = CvBridge()
        # initiate listObject and countFrame
        self.listObject=[]
        self.countFrame=0
        host = socket.gethostname()
        port = 10000
        self.c = CustomSocket(host,port)
        self.c.clientConnect()
    
    def callback(self, data):
        # connect to server
        
        
        # change subscribed data to numpy.array and save it as "frame"
        self.frame = self.bridge.imgmsg_to_cv2(data,'bgr8')
        self.frame = cv2.resize(self.frame, (640,480))
        
        # send frame to server and recieve the result      
        result = self.c.req(self.frame)
        # print(result)            
        # print(result['pointing_at'])
        
        # add countFrame counter and append the object to the list
        self.countFrame += 1
        if len(result['pointing_at']) != 0:
            self.listObject.append(str(result['pointing_at'][0]))
        
        # check list of objects
        print("listObject = ",self.listObject)
        
        # check number of frame
        print("counter frame = " + str(self.countFrame))
        # rospy.sleep(0.5)
            
    def execute(self, userdata):
        rospy.loginfo('Executing state Pose')
        self.listObject=[]
        self.countFrame=0

        self.sub = rospy.Subscriber("/camera/color/image_raw", Image, self.callback)
        
        # wait to capture 5 frame
        while self.countFrame < 40:
            pass
            # rospy.sleep(0.5)
        
        # stop subscribing /camera/color/image_raw
        self.sub.unregister()
        
        # if there is no object
        if len(self.listObject) == 0:
            userdata.pose_output = 'no_object'
            return 'continue_text_to_speech'
        # if there is an object, find most common object
        else:
            print('Most common object =', max(set(self.listObject), key=self.listObject.count))
            userdata.pose_output = max(set(self.listObject), key=self.listObject.count)
            return 'continue_text_to_speech'


class Text_to_speech(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['continue_succeeded'],
                                    input_keys=['tts_input'])
    def execute(self, userdata):
        rospy.loginfo('Executing state Text_to_speech')
        print("You are pointing at " + userdata.tts_input)
        # print(type(userdata.tts_input))
        if userdata.tts_input == 'no_object':
            # text to Speech
            d = {"text" : "Hello, you are not pointing at any object"}
            requests.post('http://localhost:5003/tts', json=d)
            # print('Hello, you are not pointing at any object')
        else:
            # text to speech
            d = {"text" : "Hello, you are pointing at "+userdata.tts_input}
            requests.post('http://localhost:5003/tts', json=d)
            # print('Hello, the object is OBJECT1')
        return 'continue_succeeded'



if __name__ == '__main__':
    rospy.init_node('what_is_that_smach')

    # flask nlp server
    stt = SpeechToText("nlp")
    t = threading.Thread(target = stt.run ,name="flask")
    t.start()

    sm = smach.StateMachine(outcomes=['Succeeded','Aborted'])
    with sm:
        smach.StateMachine.add('Standby',Standby(),
                        transitions={'continue_follow':'FOLLOW','continue_pointing':'OBJECT_DETECTION'})
        
        
        # Create sub smach state machine "FOLLOW"
        sm_follow = smach.Concurrence(outcomes=['Stop','Find_person'],
                                        default_outcome = 'Stop',
                                        outcome_map = {'Find_person':{'Stop_command':'continue_find_person','Follow_person':'continue_find_person','Get_bounding_box':'continue_find_person'}})
        with sm_follow:
            smach.Concurrence.add('Stop_command',Stop_command())
            smach.Concurrence.add('Follow_person',Follow_person())
            smach.Concurrence.add('Get_bounding_box',Get_bounding_box())
        smach.StateMachine.add('FOLLOW', sm_follow, transitions={'Find_person':'FIND_PERSON','Stop':'Aborted'})
        
        
        # Create sub smach state machine "FIND_PERSON"
        sm_find_person = smach.Concurrence(outcomes=['Stop_rotate','Found_person'],
                                            default_outcome='Stop_rotate',
                                            outcome_map = {'Found_person':{'Check_bounding_box':'continue_found_person'}})
        with sm_find_person:
             smach.Concurrence.add('Rotate',Rotate())
             smach.Concurrence.add('Check_bounding_box',Check_bounding_box())
        smach.StateMachine.add('FIND_PERSON',sm_find_person, transitions={'Stop_rotate':'Aborted','Found_person':'FOLLOW'})
        
        
        # Create sub smach state machine "OBJECT_DETECTION"
        sm_object_detection = smach.StateMachine(outcomes=['Succeeded'])
        with sm_object_detection:
            smach.StateMachine.add('Pose',Pose(),transitions={'continue_text_to_speech':'Text_to_speech'},
                                                   remapping={'pose_input':'object_is',
                                                                'pose_output':'object_is'}) 
            smach.StateMachine.add('Text_to_speech',Text_to_speech(),transitions={'continue_succeeded':'Succeeded'},
                                                                        remapping={'tts_input':'object_is'})
        smach.StateMachine.add('OBJECT_DETECTION',sm_object_detection,transitions={'Succeeded':'Standby'})
        
        
        # Set up
        sis = smach_ros.IntrospectionServer('Server_name',sm,'/PatterRoot')
        sis.start()
        outcome = sm.execute()
        rospy.spin()
        sis.stop()
