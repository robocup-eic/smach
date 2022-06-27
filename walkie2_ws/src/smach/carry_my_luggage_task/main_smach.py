#!/usr/bin/env python

from turtle import st
import roslib
import rospy
import smach
import smach_ros
import time

# import for speed-to-text
from flask import Flask, request
import threading

# import for text-to-speech
import requests
import json
from nlp_server import SpeechToText
import time

class Standby(smach.State):
    def __init__(self):
        rospy.loginfo('Initiating state Standby')
        smach.State.__init__(self, outcomes=['continue_follow'])
        # self.follow_command = True
        # self.request_command = False
        global stt
        self.stt = stt
        
    def execute(self, userdata):
        rospy.loginfo('Executing state Standby')
        # if self.follow_command == True:    
        #     return 'continue_follow'
        # if self.request_command == True:
        #     return 'continue_request_luggage'
        start_time = 0
        while True:
            if self.stt.body is not None:
                print(self.stt.body)
                if self.stt.body["intent"] == "follow_people": # waiting for "follow me" command
                    self.stt.body["intent"] = None
                    return "continue_follow"

            if time.time() - start_time > 10:
                speak("Please put your bag on my arm")
                start_time = time.time()

            time.sleep(0.01)

def speak(text) :
    try :
        url = 'http://localhost:5003/tts'
        x = requests.post(url, json={'text':text})
        return x
    except :
        print("error to connect speak api.")

class Request_luggage(smach.State):
    def __init__(self):
        rospy.loginfo('Initiating state Request_luggage')
        smach.State.__init__(self, outcomes=['continue_standby'])
        # self.request = True

    def execute(self, userdata):
        rospy.loginfo('Executing state Request_luggage')
        # if self.request == True:
        #     return 'cotinue_standby'
        print('Please put your bag on my arm')
        d = {"text" : "Please put your bag on my arm"}
        x = request.post('http://localhost:5003/tts', json=d)
        time.sleep(10)
        return 'continue_standby'


class Ask_if_arrived(smach.State):
    def __init__(self):
        rospy.loginfo('Initiating state Ask_if_arrived')
        smach.State.__init__(self, outcomes=['continue_standby','continue_place_luggage'])
        # self.check = True
        global stt
        self.stt = stt

    def execute(self, userdata):
        rospy.loginfo('Executing state Ask_if_arrived')
        # if self.check == True:
        #     return 'cotinue_standby'
        # else:
        #     return 'continue_place_luggage'
        print('Are we arrived?')
        d = {"text" : "Are we arrived?"}
        x = request.post('http://localhost:5003/tts', json=d)
        
        while True:
            if self.stt.body["intent"] is not None:
                print(self.stt.body["intent"])

            if self.stt.body["intent"] == "Yes": # waiting for "follow me" command
                self.stt.body["intent"] = None
                return "continue_place_luggage"

            if self.stt.body["intent"] == "No": # waiting for "carry my luggage" command
                self.stt.body["intent"] = None
                return "continue_standby"

            time.sleep(0.01)

class Place_luggage(smach.State):
    def __init__(self):
        rospy.loginfo('Initiating state Place_luggage')
        smach.State.__init__(self, outcomes=['continue_standby'])
        # self.check = True

    def execute(self, userdata):
        rospy.loginfo('Executing state Place_luggage')
        # if self.check == True:
        #     return 'cotinue_standby'
        print('Please pick your bag from my arm')
        d = {"text" : "Please pick your bag from my arm"}
        x = request.post('http://localhost:5003/tts', json=d)
        time.sleep(10)
        return 'continue_standby'
        

class Stop_command(smach.State):
    def __init__(self):
        rospy.loginfo('Initiating state Stop_command')
        smach.State.__init__(self, outcomes=['continue_stop','continue_find_person'])
        self.check = True

    def execute(self, userdata):
        rospy.loginfo('Executing state Stop_command')
        if self.check == True:
            return 'continue_find_person'
        else:
            return 'continue_stop'


class Follow_person(smach.State):
    def __init__(self):
        rospy.loginfo('Initiating state Follow_person')
        smach.State.__init__(self, outcomes=['continue_stop','continue_find_person'])
        self.check = True

    def execute(self, userdata):
        rospy.loginfo('Executing state Follow_person')
        if self.check == True:
            return 'continue_find_person'
        else:
            return 'continue_stop'


class Get_bounding_box(smach.State):
    def __init__(self):
        rospy.loginfo('Initiating state Get_bounding_box')
        smach.State.__init__(self, outcomes=['continue_stop','continue_find_person'])
        self.has_box = True

    def execute(self, userdata):
        rospy.loginfo('Executing state Get_bounding_box')
        if self.has_box == True:
            return 'continue_find_person'
        else:
            return 'continue_stop'


class Rotate(smach.State):
    def __inti__(self):
        rospy.loginfo('Initiating state Rotate')
        smach.State.__init__(self, outcomes=['outcome'])

    def execute(self, userdata):
        rospy.loginfo('Executing state Rotate')
        return 'outcome'


class Check_bounding_box(smach.State):
    def __init__(self):
        rospy.loginfo('Initiating state Check_bounding_box')
        smach.State.__init__(self, outcomes=['continue_found_person','continue_stop_rotate'])
        self.has_box = False

    def execute(self, userdata):
        rospy.loginfo('Executing state Check_bounding_box')
        if self.has_box == True:
            return 'continue_found_person'
        else:
            return 'continue_stop_rotate'


if __name__ == '__main__':
    # initiate ROS node
    rospy.init_node('carry_my_luggage')

    # Flask nlp server
    stt = SpeechToText("nlp")
    t = threading.Thread(target = stt.run ,name="nlp")
    t.start()


    # start state machine
    sm = smach.StateMachine(outcomes=['Succeeded','Aborted'])
    with sm:
        smach.StateMachine.add('Standby',Standby(),
                                transitions={'continue_follow':'FOLLOW'})
        
        
        smach.StateMachine.add('Ask_if_arrived',Ask_if_arrived(),
                                transitions={'continue_standby':'Standby',
                                             'continue_place_luggage':'Place_luggage'})

        smach.StateMachine.add('Place_luggage',Place_luggage(),
                                transitions={'continue_standby':'Standby'})
        
        # Create sub smach state machine
        sm_follow = smach.Concurrence(outcomes=['Stop','Find_person'],
                                        default_outcome = 'Stop',
                                        outcome_map = {'Find_person':{'Stop_command':'continue_find_person','Follow_person':'continue_find_person','Get_bounding_box':'continue_find_person'}})        
        with sm_follow:
            smach.Concurrence.add('Stop_command',Stop_command())
            smach.Concurrence.add('Follow_person',Follow_person())
            smach.Concurrence.add('Get_bounding_box',Get_bounding_box())
        smach.StateMachine.add('FOLLOW', sm_follow, transitions={'Find_person':'FIND_PERSON','Stop':'Ask_if_arrived'})
    
        # Create sub smach state machine
        sm_find_person = smach.Concurrence(outcomes=['Stop_rotate','Found_person'], 
                                            default_outcome='Stop_rotate',
                                            outcome_map = {'Found_person':{'Check_bounding_box':'continue_found_person'}})
        with sm_find_person:
             smach.Concurrence.add('Rotate',Rotate())
             smach.Concurrence.add('Check_bounding_box',Check_bounding_box())
        smach.StateMachine.add('FIND_PERSON',sm_find_person, transitions={'Stop_rotate':'Aborted','Found_person':'FOLLOW'})

        # Set up                                                    
        sis = smach_ros.IntrospectionServer('Server_name',sm,'/PatterRoot')
        sis.start()
        
        outcome = sm.execute()

        rospy.spin()
        sis.stop()
    
