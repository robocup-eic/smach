#!/usr/bin/env python

import roslib
import rospy
import smach
import smach_ros

# import for text-to-speech
import requests
import json
from client.nlp_server import SpeechToText, speak
import time

class Start_signal(smach.State):
    def __init__(self):
        rospy.loginfo('Initiating Start_signal state')
        smach.State.__init__(self,outcomes=['continue_Standby'])
    def execute(self,userdata):
        rospy.loginfo('Executing Start_signal state')
        # Detect door opening
        return 'continue_Standby'


class Standby(smach.State):
    def __init__(self):
        rospy.loginfo('Initiating Standby state')
        smach.State.__init__(self,outcomes=['continue_Ask'])
        #global person_count 
    def execute(self,userdata):
        rospy.loginfo('Executing Standby state')
        global person_count
        # run person detection constantly
        # wait untill the robot finds a person then continue to the next state
        # before continue to the next state count the number of person
        person_count += 1
        print("Number of guest : ", person_count)
        return 'continue_Ask'


class Ask(smach.State):
    def __init__(self):
        rospy.loginfo('Initiating Ask state')
        smach.State.__init__(self,outcomes=['continue_Navigation'])
    def execute(self,userdata):
        rospy.loginfo('Executing Ask state')
        # ask the guest to register's his/her face to the robot
        # ask name and favorite drink
        # save name and favorite drink in dictionary
        
        speak("Please show your face to the robot's camera")
        # register face
        speak("What is your name?")
        # listening to the person and save his/her name to the file
        speak("What is your favorite drink?")
        # listening to the person and save his his/her fav_drink to the file

        return 'continue_Navigation'


class Navigation(smach.State):
    def __init__(self):
        rospy.loginfo('Initiating Navigation state')
        smach.State.__init__(self,outcomes=['continue_No_seat','continue_Seat'])
        self.case = 1
    def execute(self,userdata):
        rospy.loginfo('Executing Navigation state')
        # navigate to seat
        # detect available
        if self.case == 0:
            return 'continue_No_seat'
        else:
            return 'continue_Seat'


class No_seat(smach.State):
    def __init__(self):
        rospy.loginfo('Initiating No_seat state')
        smach.State.__init__(self,outcomes=['continue_Introduce_guest'])
    def execute(self,userdata):
        rospy.loginfo('Executing No_seat state')
        # announce that there is no seat available
        speak("Sorry, there is no available seat")
        return 'continue_Introduce_guest'


class Seat(smach.State):
    def __init__(self):
        rospy.loginfo('Initiating Seat state')
        smach.State.__init__(self,outcomes=['continue_Introduce_guest'])
    def execute(self,userdata):
        rospy.loginfo('Executing Seat state')
        # announce that there is available seat
        # point to the furniture
        speak("There is an available seat here")
        return 'continue_Introduce_guest'


class Introduce_guest(smach.State):
    def __init__(self):
        rospy.loginfo('Initiating Introduce_guest state')
        smach.State.__init__(self,outcomes=['continue_Introduce_host'])
        global person_count
    def execute(self,userdata):
        rospy.loginfo('Executing Introduce_guest state')
        # find the host and face the robot to the host
        # clearly identify the person being introduced and state their name and favorite drink
        if person_count == 1:
            speak("Hello {host_name}, the guest who is on the {furniture} is {guest_1}".format(host_name = "John", furniture = "Couch", guest_1 = "Peter"))
            
        if person_count == 2:
            speak("Hello {host_name}, the new guest is {guest_2}".format(host_name = "John", guest_2 = "James"))
            speak("His favorite drink is {fav_drink2}".format(fav_drink2 = "sprite"))
        return 'continue_Introduce_host'

    
class Introduce_host(smach.State):
    def __init__(self):
        rospy.loginfo('Initiating Introduce_host state')
        smach.State.__init__(self,outcomes=['continue_Navigate_to_start'])
        global person_count
    def execute(self,userdata):
        rospy.loginfo('Executing Introduce_host state')
        # find the guest and face the robot to the guest
        # clearly identify the person being introduced annd state their name and favorite drink
        if person_count == 1:
            # find the guest1 and face the robot to the guest1
            speak("Hello {guest_1}, the host's name is {host_name}".format(guest_1 = "Peter", host_name = "John"))
            speak("His favorite drink is {fav_drink_host}".format(fav_drink_host = "coke"))
        if person_count == 2:
            # find the guest_2 and face the robot the the guest_2
            speak("Hello {guest_2}, the host's name is {host_name}".format(guest_2 = "James", host_name = "John"))
            speak("His favorite drink is {fav_drink_host}".format(fav_drink_host= "coke"))
        return 'continue_Navigate_to_start'


class Navigate_to_start(smach.State):
    def __init__(self):
        rospy.loginfo('Initiating Navigate_to_start state')
        smach.State.__init__(self,outcomes=['continue_Standby', 'continue_SUCCEEDED'])
        global person_count
    def execute(self,userdata):
        rospy.loginfo('Executing Navigate_to_start state')
        # navigate back to the door to wait for the next guest
        if person_count == 2:
            return 'continue_SUCCEEDED'
        else:
            # navigate back to the door
            return 'continue_Standby'


if __name__ == '__main__':
    rospy.init_node('receptionist_task')

    person_count = 0

    # Create a SMACH state machine
    sm_top = smach.StateMachine(outcomes=['SUCCEEDED'])

    # Open the container
    with sm_top:
        # Add states to the container
        smach.StateMachine.add('Start_signal', Start_signal(),
                               transitions={'continue_Standby':'Standby'})
        smach.StateMachine.add('Standby', Standby(),
                               transitions={'continue_Ask':'Ask'})
        smach.StateMachine.add('Ask', Ask(),
                               transitions={'continue_Navigation':'Navigation'})
        smach.StateMachine.add('Navigation', Navigation(),
                               transitions={'continue_No_seat':'No_seat',
                                            'continue_Seat':'Seat'})
        smach.StateMachine.add('No_seat', No_seat(),
                               transitions={'continue_Introduce_guest':'Introduce_guest'})
        smach.StateMachine.add('Seat', Seat(),
                               transitions={'continue_Introduce_guest':'Introduce_guest'})
        smach.StateMachine.add('Introduce_guest', Introduce_guest(),
                               transitions={'continue_Introduce_host':'Introduce_host'})
        smach.StateMachine.add('Introduce_host', Introduce_host(),
                               transitions={'continue_Navigate_to_start':'Navigate_to_start'})
        smach.StateMachine.add('Navigate_to_start', Navigate_to_start(),
                               transitions={'continue_Standby':'Standby',
                                            'continue_SUCCEEDED':'SUCCEEDED'})

    sis = smach_ros.IntrospectionServer('Server_name', sm_top, '/Receptionist')
    sis.start()
    # Execute SMACH plan
    outcome = sm_top.execute()
    rospy.spin()
    sis.stop()
    
    

