#!/usr/bin/env python

import roslib
import rospy
import smach
import smach_ros


class Start_signal(smach.State):
    def __init__(self):
        rospy.loginfo('Initiating Start_signal state')
        smach.State.__init__(self,outcomes=['continue_Standby'])
    def execute(self,userdata):
        rospy.loginfo('Executing Start_signal state')
        return 'continue_Standby'


class Standby(smach.State):
    def __init__(self):
        rospy.loginfo('Initiating Standby state')
        smach.State.__init__(self,outcomes=['continue_Ask'])
    def execute(self,userdata):
        rospy.loginfo('Executing Standby state')
        return 'continue_Ask'


class Ask(smach.State):
    def __init__(self):
        rospy.loginfo('Initiating Ask state')
        smach.State.__init__(self,outcomes=['continue_Navigation'])
    def execute(self,userdata):
        rospy.loginfo('Executing Ask state')
        return 'continue_Navigation'


class Navigation(smach.State):
    def __init__(self):
        rospy.loginfo('Initiating Navigation state')
        smach.State.__init__(self,outcomes=['continue_No_seat','continue_Seat'])
        self.case = 0
    def execute(self,userdata):
        rospy.loginfo('Executing Navigation state')
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
        return 'continue_Introduce_guest'


class Seat(smach.State):
    def __init__(self):
        rospy.loginfo('Initiating Seat state')
        smach.State.__init__(self,outcomes=['continue_Introduce_guest'])
    def execute(self,userdata):
        rospy.loginfo('Executing Seat state')
        return 'continue_Introduce_guest'


class Introduce_guest(smach.State):
    def __init__(self):
        rospy.loginfo('Initiating Introduce_guest state')
        smach.State.__init__(self,outcomes=['continue_Introduce_host'])
    def execute(self,userdata):
        rospy.loginfo('Executing Introduce_guest state')
        return 'continue_Introduce_host'

    
class Introduce_host(smach.State):
    def __init__(self):
        rospy.loginfo('Initiating Introduce_host state')
        smach.State.__init__(self,outcomes=['continue_Navigate_to_start'])
    def execute(self,userdata):
        rospy.loginfo('Executing Introduce_host state')
        return 'continue_Navigate_to_start'


class Navigate_to_start(smach.State):
    def __init__(self):
        rospy.loginfo('Initiating Navigate_to_start state')
        smach.State.__init__(self,outcomes=['continue_Standby', 'continue_SUCCEEDED'])
        self.case = 0
    def execute(self,userdata):
        rospy.loginfo('Executing Navigate_to_start state')
        if self.case == 0:
            return 'continue_Standby'
        else:
            return 'continue_SUCCEEDED'


def main():
    rospy.init_node('initiating receptionist node')

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
if __name__ == '__main__':
    main()
