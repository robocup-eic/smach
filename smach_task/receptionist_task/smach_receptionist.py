#!/usr/bin/env python
import roslib
import rospy
import smach
import smach_ros

class Stand_by(smach.State):
    def __init__(self):
        rospy.loginfo('initiating stand by state')
        smach.State.__init__(self, outcomes = ['continue_SM_GUEST'])

    def execute(self, userdata):
        return 'continue_SM_GUEST'

class Navigate_To_Door(smach.State):
    def __init__(self):
        rospy.loginfo('initiating navigate to door state')
        smach.State.__init__(self, outcomes = ['continue_Stand_By'])
    def execute(self, userdata):
        return 'continue_Stand_By'

class Input_Hostname(smach.State):
    def __init__(self):
        rospy.loginfo('initiating input hostname state')
        smach.State.__init__(self, outcomes = ['continue_Stand_By'])
    def execute(self, userdata):
        return 'continue_Stand_By'
# sm guest
class Ask_Guest(smach.State):
    def __init__(self):
        rospy.loginfo('initiating ask guest state')
        smach.State.__init__(self, outcomes = ['continue_Navigate_To_Seat'])
    def execute(self, userdata):
        return 'continue_Navigate_To_Seat'

class Navigate_To_Seat(smach.State):
    def __init__(self):
        rospy.loginfo('initiating navigating to seat state')
        smach.State.__init__(self, outcomes = ['continue_No_Seat','continue_Please_Seated'])
        self.x = 1
    def execute(self, userdata):
        if self.x == 1:
            return 'continue_No_Seat'
        else:
            return 'continue_Please_Seated'

class No_Seat(smach.State):
    def __init__(self):
        rospy.loginfo('initiating no seat state')
        smach.State.__init__(self, outcomes = ['continue_Introduce_Guest'])
    def execute(self, userdata):
        return 'continue_Introduce_Guest'

class Please_Seated(smach.State):
    def __init__(self):
        rospy.loginfo('initiating please seated state')
        smach.State.__init__(self, outcomes = ['continue_Introduce_Guest'])
    def execute(self, userdata):
        return 'continue_Introduce_Guest'

class Introduce_Guest(smach.State):
    def __init__(self):
        rospy.loginfo('initiating introduce guest')
        smach.State.__init__(self, outcomes = ['continue_Introduce_Host'])
    def execute(self, userdata):
        return 'continue_Introduce_Host'

class Introduce_Host(smach.State):
    def __init__(self):
        rospy.loginfo('initiating introduce host')
        smach.State.__init__(self, outcomes = ['continue_succeeded'])
    def execute(self, userdata):
        return 'continue_succeeded'
def main():
    rospy.init_node('smach_receptionist_state_machine')
    #create the top level SMACH state machine
    sm_top = smach.StateMachine(outcomes = ['succeeded','aborted'])
    #open the container
    with sm_top:
        smach.StateMachine.add('STAND_BY', Stand_By(),
                               transitions = {'continue_SM_GUEST':'SM_GUEST'})
        smach.StateMachine.add('NAVIGATE_TO_DOOR', Navigate_To_Door(),
                               transitions = {'continue_Stand_By':'STAND_BY'})
        smach.StateMachine.add('INPUT_HOSTNAME', Input_HostName(),
                               transitions = {'continue_Stand_By':'STAND_BY'})
        sm_guest = smach.StateMachine(outcomes = ['succeeded','aborted'])
        with sm_guest:
            smach.StateMachine.add('ASK_GUEST', Ask_Guest(),
                                   transitions = {'continue_Navigate_To_Seat':'NAVIGATE_TO_SEAT'})
            smach.StateMachine.add('NAVIGATE_TO_SEAT', Navigate_To_Seat(),
                                   transitions = {'continue_No_Seat':'NO_SEAT',
                                                  'continue_Please_Seated':'PLEASE_SEATED'})
            smach.StateMachine.add('NO_SEAT', No_Seat(),
                                   transitions = {'continue_Introduce_Guest':'INTRODUCE_GUEST'})
            smach.StateMachine.add('PLEASE_SEATED', Please_Seated(),
                                   transitions = {'continue_Introduce_Guest':'INTRODUCE_GUEST'})
            smach.StateMachine.add('INTRODUCE_GUEST', Introduce_Guest(),
                                   transitions = {'continue_Introduce_Host':'INTRODUCE_HOST'})
            smach.StateMachine.add('INTRODUCE_HOST', Introduce_Host(),
                                   transitions = {'continue_succeeded':'succeeded'})
        smach.StateMachine.add('SM_GUEST', sm_guest,
                               transition = {'succeeded':'NAVIGATE_TO_DOOR'})

    # create and start the introspection server
    sis = smach_ros.IntrospectionServer('receptionist server', sm_top, '/SM_RECEPTIONIST')
    sis.start()

    # Execute the state machine
    outcome = sm_top.execute()

    # wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    main()
                             
