#!usr/bin/env python
import rospy
import roslib
import smach
import smach_ros


class Start_signal(smach.State):
    def __init__(self):
        rospy.loginfo('Initiating Start_signal state')
        smach.State.__init__(self, outcomes=['continue_Go_to'])
    def execute(self,userdata):
        rospy.loginfo('Executing Start_signal state')
        return 'continue_Go_to'


class Go_to(smach.State):
    def __init__(self):
        rospy.loginfo('Initiating Go_to state')
        smach.State.__init__(self, outcomes=['continue_Rotate'])
    def execute(self,userdata):
        rospy.loginfo('Executing Go_to state')
        return 'continue_Rotate'


class Rotate(smach.State):
    def __init__(self):
        rospy.loginfo('Initiating Rotate state')
        smach.State.__init__(self, outcomes=['continue_Go_to_person'])
    def execute(self,userdata):
        rospy.loginfo('Executing Rotate state')
        return 'continue_Go_to_person'


class Go_to_person(smach.State):
    def __init__(self):
        rospy.loginfo('Initiating Go_to_person state')
        smach.State.__init__(self, outcomes=['continue_Ask'])
    def execute(self,userdata):
        rospy.loginfo('Executing Go_to_person state')
        return 'continue_Ask'


class Ask(smach.State):
    def __init__(self):
        rospy.loginfo('Initiating Ask state')
        smach.State.__init__(self, outcomes=['continue_Go_to_person','continue_Go_to_start'])
    def execute(self,userdata):
        rospy.loginfo('Executing Ask state')
        global count_person
        count_person += 1
        if count_person < 2:
            rospy.loginfo('Go_to_person')
            return 'continue_Go_to_person'

        else:
            return 'continue_Go_to_start'


class Go_to_start(smach.State):
    def __init__(self):
        rospy.loginfo('Initiating Go_to_start state')
        smach.State.__init__(self, outcomes=['continue_Announce'])
    def execute(self,userdata):
        rospy.loginfo('Executing Go_to_start state')
        return 'continue_Announce'

class Announce(smach.State):
    def __init__(self):
        rospy.loginfo('Initiating Announce state')
        smach.State.__init__(self,outcomes=['continue_SUCCEEDED'])
    def execute(self,userdata):
        rospy.loginfo('Executing Announce state')
        return 'continue_SUCCEEDED'
if __name__ == '__main__':
    rospy.init_node('Find_my_mate')
    #declare the global variable
    count_person = 0
    #start state machine
    sm_top = smach.StateMachine(outcomes =['SUCCEEDED', 'ABORTED'])
    with sm_top:
        smach.StateMachine.add('Start_signal', Start_signal(),
                               transitions={'continue_Go_to':'Go_to'})
        smach.StateMachine.add('Go_to', Go_to(),
                               transitions={'continue_Rotate':'Rotate'})
        smach.StateMachine.add('Rotate', Rotate(),
                               transitions={'continue_Go_to_person':'Go_to_person'})
        smach.StateMachine.add('Go_to_person', Go_to_person(),
                               transitions={'continue_Ask':'Ask'})
        smach.StateMachine.add('Ask', Ask(),
                               transitions={'continue_Go_to_person':'Go_to_person',
                                            'continue_Go_to_start':'Go_to_start'})
        smach.StateMachine.add('Go_to_start', Go_to_start(),
                               transitions={'continue_Announce':'Announce'})
        smach.StateMachine.add('Announce', Announce(),
                               transitions={'continue_SUCCEEDED':'SUCCEEDED'})


    #sis start
    sis = smach_ros.IntrospectionServer('Server_name', sm_top, '/FindMyMateRoot')
    sis.start()

    outcomes=sm_top.execute()
    rospy.spin()
    sis.stop()
        
