#!usr/bin/env python
import rospy
import roslib
import smach
import smach_ros
#import msg file
from geometry_msgs.msg import Pose

class Start_signal(smach.State):
    def __init__(self):
        rospy.loginfo('Initiating Start_signal state')
        smach.State.__init__(self, outcomes=['continue_Navigate_object'])

    def execute(self, userdata):
        rospy.loginfo('Executing Start_signal state')
        return 'continue_Navigate_object'


class Navigate_object(smach.State):
    def __init__(self):
        rospy.loginfo('Initiating Navigate_object state')
        smach.State.__init__(self, outcomes=['continue_ABORTED', 'continue_Find_object'])
        self.case = 1
    def execute(self, userdata):
        global count_round
        count_round += 1
        rospy.loginfo('Execute Navigate_object state')
        rospy.loginfo('Round %s', count_round)
        if count_round == 6:
            return 'continue_ABORTED'
        else:
            return 'continue_Find_object'


class Find_object(smach.State):
    def __init__(self):
        rospy.loginfo('Initiating Find_object state')
        smach.State.__init__(self, outcomes=['continue_Navigate_object', 'continue_Get_pose'])
        self.case = 1
    def execute(self, userdata):
        rospy.loginfo('Executing Find_object state')
        if self.case == 0:
            return 'continue_Get_pose'

        else:
            return 'continue_Navigate_object'


class Get_pose(smach.State):
    def __init__(self):
        rospy.loginfo('Initiating Get_pose state')
        smach.State.__init__(self, outcomes=['continue_Pick'],
                             input_keys = ['Get_pose_out'],
                             output_keys = ['Get_pose_out'])
    def execute(self, userdata):
        rospy.loginfo('Executing Get_pose state')
        #output
        userdata.Get_pose_out.position.x = 1
        userdata.Get_pose_out.position.y = 2
        userdata.Get_pose_out.position.z = 3
        userdata.Get_pose_out.orientation.x = 4
        userdata.Get_pose_out.orientation.y = 5
        userdata.Get_pose_out.orientation.z = 6        
        userdata.Get_pose_out.orientation.w = 7

        
        return 'continue_Pick'


class Pick(smach.State):
    def __init__(self):
        rospy.loginfo('Initiating Pick state')
        smach.State.__init__(self, outcomes=['continue_Navigate_table'],
                             input_keys = ['Pick_in'])
    def execute(self, userdata):
        rospy.loginfo('Executing Pick state')
        
        return 'continue_Navigate_table'


class Navigate_table(smach.State):
    def __init__(self):
        rospy.loginfo('Initiating Navigate_table state')
        smach.State.__init__(self, outcomes=['continue_Place_object'])
    def execute(self, userdata):
        rospy.loginfo('Executing Navigate_table state')
        return 'continue_Place_object'

class Place_object(smach.State):
    def __init__(self):
        rospy.loginfo('Initiating Place_object state')
        smach.State.__init__(self, outcomes=['continue_Navigate_object',
                                             'continue_SUCCEEDED'])
    def execute(self, userdata):
        global count_object
        rospy.loginfo('Executing Place_object state')
        count_object += 1
        rospy.loginfo('object count = %s', count_object)
        if count_object == 5:
            return 'continue_SUCCEEDED'
        else:
            
            return 'continue_Navigate_object'


if __name__ == '__main__':
    #content in the main function bruh bruh bruh
    rospy.init_node('serving_breakfast')
    #declare the global variable
    count_object = 0
    count_round = 0
    #declare
    sm_top = smach.StateMachine(outcomes=['SUCCEEDED', 'ABORTED'])
    #passing data
    sm_top.userdata.sm_pose = Pose()
    with sm_top:
        smach.StateMachine.add('Start_signal', Start_signal(),
                               transitions={'continue_Navigate_object':'Navigate_object'})
        smach.StateMachine.add('Navigate_object', Navigate_object(),
                               transitions={'continue_Find_object':'Find_object',
                                            'continue_ABORTED':'ABORTED'})
        smach.StateMachine.add('Find_object', Find_object(),
                               transitions={'continue_Get_pose':'Get_pose',
                                            'continue_Navigate_object':'Navigate_object'})
        smach.StateMachine.add('Get_pose', Get_pose(),
                               transitions={'continue_Pick':'Pick'},
                               remapping={'Get_pose_in':'sm_pose',
                                          'Get_pose_out':'sm_pose'})
        smach.StateMachine.add('Pick', Pick(),
                               transitions={'continue_Navigate_table':'Navigate_table'},
                               remapping={'Pick_in':'sm_pose'})
        smach.StateMachine.add('Navigate_table', Navigate_table(),
                               transitions={'continue_Place_object':'Place_object'})
        smach.StateMachine.add('Place_object', Place_object(),
                               transitions={'continue_Navigate_object':'Navigate_object',
                                            'continue_SUCCEEDED':'SUCCEEDED'})

        sis = smach_ros.IntrospectionServer('Service_name', sm_top, '/ServingBreakfast')
        sis.start()

        outcomes = sm_top.execute()

        rospy.spin()
        sis.stop()
