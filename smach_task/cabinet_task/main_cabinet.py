#!usr/bin/env python
import rospy
import roslib
import smach
import smach_ros

class Start_signal(smach.State):
    def __init__(self):
        rospy.loginfo('Initiating Start_signal state')
        smach.State.__init__(self,outcomes=['continue_Navigate_cabinet'])
    def execute(self, userdata):
        rospy.loginfo('Executing Start_signal state')
        return 'continue_Navigate_cabinet'


class Navigate_cabinet(smach.State):
    def __init__(self):
        rospy.loginfo('Initiating Navigate_cabinet state')
        smach.State.__init__(self,outcomes=['continue_Categorize_object', 'continue_Place_object'])
        self.case = 0
    def execute(self, userdata):
        rospy.loginfo('Executing Navigate_cabinet state')
        if self.case == 0:
            return 'continue_Categorize_object'
        else:
            return 'continue_Place_object'


class Place_object(smach.State):
    def __init__(self):
        rospy.loginfo('Initiating Place_object state')
        smach.State.__init__(self,outcomes=['continue_Navigate_table', 'continue_SUCCEEDED'])
        self.case = 0
    def execute(self, userdata):
        rospy.loginfo('Executing Place_object state')
        if self.case == 0:
            return 'continue_Navigate_table'
        else:
            return 'continue_SUCCEEDED'


class Categorize_object(smach.State):
    def __init__(self):
        rospy.loginfo('Initiating Categorize_object state')
        smach.State.__init__(self,outcomes=['continue_Navigate_table'])
    def execute(self, userdata):
        rospy.loginfo('Executing Categorize_object state')
        return 'continue_Navigate_table'


class Navigate_table(smach.State):
    def __init__(self):
        rospy.loginfo('Initiating Navigate_table state')
        smach.State.__init__(self,outcomes=['continue_Find_object'])
    def execute(self,userdata):
        rospy.loginfo('Executing Navigate_table state')
        return 'continue_Find_object'


class Find_object(smach.State):
    def __init__(self):
        rospy.loginfo('Initiating Find_object state')
        smach.State.__init__(self,outcomes=['continue_Get_pose'])
    def execute(self,userdata):
        rospy.loginfo('Executing Find_object state')
        return 'continue_Get_pose'


class Get_pose(smach.State):
    def __init__(self):
        rospy.loginfo('Initiating Get_pose state')
        smach.State.__init__(self,outcomes=['continue_Pick'])
    def execute(self,userdata):
        rospy.loginfo('Executing Get_pose state')
        return 'continue_Pick'


class Pick(smach.State):
    def __init__(self):
        rospy.loginfo('Initiating Pick state')
        smach.State.__init__(self,outcomes=['continue_Navigate_cabinet'])
    def execute(self,userdata):
        rospy.loginfo('Executing Pick state')
        return 'continue_Navigate_cabinet'


if __name__ == '__main__':
    rospy.init_node('main_cabinet')


    sm_top = smach.StateMachine(outcomes=['SUCCEEDED'])
    with sm_top:
        smach.StateMachine.add('Start_signal', Start_signal(),
                               transitions={'continue_Navigate_cabinet':'Navigate_cabinet'})
        smach.StateMachine.add('Navigate_cabinet', Navigate_cabinet(),
                               transitions={'continue_Place_object':'Place_object',
                                            'continue_Categorize_object':'Categorize_object'})
        smach.StateMachine.add('Place_object', Place_object(),
                               transitions={'continue_SUCCEEDED':'SUCCEEDED',
                                            'continue_Navigate_table':'Navigate_table'})
        smach.StateMachine.add('Navigate_table', Navigate_table(),
                               transitions={'continue_Find_object':'Find_object'})
        smach.StateMachine.add('Categorize_object', Categorize_object(),
                               transitions={'continue_Navigate_table':'Navigate_table'})
        smach.StateMachine.add('Find_object', Find_object(),
                               transitions={'continue_Get_pose':'Get_pose'})
        smach.StateMachine.add('Get_pose', Get_pose(),
                               transitions={'continue_Pick':'Pick'})
        smach.StateMachine.add('Pick', Pick(),
                               transitions={'continue_Navigate_cabinet':'Navigate_cabinet'})
    sis = smach_ros.IntrospectionServer('Server_name', sm_top, '/Kannroot')
    sis.start()

    outcome = sm_top.execute()


    rospy.spin()
    sis.stop()
