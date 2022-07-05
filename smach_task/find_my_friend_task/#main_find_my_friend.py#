#/usr/bin/env python

import roslib
import rospy
import smach
import smach_ros
#import msgs file
from geometry_msgs.msg import Pose

class Start_signal(smach.State):
    def __init__(self):
        rospy.loginfo('Initiating Start_signal state')
        smach.State.__init__(self, outcomes=['continue_Navigate_living_room'])
    def execute(self, userdata):
        rospy.loginfo('Executing Start_signal state')
        return 'continue_Navigate_living_room'


class Navigate_living_room(smach.State):
    def __init__(self):
        rospy.loginfo('Initiating Navigate_living_room state')
        smach.State.__init__(self, outcomes=['continue_Approach_person',
                                             'continue_find_person'])
        self.case = 1
    def execute(self, userdata):
        rospy.loginfo('Executing Navigate_living_room state')
        global posi

        #initiate the pose
        posi.orientation.x = 1
        posi.orientation.y = 1
        posi.orientation.z = 1
        posi.orientation.w = 1
        posi.position.x = 1
        posi.position.y = 1
        posi.position.z = 1

        #send the pose
        rospy.loginfo('Send the pose to the find person state')
        print(posi.orientation.x)
        print(posi.orientation.y)
        print(posi.orientation.z)
        print(posi.orientation.w)
        print(posi.position.x)
        print(posi.position.y)
        print(posi.position.z)
        if self.case == 0:
            return 'continue_Approach_person'
        else:
            return 'continue_find_person'


class Approach_person(smach.State):
    def __init__(self):
        rospy.loginfo('Initiating Approach_person state')
        smach.State.__init__(self,outcomes=['continue_Ask'])
    def execute(self, userdata):
        rospy.loginfo('Executing Approach_person state')
        global posi
        #receive the pose
        
        rospy.loginfo('Print the pose from the find person state')
        
        print(posi.orientation.x)
        print(posi.orientation.y)
        print(posi.orientation.z)
        print(posi.orientation.w)
        print(posi.position.x)
        print(posi.position.y)
        print(posi.position.z)
        #initialiize the pose
        posi.orientation.x = 10
        posi.orientation.y = 20
        posi.orientation.z = 30
        posi.orientation.w = 40
        posi.position.x = 50
        posi.position.y = 60
        posi.position.z = 70
        rospy.loginfo('The pose in the approach person state')
        print(posi.orientation.x)
        print(posi.orientation.y)
        print(posi.orientation.z)
        print(posi.orientation.w)
        print(posi.position.x)
        print(posi.position.y)
        print(posi.position.z)
        return 'continue_Ask'


class find_person(smach.State):
    def __init__(self):
        rospy.loginfo('Initiating find_person state')
        smach.State.__init__(self,outcomes=['continue_Approach_person'])
    def execute(self, userdata):
        rospy.loginfo('Executing find_person state')
        global posi

        #receive the pos3
        rospy.loginfo('Receive the pose from the navigation')
        print(posi.orientation.x)
        print(posi.orientation.y)
        print(posi.orientation.z)
        print(posi.orientation.w)
        print(posi.position.x)
        print(posi.position.y)
        print(posi.position.z)
        #send the pose to the approach person
        
        posi.orientation.x = 1
        posi.orientation.y = 2
        posi.orientation.z = 3
        posi.orientation.w = 4
        posi.position.x = 5
        posi.position.y = 6
        posi.position.z = 7
        rospy.loginfo('Send the pose to the approach person')
        print(posi.orientation.x)
        print(posi.orientation.y)
        print(posi.orientation.z)
        print(posi.orientation.w)
        print(posi.position.x)
        print(posi.position.y)
        print(posi.position.z)

        return 'continue_Approach_person'


class Ask(smach.State):
    def __init__(self):
        rospy.loginfo('Initiating Ask state')
        smach.State.__init__(self,outcomes=['continue_find_person',
                                            'continue_Navigate_to_start'])
    def execute(self, userdata):
        global count_person
        rospy.loginfo('Executing Ask state')
        if count_person < 2:
            count_person += 1
            return 'continue_find_person'
        else:
            return 'continue_Navigate_to_start'


class Navigate_to_start(smach.State):
    def __init__(self):
        rospy.loginfo('Initiating Navigate_to_start state')
        smach.State.__init__(self,outcomes=['continue_Announce'])
    def execute(self, userdata):
        rospy.loginfo('Executing Navigate_to_start state')
        return 'continue_Announce'


class Announce(smach.State):
    def __init__(self):
        rospy.loginfo('Initiating Announce state')
        smach.State.__init__(self,outcomes=['continue_SUCCEEDED'])
    def execute(self, userdata):
        rospy.loginfo('Executing Announce state')
        return 'continue_SUCCEEDED'


if __name__ == '__main__':
    # initiate ros node
    rospy.init_node('find_my_friend')
    # initiate global variable
    count_person = 0
    posi = Pose()


    # start state machine
    sm_top = smach.StateMachine(outcomes=['SUCCEEDED'])
    # declare userdata
    sm_top.userdata.sm_pose = Pose()
    with sm_top:
        smach.StateMachine.add('Start_signal', Start_signal(),
                               transitions={'continue_Navigate_living_room':'Navigate_living_room'})
        smach.StateMachine.add('Navigate_living_room', Navigate_living_room(),
                               transitions={'continue_find_person':'find_person',
                                            'continue_Approach_person':'Approach_person'})
        smach.StateMachine.add('find_person', find_person(),
                               transitions={'continue_Approach_person':'Approach_person'})
        smach.StateMachine.add('Approach_person', Approach_person(),
                               transitions={'continue_Ask':'Ask'})
        smach.StateMachine.add('Ask', Ask(),
                               transitions={'continue_Navigate_to_start':'Navigate_to_start',
                                            'continue_find_person':'find_person'})
        smach.StateMachine.add('Navigate_to_start', Navigate_to_start(),
                               transitions={'continue_Announce':'Announce'})
        smach.StateMachine.add('Announce', Announce(),
                               transitions={'continue_SUCCEEDED':'SUCCEEDED'})



        #sis start
        sis = smach_ros.IntrospectionServer('Server_name', sm_top, '/Findmyfriendroot')
        sis.start()
        outcomes = sm_top.execute()

        rospy.spin()
        sis.stop()
        
