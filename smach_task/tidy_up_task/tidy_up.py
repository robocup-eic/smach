import roslib
import rospy
import smach
import smach_ros

class Stand_by(smach.State):
    def __init__(self):
        rospy.loginfo('initiating standby state')
        smach.State.__init__(self, outcomes = ['continue_Ask_room'])
    def execute(self, userdata):
        return 'continue_Ask_room'

#--------------------------

class Ask_room(smach.State):
    def __init__(self):
        rospy.loginfo('initiating ask room state')
        smach.State.__init__(self, outcomes = ['continue_Ask_location'])
    def execute(self, userdata):
        return 'continue_Ask_location'

#---------------------------
    
class Ask_location(smach.State):
    def __init__(self):
        rospy.loginfo('initiating ask location state')
        smach.State.__init__(self, outcomes = ['continue_succeeded', 'continue_Navigation'])
    def execute(self, userdata):
        if True:
            return 'continue_succeeded'
        else:
            return 'continue_Navigation'
#----------------------------
class Navigation(smach.State):
    def __init__(self):
        rospy.loginfo('initiating navigation state')
        smach.State.__init__(self, outcomes = ['continue_GetObjectName'])
    def execute(self, userdata):
        return 'continue_GetObjectName'
        
#---------------------------
#--sm_manipulation
#---------------------------

class GetObjectName(smach.State):
    def __init__(self):
        rospy.loginfo('initiating get object name')
        smach.State.__init__(self, outcomes = ['continue_GetObjectPose'])
    def execute(self, userdata):
        return 'continue_GetObjectPose'

#---------------------------

class GetObjectPose(smach.State):
    def __init__(self):
        rospy.loginfo('initiating get object pose')
        smach.State.__init__(self, outcomes = ['continue_Pick'])
    def execute(self, userdata):
        return 'continue_Pick'

#---------------------------

class Pick(smach.State):
    def __init__(self):
        rospy.loginfo('initiating pick')
        smach.State.__init__(self, outcomes = ['continue_Place'])
    def execute(self, userdata):
        return 'continue_Place'

class Place(smach.State):
    def __init__(self):
        rospy.loginfo('initiating place')
        smach.State.__init__(self, outcomes = ['continue_succeeded2'])
    def execute(self, userdata):
        return 'continue_succeeded2'

def main():
    rospy.init_node('smach_tidy_up_state_machine')

    # Create the top level SMACH state machine

    sm_top = smach.StateMachine(outcomes = ['succeeded'])

    # Open the container
    with sm_top:
        smach.StateMachine.add('STAND_BY', Stand_by(),
                               transitions={'continue_Ask_room':'ASK_ROOM'})
        smach.StateMachine.add('ASK_ROOM', Ask_room(),
                               transitions={'continue_Ask_location':'ASK_LOCATION'})
        smach.StateMachine.add('ASK_LOCATION', Ask_location(),
                               transitions={'continue_succeeded':'succeeded',
                                            'continue_Navigation':'NAVIGATION'})
        smach.StateMachine.add('NAVIGATION', Navigation(),
                               transitions={'continue_GetObjectName': 'SM_MANIPULATION'})
        sm_Manipulation = smach.StateMachine(outcomes = ['succeeded2'])
        with sm_Manipulation:

            smach.StateMachine.add('GET_OBJECT_NAME', GetObjectName(),
                                   transitions={'continue_GetObjectPose':'GET_OBJECT_POSE'})
            smach.StateMachine.add('GET_OBJECT_POSE', GetObjectPose(),
                                   transitions={'continue_Pick':'PICK'})
            smach.StateMachine.add('PICK', Pick(),
                                   transitions={'continue_Place':'PLACE'})
            smach.StateMachine.add('PLACE', Place(),
                                   transitions={'continue_succeeded2':'succeeded2'})
        smach.StateMachine.add('SM_MANIPULATION', sm_Manipulation,
                               transitions={'succeeded2':'ASK_LOCATION'})


    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('tydyup_server', sm_top, '/SM_TIDYUP')
    sis.start()

    # Execute the state machine
    outcome = sm_top.execute()

    # Wait for ctrl-c to stop the application
    outcome = sm_top.execute()
    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    main()
