import roslib
import rospy
import smach
import smach_ros

class Standby(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['continue_Detect_guest'])
    def execute(self, userdata):
        return 'continue_Detect_guest'

class Detect_guest(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['continue_SM_GUEST','guest_not_found'])
        self.check_guest = True
    def execute(self, userdata):
        if self.check_guest == True:
            return 'continue_SM_GUEST'
        else:
            return 'guest_not_found'

class Navigate_to_door(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['continue_Standby'])
    def execute(self, userdata):
        return 'continue_Standby'

#################################################################
######################## SM_GUEST ###############################

class Ask_guest(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['continue_Navigate_to_seat'])
    def execute(self, userdata):
        return 'continue_Navigate_to_seat'

class Navigate_to_seat(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['continue_Point_seat','continue_No_seat'])
        self.check_seat = True
    def execute(self, userdata):
        if self.check_seat == True:
            return 'continue_Point_seat'
        else:
            return 'continue_No_seat'

class Point_seat(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['continue_Introduce_guest'])
    def execute(self, userdata):
        return 'continue_Introduce_guest'

class No_seat(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['continue_Introduce_guest'])
    def execute(self, userdata):
        return 'continue_Introduce_guest'

class Introduce_guest(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['continue_Introduce_host'])
    def execute(self, userdata):
        return 'continue_Introduce_host'

class Introduce_host(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['continue_Succeeded'])
    def execute(self, userdata):
        return 'continue_Succeeded'

def main():
    rospy.init_node('Reception')

    sm = smach.StateMachine(outcomes=['Succeeded','Aborted'])

    with sm:
        smach.StateMachine.add('Standby',Standby(),
                                transitions={'continue_Detect_guest':'Detect_guest'})
        
        smach.StateMachine.add('Detect_guest',Detect_guest(),
                                transitions = {'continue_SM_GUEST':'SM_GUEST','guest_not_found':'Standby'})
        
        sm_guest = smach.StateMachine(outcomes = ['Succeeded'])

        with sm_guest:

            smach.StateMachine.add('Ask_guest',Ask_guest(),
                                    transitions = {'continue_Navigate_to_seat':'Navigate_to_seat'})

            smach.StateMachine.add('Navigate_to_seat',Navigate_to_seat(),
                                    transitions = {'continue_Point_seat':'Point_seat','continue_No_seat':'No_seat'})

            smach.StateMachine.add('Point_seat',Point_seat(),
                                    transitions = {'continue_Introduce_guest':'Introduce_guest'})
            
            smach.StateMachine.add('No_seat',No_seat(),
                                    transitions = {'continue_Introduce_guest':'Introduce_guest'})

            smach.StateMachine.add('Introduce_guest',Introduce_guest(),
                                    transitions = {'continue_Introduce_host':'Introduce_host'})

            smach.StateMachine.add('Introduce_host',Introduce_host(),
                                    transitions = {'continue_Succeeded':'Succeeded'})

        smach.StateMachine.add('SM_GUEST', sm_guest,
                                transitions = {'Succeeded':'Navigate_to_door'})                                 

        smach.StateMachine.add('Navigate_to_door',Navigate_to_door(),
                                transitions = {'continue_Standby':'Succeeded'})   

# Set up                                                    
        
        sis = smach_ros.IntrospectionServer('Server_name',sm,'/ArchRoot')
        sis.start()
        
        outcome = sm.execute()

        rospy.spin()
        sis.stop()

if __name__ == '__main__':
    main()