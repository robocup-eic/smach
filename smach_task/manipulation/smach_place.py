import roslib
import rospy
import smach
import smach_ros

class Input(smach.State):
    def __init__(self):
        rospy.loginfo('initiating input state')
        smach.State.__init__(self, outcomes = ['continue_CreateEnvironment'])

    def execute(self, userdata):
        return 'continue_CreateEnvironment'

# --- sm_place -------

class CreateEnvironment(smach.State):
    def __init__(self):
        rospy.loginfo('initiating create environment state')
        smach.State.__init__(self, outcomes = ['continue_GetObjectPoseList'])

    def execute(self, userdata):
        return 'continue_GetObjectPoseList'

class GetObjectPoseList(smach.State):
    def __init__(self):
        rospy.loginfo('initiating get object pose list state')
        smach.State.__init__(self, outcomes = ['continue_Place'])

    def execute(self, userdata):
        return 'continue_Place'

class Place(smach.State):
    def __init__(self):
        rospy.loginfo('initiating place state')
        smach.State.__init__(self, outcomes =['continue_aborted','continue_succeeded'])
    def execute(self, userdata):
        if True:
            return 'continue_aborted'
        else:
            return 'continue_succeeded'



# class Place :

#     def __init__(self, corner_table11, corner_table12, corner_table21, corner_table22, high, current_collision_object_pos) :
#         self.corner_table11 = corner_table11
#         self.corner_table12 = corner_table12
#         self.corner_table21 = corner_table21
#         self.corner_table22 = corner_table22
#         self.high = high
#         self.current_collision_object_pos = current_collision_object_pos

#         rospy.loginfo('initiating place state')
#         smach.State.__init__(self, outcomes =['continue_aborted','continue_succeeded'])

#     def execute(self) :

#         def place_service() :
#             rospy.wait_for_service('cr3_place')
#             try:
#                 place = rospy.ServiceProxy('cr3_place', cr3_place)
#                 res = place(self.corner_table11, self.corner_table12, self.corner_table21, self.corner_table22, self.high, self.current_collision_object_pos)
#                 return res.success_place
#             except rospy.ServiceException as e:
#                 print("Service call failed: %s"%e)

#         rospy.loginfo(self.corner_table11, self.corner_table12, self.corner_table21, self.corner_table22, self.high, self.current_collision_object_pos)
#         success = place_service()
#         print(success)

#         if success :
#             return "continue_aborted"
#         else :
#             return "continue_succeeded"





def main():
    rospy.init_node('smach_sm_place_state_machine')

    sm_top = smach.StateMachine(outcomes = ['succeeded1', 'aborted1'])
    with sm_top:
        smach.StateMachine.add('INPUT', Input(),
                                transitions = {'continue_CreateEnvironment':'CREATEENVIRONMENT'})
        
        smach.StateMachine.add('CREATEENVIRONMENT', CreateEnvironment(),
                                   transitions = {'continue_GetObjectPoseList':'SM_PLACE'})

        sm_place = smach.StateMachine(outcomes = ['succeeded', 'aborted'])
        with sm_place:
            smach.StateMachine.add('GETOBJECTPOSELIST', GetObjectPoseList(),
                                   transitions = {'continue_Place':'PLACE'})
            smach.StateMachine.add('PLACE', Place(),
                                   transitions = {'continue_aborted':'aborted',
                                                  'continue_succeeded':'succeeded'})
        smach.StateMachine.add('SM_PLACE', sm_place,
                               transitions = {'aborted':'aborted1','succeeded':'succeeded1'})

    sis = smach_ros.IntrospectionServer('sm_place_server', sm_top, '/SM_PLACE')
    sis.start()
    
    outcome = sm_top.execute()

    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()
