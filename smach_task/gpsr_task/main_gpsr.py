#!/usr/bin/env python

import roslib
import rospy
import smach
import smach_ros


class Standby(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['continue_GOTO','wait_FINDITEM','wait_BRINGIT','wait_FIND_PERSON'])
        self.command_GOTO = True
        self.command_FINDITEM = False
        self.command_BRINGIT = False
        self.command_FIND_PERSON = False
    def execute(self, userdata):
        if self.command_GOTO == True:
            return 'wait_GOTO'
        elif self.command_FINDITEM == True:
            return 'wait_FINDITEM'
        elif self.command_BRINGIT == True:
            return 'wait_BRINGIT'
        elif self.command_FIND_PERSON == True:
            return 'wait_FIND_PERSON'

#################################
######### SM_GOTO ###############

class Navigate(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['continue_Announce'])
    def execute(self, userdata):
        return 'continue_Announce'

class Announce1(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['continue_Succeeded'])
    def execute(self, userdata):
        return 'continue_Succeeded'

#################################
######### SM_FINDITEM ###########

class Store_location(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['continue_Ask_user'])
    def execute(self, userdata):
        return 'continue_Ask_user'

class Ask_user(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['continue_Navigate_to_room'])
    def execute(self, userdata):
        return 'continue_Navigate_to_room'

class Navigate_to_room(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['continue_Find_object1'])
    def execute(self, userdata):
        return 'continue_Find_object1'

class Find_object1(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['continue_Announce2'])
    def execute(self, userdata):
        return 'continue_Announce2'

class Announce2(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes = ['continue_Succeeded'])
    def execute(self, userdata):
        return 'continue_Succeeded'

#################################
########## SM_BRINGIT ###########

class Store_userface_location(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['continue_Find_object2'])
    def execute(self, userdata):
        return 'continue_Find_object2'

class Find_object2(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['continue_Pick_up'])
    def execute(self, userdata):
        return 'continue_Pick_up'

class Pick_up(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['continue_Find_user'])
    def execute(self, userdata):
        return 'continue_Find_user'

class Find_user(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['continue_Place_object'])
    def execute(self, userdata):
        return 'continue_Place_object'

class Place_object(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['continue_Succeeded'])
    def execute(self, userdata):
        return 'continue_Succeeded'

###################################
######### SM_FIND_PERSON ##########

class Navigate_to_room1(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['continue_Find_person'])
    def execute(self, userdata):
        return 'continue_Find_person'

class Navigate_to_room2(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['continue_Find_person'])
    def execute(self, userdata):
        return 'continue_Find_person'

class Find_person(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['continue_room2','Found_person','not_Found_person'])
        self.check_room2 = True
        self.check_Found_person = False
    def execute(self, userdata):
        if self.check_room2 == False:
            return 'continue_room2'
        else:
            if self.check_Found_person == True:
                return 'Found_person'
            else:
                return 'not_Found_person'



def main():
    rospy.init_node('first_smach')

    #Top level 
    sm = smach.StateMachine(outcomes=['SM_Succeeded','SM_Aborted'])

    with sm:
        smach.StateMachine.add('Standby',Standby(),
                                transitions = {'wait_GOTO':'SM_GOTO', 'wait_FINDITEM':'SM_FINDITEM', 'wait_BRINGIT':'SM_BRINGIT','wait_FIND_PERSON':'SM_FIND_PERSON'})

        ###########################################################################
        ############################# SM_GOTO #####################################

        sm_goto = smach.StateMachine(outcomes=['Succeeded'])

        with sm_goto:
            smach.StateMachine.add('Navigate', Navigate(),
                                    transitions = {'continue_Announce':'Announce'})
            
            smach.StateMachine.add('Announce', Announce1(),
                                    transitions = {'continue_Succeeded':'Succeeded'})


        smach.StateMachine.add('SM_GOTO',sm_goto,
                            transitions={'Succeeded':'SM_Succeeded'})
        
        ###########################################################################
        ########################### SM_FINDITEM ###################################

        sm_finditem = smach.StateMachine(outcomes = ['Succeeded'])

        with sm_finditem:
            smach.StateMachine.add('Store_location',Store_location(),
                                    transitions = {'continue_Ask_user':'Ask_user'})
            
            smach.StateMachine.add('Ask_user',Ask_user(),
                                    transitions = {'continue_Navigate_to_room':'Navigate_to_room'})
            
            smach.StateMachine.add('Navigate_to_room', Navigate_to_room(),
                                    transitions = {'continue_Find_object1':'Find_object1'})
            
            smach.StateMachine.add('Find_object1',Find_object1(),
                                    transitions = {'continue_Announce2':'Announce2'})

            smach.StateMachine.add('Announce2',Announce2(),
                                    transitions = {'continue_Succeeded':'Succeeded'})
        
        smach.StateMachine.add('SM_FINDITEM',sm_finditem,
                                transitions = {'Succeeded':'SM_Succeeded'})
        
        ###########################################################################
        ########################### SM_BRINGIT ####################################

        sm_bringit = smach.StateMachine(outcomes = ['Succeeded'])

        with sm_bringit:
            smach.StateMachine.add('Store_userface_location',Store_userface_location(),
                                    transitions = {'continue_Find_object2':'Find_object2'})
            
            smach.StateMachine.add('Find_object2',Find_object2(),
                                    transitions = {'continue_Pick_up':'Pick_up'})
            
            smach.StateMachine.add('Pick_up',Pick_up(),
                                    transitions = {'continue_Find_user':'Find_user'})
            
            smach.StateMachine.add('Find_user',Find_user(),
                                    transitions = {'continue_Place_object':'Place_object'})
            
            smach.StateMachine.add('Place_object',Place_object(),
                                    transitions = {'continue_Succeeded':'Succeeded'})

        smach.StateMachine.add('SM_BRINGIT',sm_bringit,
                                transitions = {'Succeeded':'SM_Succeeded'})

        ############################################################################
        ########################### SM_FIND_PERSON #################################

        sm_find_person = smach.StateMachine(outcomes =['Succeeded','Aborted'])

        with sm_find_person:
            smach.StateMachine.add('Navigate_to_room1',Navigate_to_room1(),
                                    transitions = {'continue_Find_person':'Find_person'})
            
            smach.StateMachine.add('Find_person',Find_person(),
                                    transitions = {'continue_room2':'Navigate_to_room2',
                                                    'not_Found_person':'Aborted',
                                                    'Found_person':'Succeeded'})
            
            smach.StateMachine.add('Navigate_to_room2',Navigate_to_room2(),
                                    transitions = {'continue_Find_person':'Find_person'})
        
        smach.StateMachine.add('SM_FIND_PERSON',sm_find_person,
                                transitions = {'Aborted':'SM_Aborted',
                                                'Succeeded':'SM_Succeeded'})


# Set up                                                    
        
        sis = smach_ros.IntrospectionServer('Server_name',sm,'/ArchRoot')
        sis.start()
        
        outcome = sm.execute()

        rospy.spin()
        sis.stop()

if __name__ == '__main__':
    main()
