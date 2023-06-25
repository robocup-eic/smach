# run with conda env: nlp
import roslib
import rospy
import smach
import smach_ros
import nlp_client
from ratfin import *
from person import Person
from core_nlp.utils import (WakeWord, Speak, GetIntent, GetName, GetObject,
                            GetLocation, GetEntities)
from core_cv.pose_estimation import PoseEstimation
from core_smach.move_to import MoveTo
from core_smach.grasp_object import GraspObject
from core_smach.place_object import PlaceObject


class RestaurantDecision(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['out1', 'out2'],
                             input_keys=['intent', 'object'])

    def execute(self, userdata):
        if userdata.intent == 'deny':
            return 'out0'
        elif userdata.intent == 'restaurant_order' and userdata.object:
            return 'out1'
        else:
            return 'out2'


class DummyIdle(smach.State):
    '''
    DUMMY THICKKKKKKKKKKKK
    CANT BREATH WITH ALL THAT ASTHHH MAAAA
    '''
    def __init__(self):
        smach.State.__init__(self, outcomes=['out1'])

    def execute(self, userdata):
        return 'out1'


def main():
    rospy.init_node('smach_example_state_machine')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['out1', 'out0'])

    # Open the container
    with sm:
        # TODO Pose-Estimation for raising hand
        smach.StateMachine.add('IDLE',
                               DummyIdle(),
                               transitions={'out1': 'MOVE_TO_CUSTOMER'})

        smach.StateMachine.add('MOVE_TO_CUSTOMER',
                               MoveTo(),
                               transitions={'out1': 'ASK_CUSTOMER'})

        smach.StateMachine.add(
            'ASK_CUSTOMER',
            Speak(text='Hello! Can I get you something?'),
            transitions={'out1': 'COMPUTE_COMMAND'},
        )

        smach.StateMachine.add(
            'COMPUTE_COMMAND',
            GetEntities(intent=True, object=True),
            transitions={
                'out1': 'RESTAURANT_DECISION',
                'out2': 'SPEAK_REORDER'
            },
            remapping={
                'intent': 'intent',
                'object': 'object'
            },
        )

        smach.StateMachine.add(
            'RESTAURANT_DECISION',
            RestaurantDecision(),
            transitions={
                'out1': 'MOVE_TO_KITCHEN',
                'out0': 'SPEAK_REORDER',
                'out2': 'SPEAK_DONE'
            },
            remapping={'intent': 'intent'},
        )

        smach.StateMachine.add(
            'SPEAK_DONE',
            Speak(text='Okay, you can call me anytime if you need anything!'),
            transitions={'out1': 'IDLE'},
        )

        smach.StateMachine.add(
            'SPEAK_REORDER',
            Speak(
                text=
                'Sorry, I did not get that. Can you repeat? Please note that I can only take one order at a time.'
            ),
            transitions={'out1': 'COMPUTE_COMMAND'},
        )

        smach.StateMachine.add(
            'MOVE_TO_KITCHEN',
            MoveTo(),
            transitions={'out1': 'GRASP_OBJECT'},
        )

        smach.StateMachine.add(
            'GRASP_OBJECT',
            GraspObject(),
            transitions={'out1': 'PLACE_OBJECT'},
        )

        smach.StateMachine.add(
            'PLACE_OBJECT',
            PlaceObject(),
            transitions={'out1': 'SPEAK_HERE_IS_YOUR_ORDER'},
        )

        smach.StateMachine.add(
            'SPEAK_HERE_IS_YOUR_ORDER',
            Speak(text='Here is your order!'),
            transitions={'out1': 'SPEAK_ANYTHING_ELSE'},
        )

        smach.StateMachine.add(
            'SPEAK_ANYTHING_ELSE',
            Speak(text='Is there anything else I can get you?'),
            transitions={'out1': 'COMPUTE_COMMAND'},
        )

    # Execute SMACH plan
    outcome = sm.execute()


if __name__ == '__main__':
    main()