
# for developement

import smach
from nlp_client import *
from ratfin import *
import sys
# add the path for live ROBOCUP2023-NLP
sys.path.append('/home/robocup2023-nlp/ROBOCUP2023-NLP')
import smach

# Define the states
class State1(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'failure', 'loop'])

    def execute(self, userdata):
        # Your code here
        pass

class State2(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'failure', 'loop'])

    def execute(self, userdata):
        # Your code here
        pass

# Mapping of state names to classes
state_mapping = {
    'State1': State1(),
    'State2': State2(),
    # Add more states here...
}

def generate_state_dict_list(state_sequence):
    state_dict_list = []
    
    for i in range(len(state_sequence)):
        current_state = state_sequence[i]
        state_obj = state_mapping[current_state]
        current_state_with_suffix = current_state + "_" + str(i + 1)  # appending the suffix

        # Define remappings here
        remappings = {'data1': 'data1', 'data2': 'data2'}  # Use actual remappings

        if i == len(state_sequence) - 1:  # Last state
            transitions = {'success': 'final_outcome', 
                           'failure': state_sequence[i - 1] + "_" + str(i) if i > 0 else 'final_outcome', 
                           'loop': current_state_with_suffix}
        else:
            transitions = {'success': state_sequence[i + 1] + "_" + str(i + 2), 
                           'failure': state_sequence[i - 1] + "_" + str(i) if i > 0 else state_sequence[i + 1] + "_" + str(i + 2), 
                           'loop': current_state_with_suffix}

        state_dict_list.append({
            'state_name': current_state_with_suffix,
            'state_obj': state_obj,
            'transitions': transitions,
            'remappings': remappings  # Added remappings here
        })

    return state_dict_list


def construct_smach_state_machine(states_dict_list):
    sm = smach.StateMachine(outcomes=['final_outcome'])

    with sm:
        for state_dict in states_dict_list:
            smach.StateMachine.add(state_dict['state_name'], 
                                   state_dict['state_obj'],
                                   transitions=state_dict['transitions'])

    return sm

# Generate state dict list
state_sequence = ['State1', 'State2', 'State1']
state_dict_list = generate_state_dict_list(state_sequence)

# Construct state machine
sm = construct_smach_state_machine(state_dict_list)
