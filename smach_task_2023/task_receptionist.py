# run with conda env: nlp
# import sys
# sys.path.append('/core_nlp/')
import signal
import rospy
import smach
import nlp_client
import threading
from ratfin import *

from core_smach.person import Person
from core_nlp.utils import (WakeWord, Speak, GetIntent, GetName, GetObject, GetLocation,)
from core_nlp.emerStop import EmergencyStop
from core_cv.custom_socket import CustomSocket
from core_cv.face_recognition import RegisterFace, DetectFace
from core_cv.image_captioning  import ImageCaption

# Task specific state
class AddPerson(smach.State):
    def __init__(self): 
        smach.State.__init__(self,
                            outcomes=['out1','out0'],
                            input_keys=[
                                        # from CV
                                        "age",
                                        "gender",
                                        "race", 
                                        "hair_color",
                                        "shirt_color",
                                        "glasses", 
                                        # from NLP
                                        "name",
                                        "favorite_drink",
                                        # from top-state,
                                        "people_list",
                                        "people_index",
                                        ],
                            output_keys=[
                                        # to top-state
                                        'people_list',
                                        'people_index'
                                        ])

    def execute(self, userdata):

        # Log the execution stage
        rospy.loginfo(f'(AddPerson): Executing..')
        
        p = Person(name=userdata.name,
                   favorite_drink=userdata.favorite_drink,
                   age=userdata.age,
                   shirt_color=userdata.shirt_color,
                   hair_color=userdata.hair_color
                   )
        
        # Add person object to people_list
        userdata.people_list.append(p)

        # print all people attributes
        # for person in userdata.people_list:
        #     print(person.__dict__)

        userdata.people_index += 1
        rospy.loginfo(f'(AddPerson): {p.name} added to people_list. {p.__dict__}')
        
        return 'out1'

# Task specific state
class IntroducePeople(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                            outcomes=['out1', 'out0'],
                            input_keys=['people_list', 'max_people','people_index'],
                            output_keys=['people_index'])

    def execute(self, userdata):

        # Log the execution stage
        rospy.loginfo(f'(IntroducePeople): Executing..')

        # Extract people from userdata
        person1 : Person = userdata.people_list[0]

        # Contruct text to speak
        if userdata.max_people == 1:

            text : str = f"""Hello, {person1.name}. 
            you are {person1.age} years old 
            has {person1.hair_color} hair 
            wears a {person1.shirt_color} shirt and your favorite drink is {person1.favorite_drink}. """

        elif userdata.max_people == 2:
            person2 : Person = userdata.people_list[1]
            text : str = f"""Hello, {person1.name}. 
            you are {person1.age} years old 
            has {person1.hair_color} hair 
            wears a {person1.shirt_color} shirt and 
            your favorite drink is {person1.favorite_drink}. 
            Next to you is {person2.name}. 
            they are {person2.age} years old 
            has {person2.hair_color} hair 
            wears a {person2.shirt_color} shirt 
            and their favorite drink is {person2.favorite_drink}."""
        else:
            raise Exception("")
            
        
        # Speak the text
        nlp_client.speak(text=text)

        return 'out1'

# Task specific state
class DummyCv(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['out1', 'out0'],
                             output_keys=['age', 'shirt_color', 'hair_color']
                             )

    def execute(self, userdata):
        userdata.age = 12
        userdata.shirt_color = "blue"
        userdata.hair_color = "negro"

        return 'out1'


# Task specific state
class CheckPeople(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['out1', 'out0'],
                             input_keys=['people_index','max_people'],
                             output_keys=['people_index'])
        
    def execute(self, userdata):
        # Log the execution stage
        rospy.loginfo(f'(CheckPeople): Executing..')

        if userdata.people_index < userdata.max_people:
            # Log the execution stage
            rospy.loginfo(f'(CheckPeople): More people to add, {userdata.people_index} < {userdata.max_people}')
            return "out1"
        else:
            # Log the execution stage
            rospy.loginfo(f'(CheckPeople): Max Capacity')
            return "out0"


def main():
    speak_debug = False
    response_debug = False
    NODE_NAME = "smach_task_receptionist"
    rospy.init_node(NODE_NAME)
    
    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['out0'])
    

    
    
    # Declear Variables for top-state
    sm.userdata.max_people = 1
    sm.userdata.people_index = 0
    sm.userdata.intent = ""
    sm.userdata.people_list : list[Person] = [] 
    sm.userdata.couch_location = [0,1,2]

    # Current person attribute
    # from CV
    sm.userdata.age = 0
    sm.userdata.gender = ""
    sm.userdata.race  = ""
    sm.userdata.hair_color = ""
    sm.userdata.shirt_color = ""
    sm.userdata.glasses = ""

    # from NLP
    sm.userdata.name = ""
    sm.userdata.favorite_drink = ""

    with sm:

        smach.StateMachine.add('CHECK_PEOPLE',
                            # Check if there are more people to add
                            CheckPeople(),
                            remapping={
                                            'people_index': 'people_index'
                                        },
                            transitions={
                                            'out1': 'SPEAK_ASK_NAME',
                                            'out0': 'INTRODUCE_PEOPLE'
                                        }
                                        )
        
        smach.StateMachine.add('INTRODUCE_PEOPLE',
                            # if there are no more people to add, introduce people
                            IntroducePeople(),
                            remapping={'people_list': 'people_list',
                                       'max_people':'max_people'},
                            transitions={'out1': 'out0',
                                        'out0': 'out0'}
                                        )
        
        smach.StateMachine.add('SPEAK_ASK_NAME',
                            Speak(text="Greetings My name is Walkie, What's your name?"),
                                # Speak(text="Walkie name"),
                            transitions={'out1': 'GET_NAME',
                                            'out0': 'out0'},)
        smach.StateMachine.add('GET_NAME',
                            GetName(speak_debug=speak_debug,
                                    response_debug=response_debug,
                                    timeout=3),
                            transitions={'out1': 'SPEAK_RESPOND_NAME_FACE',
                                            'out0': 'out0'},
                            remapping={'listen_name': 'name'})
        smach.StateMachine.add('SPEAK_RESPOND_NAME_FACE',
                            Speak(text="Hello {} nice to meet you I will now take a picture of you please stand still",keys=["name"]),
                            transitions={'out1': 'REGISTER_FACE',
                                            'out0': 'out0'},)
        
        # CV Facial Recognition saved to server
        smach.StateMachine.add('REGISTER_FACE',
                            # if not registered, register faces
                            RegisterFace(),
                            remapping={'name': 'name',},
                            transitions={
                                            'out1': 'IMAGE_CAPTION',
                                            'undo': "REGISTER_FACE"
                                        }
                                        )
        
        # CV Image Caption: output all userdata
        smach.StateMachine.add('IMAGE_CAPTION',
                            ImageCaption(),
                            remapping={
                                        "age": "age",
                                        "gender": "gender",
                                        "race": "race",
                                        "hair_color": "hair_color",
                                        "shirt_color": "shirt_color",
                                        "glasses": "glasses",
                                        },
                            transitions={
                                        'out1': 'SPEAK_ASK_OBJECT',
                                        'undo': "REGISTER_FACE"
                                        }
                                        )


        # smach.StateMachine.add('DUMMY_CV',
        #                     DummyCv(),
        #                     transitions={'out1': 'SPEAK_ASK_NAME',
        #                                     'out0': 'out0'},
        #                         remapping={'age': 'age', 'shirt_color': 'shirt_color', 'hair_color': 'hair_color'})
        
        
        
        smach.StateMachine.add('SPEAK_ASK_OBJECT',
                    Speak(text="So {}, What's your favorite drink?",
                            keys=["name"]),
                            transitions={'out1': 'GET_OBJECT',
                                    'out0': 'out0'},)

        smach.StateMachine.add('GET_OBJECT',
                            GetObject(speak_debug=speak_debug,
                                        response_debug=response_debug,
                                        timeout=3),
                            transitions={'out1': 'SPEAK_RESPOND_OBJECT',
                                            'out0': 'out0'},
                            remapping={'listen_object': 'favorite_drink'})
        
        smach.StateMachine.add('SPEAK_RESPOND_OBJECT',
                                Speak(text="oh!, I like {} too",
                                    keys=["favorite_drink"]),
                                transitions={'out1': 'ADD_PERSON',
                                            'out0': 'out0'})
                                                
        smach.StateMachine.add('ADD_PERSON',
                                AddPerson(),
                                transitions={'out1': 'CHECK_PEOPLE', 
                                             'out0': 'out0'},
                                remapping={
                                        # from CV
                                        "age":"age",
                                        "gender":"gender",
                                        "race":"race", 
                                        "hair_color":"hair_color",
                                        "shirt_color":"shirt_color",
                                        "glasses":"glasses", 
                                        # from NLP
                                        "name":"name",
                                        "favorite_drink":"favorite_drink",
                                        # from top-state
                                        "people_list":"people_list",
                                        'people_index':'people_index',
                                        
                                        }
                                )
        
        
    # Execute SMACH plan
    smach_thread = threading.Thread(target=sm.execute)
    smach_thread.start()

    es = EmergencyStop()
    es_thread = threading.Thread(target=es.execute)
    es_thread.start()
    es.emer_stop_handler()


    
if __name__ == '__main__':
    main()
