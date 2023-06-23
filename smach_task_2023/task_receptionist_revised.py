# run with conda env: nlp
import os
import signal
import roslib
import rospy
import smach
import smach_ros
import nlp_client
import threading
from ratfin import *

from core_smach.person import Person
from utils import (WakeWord, Speak, GetIntent, GetName, GetObject, GetLocation,)
from core_smach.move_to import MoveTo


""" Overall Flow 
1. Wait in the room
1.5 (Optional) Open the door the guest
2. When Person1 enters the room, greet them "Greetings, May I have your name please?" 
2.5 Turn towards Person1 (Remain eye contact)
3. Wait for the person to respond with their name
4. When the person responds with their name, greet them with their name and ask them "What is your favorite drink?"
5. Wait for the person to respond with their favorite drink
6. Then ask them to stand still for a moment while you scan them
7. Scan the person
8. When the scan is complete
9. Look towards the couch
10. Say "Please take a sit on the couch, This is our host "Game" his favorite drink is "Coke". Game this is Person1 etc....
11. Wait for the person to sit down
12. Back to the idle position
13 When Person2 enters the room, greet them "Greetings, May I have your name please?" 
14 Turn towards Person1 (Remain eye contact)
15 Wait for the person to respond with their name
16 When the person responds with their name, greet them with their name and ask them "What is your favorite drink?"
17 Wait for the person to respond with their favorite drink
18 Then ask them to stand still for a moment while you scan them
19 Scan the person
20 When the scan is complete
21 Look towards the couch
22 Say "Please take a sit on the couch, This is our host "Game" his the in the middle of the couch, his favorite drink is "Coke". Person1 name this is Person2 etc....

"""




# Task specific state
class AddPerson(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                            outcomes=['out1','out0'],
                            input_keys=['name','favorite_drink','age',
                                         'shirt_color','hair_color',
                                         'people_list', 'people_index'],
                            output_keys=['people_list','people_index'])

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
                            input_keys=['people_list', 'people_index'],
                            output_keys=['people_index'])

    def execute(self, userdata):

        # Log the execution stage
        rospy.loginfo(f'(IntroducePeople): Executing..')

        # Extract people from userdata
        person1 : Person = userdata.people_list[0]
        person2 : Person = userdata.people_list[1]

        # Contruct text to speak
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
    sm = smach.StateMachine(outcomes=['out0','out1'])
    

    
    
    # Declear Variables for top-state
    sm.userdata.max_people = 2
    sm.userdata.intent = ""
    sm.userdata.people_list : list[Person] = [] 
    sm.userdata.people_index = 0
    sm.userdata.age = 0
    sm.userdata.hair_color = ""
    sm.userdata.shirt_color = ""
    sm.userdata.name = ""
    sm.userdata.favorite_drink = ""
    sm.userdata.couch_location = [0,1,2]
    

    with sm:
        smach.StateMachine.add('GREETINGS_ASK_NAME',
                            Speak(text="""Greetings, May I have your name please?"""),
                            # Speak(text="Please ."),
                            transitions={'out1': 'TURN_TO_PERSON_ONE',
                                            'out0': 'out0'})
        
        smach.StateMachine.add('TURN_TO_PERSON_ONE',
                               MoveTo(),
                                transitions={'out1': 'GET_NAME',
                                            'out0': 'out0'})
                               
        smach.StateMachine.add('GET_NAME',
                            GetName(speak_debug=speak_debug,
                                    response_debug=response_debug,
                                    timeout=3),
                            transitions={'out1': 'SPEAK_ASK_OBJECT',
                                            'out0': 'out0'},
                            remapping={'listen_name': 'name'})
        
        smach.StateMachine.add('SPEAK_ASK_OBJECT',
                    Speak(text="Hello {}, What's your favorite drink?",
                        # Speak(text="Hello {}, favorite drink?",
                            keys=["name"]),
                            transitions={'out1': 'GET_OBJECT',
                                    'out0': 'out0'},
                            remapping={'name': 'name'})

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
                                remapping={'favorite_drink': 'favorite_drink'},
                                transitions={'out1': 'ADD_PERSON',
                                            'out0': 'out0'})
                                                
        smach.StateMachine.add('ADD_PERSON',
                                AddPerson(),
                                transitions={'out1': 'TURN_TO_COUCH', 
                                             'out0': 'out0'},
                                remapping={"age":"age",
                                        "hair_color":"hair_color",
                                        "shirt_color":"shirt_color",
                                        "name":"name",
                                        "favorite_drink":"favorite_drink",
                                        "people_list":"people_list",
                                        'people_index':'people_index'}
                                )
        
        smach.StateMachine.add('TURN_TO_COUCH',
                                 MoveTo(),
                                  transitions={'out1': 'SIT_TIGHT',
                                              'out0': 'out0'})
        
        smach.StateMachine.add('SIT_TIGHT',
                               Speak(text="Please have a seat. I'll be right back. This is our host Game. He likes tits."),
                            transitions = {'out1': 'GREETINGS_ASK_NAME_2',
                                            'out0':'out0'})
        
        smach.StateMachine.add('GREETINGS_ASK_NAME_2',
                            Speak(text="""Greetings, May I have your name please?"""),
                            # Speak(text="Please ."),
                            transitions={'out1': 'TURN_TO_PERSON_TWO',
                                            'out0': 'out0'})
        
        smach.StateMachine.add('TURN_TO_PERSON_TWO',
                               MoveTo(),
                                transitions={'out1': 'GET_NAME_2',
                                            'out0': 'out0'})

        smach.StateMachine.add('GET_NAME_2',
                            GetName(speak_debug=speak_debug,
                                    response_debug=response_debug,
                                    timeout=3),
                            transitions={'out1': 'SPEAK_ASK_OBJECT_2',
                                            'out0': 'out0'},
                            remapping={'listen_name': 'name'})
        
        smach.StateMachine.add('SPEAK_ASK_OBJECT_2',
                    Speak(text="Hello {}, What's your favorite drink?",
                        # Speak(text="Hello {}, favorite drink?",
                            keys=["name"]),
                            transitions={'out1': 'GET_OBJECT_2',
                                    'out0': 'out0'},
                            remapping={'name': 'name'})

        smach.StateMachine.add('GET_OBJECT_2',
                            GetObject(speak_debug=speak_debug,
                                        response_debug=response_debug,
                                        timeout=3),
                            transitions={'out1': 'SPEAK_RESPOND_OBJECT_2',
                                            'out0': 'out0'},
                            remapping={'listen_object': 'favorite_drink'})
        
        smach.StateMachine.add('SPEAK_RESPOND_OBJECT_2',
                                Speak(text="oh!, I like {} too",
                                    keys=["favorite_drink"]),
                                remapping={'favorite_drink': 'favorite_drink'},
                                transitions={'out1': 'ADD_PERSON_2',
                                            'out0': 'out0'})
                                                
        smach.StateMachine.add('ADD_PERSON_2',
                                AddPerson(),
                                transitions={'out1': 'TURN_TO_COUCH_2', 
                                             'out0': 'out0'},
                                remapping={"age":"age",
                                        "hair_color":"hair_color",
                                        "shirt_color":"shirt_color",
                                        "name":"name",
                                        "favorite_drink":"favorite_drink",
                                        "people_list":"people_list",
                                        'people_index':'people_index'}
                                )

    smach.StateMachine.add('TURN_TO_COUCH_2',
                                 MoveTo(),
                                  transitions={'out1': 'SIT_TIGHT',
                                              'out0': 'out0'})
        
    smach.StateMachine.add('SIT_TIGHT_2',
                               Speak(text="Please have a seat. I'll be right back. This is our host Game. He likes tits."),
                            transitions = {'out1': ''})





    from emerStopDumb import EmergencyStop
    es = EmergencyStop()
    import time
    # Create a thread to execute the smach container
    # Execute SMACH plan
    smach_thread = threading.Thread(target=sm.execute)
    smach_thread.start()

    es_thread = threading.Thread(target=es.execute)
    es_thread.start()
    
    import os

    while True:
        
        pid = os.getpid()
        if es.stop_flag:
            print("fuckkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkk")
            

            pid = pid  # Replace with your process id

            try:
                os.kill(pid, signal.SIGKILL)
            except ProcessLookupError:
                print(f"Process with id {pid} does not exist.")
            except PermissionError:
                print(f"You don't have permission to kill process with id {pid}.")
            break
        time.sleep(0.1)


    
if __name__ == '__main__':
    main()
