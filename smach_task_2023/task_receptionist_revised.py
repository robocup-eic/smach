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
from core_nlp.utils import (WakeWord, Speak, GetIntent, GetName, GetObject, GetLocation,)
from core_smach.move_to import MoveTo
from core_cv.image_captioning import ImageCaption


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
    def __init__(self,people_index : int = 0, header_text : str = "", introduce_to : int = None):
        smach.State.__init__(self,
                            outcomes=['out1'],
                            input_keys=['people_list', 'people_index'],
                            output_keys=['people_index'])
        self.index = people_index
        self.header_text = header_text
        self.introduce_to = introduce_to
    def execute(self, userdata):

        # Log the execution stage
        rospy.loginfo(f'(IntroducePeople): Executing..')

        # Extract people from userdata
        person : Person = userdata.people_list[self.index]
        age_txt = f' {person.age}'
        if person.age == 'young':
            age_txt = 'younger than 20'
        elif person.age == 'elderly':
            age_txt = 'older than 60'
        # Contruct text to speak
        text : str = f"this is {person.name}, their favorite drink is {person.favorite_drink}, they are {age_txt} years old, they are wearing a {person.shirt_color} shirt, and they have {person.hair_color} hair."
        
        introduce_to_txt = ""
        if self.introduce_to is not None:
            introduce_to_txt = f"Hey {userdata['people_list'][self.introduce_to].name}, "
        # Speak the text
        nlp_client.speak(text=introduce_to_txt+self.header_text+text)

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
    timeout = 3

    host = Person(name="Game",
                    # favorite_drink="milkies from your mom's titties",
                    favorite_drink='human blood',
                    age=20,
                    shirt_color="black",
                    hair_color="black",
                    gender='male',
                    race='asian',
                    glasses=False)
    sm.userdata.people_list.append(host)   

    with sm:
        smach.StateMachine.add('GREETINGS_ASK_NAME',
                            Speak(text="""Greetings, May I have your name please?"""),
                            # Speak(text="Please ."),
                            transitions={'out1': 'TURN_TO_PERSON_ONE',
                                            'out0': 'out0'})
        
        smach.StateMachine.add('TURN_TO_PERSON_ONE',
                               MoveTo(),
                                transitions={'out1': 'GET_NAME'})
                               
        smach.StateMachine.add('GET_NAME',
                            GetName(speak_debug=speak_debug,
                                    response_debug=response_debug,
                                    timeout=timeout),
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
                                        timeout=timeout),
                            transitions={'out1': 'SPEAK_RESPOND_OBJECT',
                                            'out0': 'out0'},
                            remapping={'listen_object': 'favorite_drink'})
        
        smach.StateMachine.add('SPEAK_RESPOND_OBJECT',
                                Speak(text="oh!, I like {} too! Can you stay still for a second so I can memorize your details?",
                                    keys=["favorite_drink"]),
                                remapping={'favorite_drink': 'favorite_drink'},
                                transitions={'out1': 'IMAGE_CAPTION',
                                            'out0': 'out0'})
        
        smach.StateMachine.add('IMAGE_CAPTION',
                               ImageCaption(),
                               remapping = {'age':'age',
                                            'shirt_color':'shirt_color',
                                            'hair_color':'hair_color',
                                            'gender':'gender',
                                            'race':'race',
                                            'wearing_glasses':'wearing_glasses'},
                                transitions={'out1': 'ADD_PERSON',
                                             'undo':'out0'})
                                                
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
                                  transitions={'out1': 'SIT_TIGHT'})
        
        smach.StateMachine.add('SIT_TIGHT',
                               Speak(text="Please have a seat."),
                            transitions = {'out1': 'INTRODUCE_HOST',
                                            'out0':'out0'})
        
        smach.StateMachine.add('INTRODUCE_HOST',
                               IntroducePeople(people_index=0,header_text='I am going to introduce you to our host     '),
                               transitions = {'out1': 'TURN_TO_HOST'})

        smach.StateMachine.add('TURN_TO_HOST',
                                 MoveTo(),
                                  transitions={'out1': 'INTRODUCE_PERSON_1'})

        smach.StateMachine.add('INTRODUCE_PERSON_1',
                               IntroducePeople(people_index=1,introduce_to=0),
                               transitions = {'out1': 'GREETINGS_ASK_NAME_2'})
        
        smach.StateMachine.add('GREETINGS_ASK_NAME_2',
                            Speak(text="""Greetings, May I have your name please?"""),
                            # Speak(text="Please ."),
                            transitions={'out1': 'TURN_TO_PERSON_TWO',
                                            'out0': 'out0'})
        
        smach.StateMachine.add('TURN_TO_PERSON_TWO',
                               MoveTo(),
                                transitions={'out1': 'GET_NAME_2'})

        smach.StateMachine.add('GET_NAME_2',
                            GetName(speak_debug=speak_debug,
                                    response_debug=response_debug,
                                    timeout=timeout),
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
                                        timeout=timeout),
                            transitions={'out1': 'SPEAK_RESPOND_OBJECT_2',
                                            'out0': 'out0'},
                            remapping={'listen_object': 'favorite_drink'})
        
        smach.StateMachine.add('SPEAK_RESPOND_OBJECT_2',
                                Speak(text="oh!, I like {} too! Can you stay still for a second so I can memorize your details?",
                                    keys=["favorite_drink"]),
                                remapping={'favorite_drink': 'favorite_drink'},
                                transitions={'out1': 'IMAGE_CAPTION_2',
                                            'out0': 'out0'})

        smach.StateMachine.add('IMAGE_CAPTION_2',
                               ImageCaption(),
                               remapping = {'age':'age',
                                            'shirt_color':'shirt_color',
                                            'hair_color':'hair_color',
                                            'gender':'gender',
                                            'race':'race',
                                            'wearing_glasses':'wearing_glasses'},
                                transitions={'out1': 'ADD_PERSON_2',
                                             'undo':'out0' })
                                                

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
                                    transitions={'out1': 'SIT_TIGHT_2'})
            
        smach.StateMachine.add('SIT_TIGHT_2',
                                Speak(text="Please have a seat."),
                                transitions = {'out1': 'INTRODUCE_HOST_2'})

        smach.StateMachine.add('INTRODUCE_HOST_2',
                               IntroducePeople(people_index=0,header_text='I am going to introduce you to our host       '),
                               transitions = {'out1': 'TURN_TO_HOST_2'})

        smach.StateMachine.add('TURN_TO_HOST_2',
                                 MoveTo(),
                                  transitions={'out1': 'INTRODUCE_PERSON_2'})
        
        smach.StateMachine.add('INTRODUCE_PERSON_2',
                               IntroducePeople(people_index=2,introduce_to=0),
                               transitions = {'out1': 'TURN_TO_PERSON_TWO_2'})
        
        smach.StateMachine.add('TURN_TO_PERSON_TWO_2',
                               MoveTo(),
                                transitions={'out1': 'SPEAK_INTRODUCE_TO_EACH_OTHER'})

        smach.StateMachine.add('SPEAK_INTRODUCE_TO_EACH_OTHER',
                               Speak(text="Cool! now I am going to introduce you to each other "),
                               transitions = {'out1': 'INTRODUCE_TO_EACH_OTHER_1'})
        
        smach.StateMachine.add('INTRODUCE_TO_EACH_OTHER_1',
                               IntroducePeople(people_index=1,introduce_to=2),
                               transitions = {'out1': 'TURN_TO_PERSON_ONE_2'}) 
        
        smach.StateMachine.add('TURN_TO_PERSON_ONE_2',
                                 MoveTo(),
                                  transitions={'out1': 'INTRODUCE_TO_EACH_OTHER_2'})
        
        smach.StateMachine.add('INTRODUCE_TO_EACH_OTHER_2',
                                 IntroducePeople(people_index=2,introduce_to=1),
                                 transitions = {'out1': 'out1'})
        
        
               




    from core_nlp.emerStop import EmergencyStop
    es = EmergencyStop()
    import time
    # Create a thread to execute the smach container
    # Execute SMACH plan
    smach_thread = threading.Thread(target=sm.execute)
    smach_thread.start()

    es = EmergencyStop()
    es_thread = threading.Thread(target=es.execute)
    es_thread.start()
    es.emer_stop_handler()



    
if __name__ == '__main__':
    main()
