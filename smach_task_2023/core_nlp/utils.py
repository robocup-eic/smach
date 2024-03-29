
# run with conda env: nlp
import roslib
import rospy
import smach
import threading
import smach_ros
from nlp_client import *
from ratfin import *
from geometry_msgs.msg import Twist, Vector3
from rospy import Publisher, init_node, Rate




def prompt_user_repeat():
    speak("Sorry I didn't get that. Please rephrase that?")

# define state speak




class WakeWord(smach.State):
    """ 
    return "out1" if wakeword is detected
    >>> outcome map: {'WAKEWORD_DETECTED': 'out1'}
     
       """
    def __init__(self):
        smach.State.__init__(self, outcomes=['out1'])

    def execute(self, userdata):
        rospy.loginfo('(WakeWord): Listening for Hey Walkie')
        ww_listen()
        rospy.loginfo('WakeWord detected')
        return "out1"


class Speak(smach.State):
    """ 
    speak hello to Person1 
      """
    def __init__(self, 
                 text: str, # text to speak
                 keys = None, # replace {} with userdata keys
                 person_name_to_track : str  = None, # track the listener while speaking
                 response_debug :  bool = False):
        """  
        >>> Speak(text="Hello {}, What's your favorite drink?", keys=["name"]) 

        >>> Speak(text="Hello {}, What's your favorite drink?", 
                  keys=["name"], 
                  track_listener= "Sharon"
                  )

                            """
        if keys is None:
            keys = []

        smach.State.__init__(self, outcomes=['out1', 'out0'], input_keys=keys)

        self.response_debug = response_debug
        self.text = text
        self.keys = keys
        self.person_name_to_track = person_name_to_track

    def person_tracker(self, 
                       userdata, 
                       person_name_to_track,
                       continous: bool = True ):
        """ 
        track the person_name_to_track
        """
        CAMERA_RESOLUTION = [] # width, height
        CENTER_TOLERANCE = 0.36 # percentage of the frame from the center

        # bounderies of the center
        X1 = int(CAMERA_RESOLUTION[0]*0.5 - 0.36*0.5*CAMERA_RESOLUTION[0])
        X2 = int(CAMERA_RESOLUTION[0]*0.5 + 0.36*0.5*CAMERA_RESOLUTION[0])


        self.centered = False # for outside checking.


        # init node for publishing to cmd_vel
        rospy.init_node('person_tracker_nlp+cv')
        cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        while True:
            try:
                # request to CV to track the person_name_to_track that matches the name

                # CV return the list of names and [X1,Y1,X2,Y2]
                cv_response = cv_track_person(person_name_to_track)
                list_of_detections = cv_response.detections

                # if the person is not detected, then don't move
                for detection in list_of_detections:
                    if detection.name == person_name_to_track:
                        detection_midpoint = (detection.coordinates[2] - detection.coordinates[0])/2
                        # if the person is centered, break
                        if detection_midpoint > X1 and detection_midpoint < X2:
                            self.centered = True
                            # stop the robot
                            cmd_vel_pub.publish(Twist())
                        else:
                            # the person is not centered, move the robot
                            self.centered = False

                            # maybe add a speed curve here
                            angular_speed = 0.1

                            # if not centered, move the robot
                            if detection_midpoint < X1:
                                cmd_vel_pub.publish(Twist(linear=Vector3(0.0, 0.0, 0.0), angular=Vector3(z=angular_speed)))
                            elif detection_midpoint > X2:
                                cmd_vel_pub.publish(Twist(linear=Vector3(0.0, 0.0, 0.0), angular=Vector3(z=-angular_speed)))

            except Exception as e:
                printclr(e, "red")
                self.centered = False
                break   

    def execute(self, userdata):
        try:
            rospy.loginfo(f'(Speak): Executing..')


            # Prepare arguments for the format string from userdata
            args = [getattr(userdata, key)
                    for key in self.keys] if self.keys else []


            # Check if there's a person to track
            if self.person_name_to_track is not None:
                # create an instance thread to continously track the person
                listener_tracker = threading.Thread(
                    target=self.person_tracker, args=(userdata,person_name_to_track))
                listener_tracker.start()
            
                while True: 
                    if listener_tracker.centered:
                        break

                
            # Prepare the text to speak
            text = self.text.format(*args)

            rospy.loginfo(f'Speaking : {text}')

            # speak the intent
            speak(text)
    
            # kill the listener thread
            listener_tracker.join()


            return "out1"
        except Exception as e:
            printclr(e, "red")
            return "out0"


class GetEntities(smach.State):
    """ 
    smach.StateMachine.add('GET_ENTITIES',
                           GetEntities(intent=True, name=True, object=False, location=False,
                                                       speak_debug=speak_debug,
                                                       response_debug=response_debug,
                                                       timeout=2),
                           transitions={'out1': 'NEXT_STATE',
                                        'out0': 'END'},
                           remapping={'listen_intent': 'intent',
                                      'listen_name': 'name',
                                      'listen_object': 'object',
                                      'listen_location': 'location'})
    """

    def __init__(self,
                 intent: bool = False,
                 name: bool = False,
                 object: bool = False,
                 location: bool = False,
                 confidence: bool = False,
                 speak_debug: bool = False,
                 response_debug: bool = False,
                 speak_repeat: bool = False,
                 
                 timeout=0):

        # Raise exceptions if any entity parameter is not of type bool
        if not isinstance(intent, bool):
            raise ValueError("Argument 'intent' must be of type bool")
        if not isinstance(name, bool):
            raise ValueError("Argument 'name' must be of type bool")
        if not isinstance(object, bool):
            raise ValueError("Argument 'object' must be of type bool")
        if not isinstance(location, bool):
            raise ValueError("Argument 'location' must be of type bool")
        if not isinstance(confidence, bool):
            raise ValueError("Argument 'confidence' must be of type bool")
        if not isinstance(speak_debug, bool):
            raise ValueError("Argument 'speak_debug' must be of type bool")
        if not isinstance(response_debug, bool):
            raise ValueError("Argument 'response_debug' must be of type bool")
        if not isinstance(timeout, int):
            raise ValueError("Argument 'timeout' must be of type integer")

        # Init class variables
        self.speak_debug = speak_debug
        self.response_debug = response_debug
        self.listen_counter = int(0)
        self.timeout = timeout
        self.speak_repeat = speak_repeat
        self.valid_out = False  # check if the output is valid

        # adding enities to extract 
        self.attributes = []

        if intent:
            self.attributes.append('intent')
        if name:
            self.attributes.append('name')
        if object:
            self.attributes.append('object')
        if location:
            self.attributes.append('location')

        # timout config
        if timeout == 0:
            self.timeout_bool = False
        else:
            self.timeout_bool = True

        # Smach Parameters
        smach.State.__init__(self,
                             outcomes=['out1', 'out0'],
                             input_keys=['listen_' +
                                         a for a in self.attributes],
                             output_keys=['listen_' + a for a in self.attributes])

    def execute(self, userdata):
        try:
            # Log the execution stage
            rospy.loginfo(f'(GetEntities): Executing..')

            # Minimum confidence
            min_confidence = 0.5

            while (self.listen_counter < self.timeout) or not self.timeout_bool:

                # Increment the counter
                self.listen_counter += 1

                # Log the execution stage
                rospy.loginfo(
                    f'(GetEntities): [{self.listen_counter}]Listening..')

                # listen for user
                res_obj = listen(intent=True)

                # print the response object if debug
                if self.response_debug:
                    print(res_obj)

                # Check the object & confidence for each attribute
                for a in self.attributes:
                    attr_value = getattr(res_obj, a, "")
                    if attr_value != "" and res_obj.confidence > min_confidence:
                        setattr(userdata, 'listen_'+a, attr_value)
                        self.valid_out = True
                    else: # If any attribute is not found, break
                        self.valid_out = False
                        if self.speak_repeat: 
                            prompt_user_repeat()
                        


                # If any valid attribute is found, break
                if self.valid_out:
                    break

            if not self.valid_out:
                if self.speak_debug:
                    speak("(GetEntities): No attribute detected in the timeout period")
                raise Exception(
                    "(GetEntities): No attribute detected in the timeout period")

            return "out1"

        except Exception as e:
            printclr(e, "red")
            return "out0"


# define state GetIntent
class GetIntent(smach.State):
    """ 
    smach.StateMachine.add('GET_INTENT',
                               GetIntent(speak_debug=speak_debug,
                                         response_debug=response_debug,
                                         timeout=2),
                               transitions={'out1': 'GET_NAME',
                                            'out0': 'END'},
                               remapping={'listen_intent': 'intent'})
    """

    def __init__(self,
                 speak_debug=False,
                 response_debug=False,
                 speak_repeat=True,
                 timeout=0):

        # Init class variables
        self.speak_debug = speak_debug
        self.response_debug = response_debug
        self.listen_counter = int(0)
        self.timeout = timeout
        self.speak_repeat = speak_repeat
        self.valid_out = False  # check if the output is valid

        if timeout == 0:
            self.timeout_bool = False
        else:
            self.timeout_bool = True

        # Smach Parameters
        smach.State.__init__(self,
                             outcomes=['out1', 'out0'],
                             input_keys=['listen_intent', 'listen_text'],
                             output_keys=['listen_intent', 'listen_text'])

    def execute(self, userdata):
        try:
            # Log the execution stage
            rospy.loginfo(f'(GetIntent): Executing..')

            # Minimum confidence
            min_confidence = 0.5

            while (self.listen_counter < self.timeout) or not self.timeout_bool:

                # Increment the counter
                self.listen_counter += 1

                # Log the execution stage
                rospy.loginfo(
                    f'(GetIntent): [{self.listen_counter}]Listening..')

                # listen for user
                # res_obj = listen(intent=True)
                res_obj = listen(intent=True)

                # print the response object if debug
                if self.response_debug:
                    print(res_obj)

                # TODO Have it be content aware?

                # Check the object & confidence
                if (res_obj.intent != "") and (res_obj.confidence > min_confidence):
                    self.valid_out = True
                    break

                # Ask the user to repeat if not valid
                if self.speak_repeat:
                    prompt_user_repeat()

            if not self.valid_out:
                if self.speak_debug:
                    speak("(GetIntent): No intent detected in the timeout period")
                raise Exception(
                    "(GetIntent): No intent detected in the timeout period")

            # TODO check if object is in the database
            # TODO if not, ask for the object
            # TODO check if it's what the user wants, check if null

            # Store intent in userdata for later use
            userdata.listen_intent = res_obj.intent

            # Store text in userdata for later use
            userdata.listen_text = res_obj.text

            # Log the intent
            rospy.loginfo(f'(GetIntent): {userdata.listen_intent}')

            # speak the intent if debug
            if self.speak_debug:
                speak(f'(GetIntent): {userdata.listen_intent}')

            return "out1"
        except Exception as e:
            printclr(e, "red")
            return "out0"


class GetName(smach.State):
    """ 
    smach.StateMachine.add('GET_NAME',
                               GetName(speak_debug=speak_debug,
                                       response_debug=response_debug,
                                       timeout=2),
                               transitions={'out1': 'GET_OBJECT',
                                            'out0': 'END'},
                               remapping={'listen_name': 'name'})
    """

    def __init__(self,
                 speak_debug=False,
                 response_debug=False,
                 speak_repeat=True,
                 timeout=0):

        # Init class variables
        self.speak_debug = speak_debug
        self.response_debug = response_debug
        self.listen_counter = int(0)
        self.counter = 0
        self.timeout = timeout
        self.speak_repeat = speak_repeat
        self.valid_out = False  # check if the output is valid

        if timeout == 0:
            self.timeout_bool = False
        else:
            self.timeout_bool = True

        # Smach Parameters
        smach.State.__init__(self,
                             outcomes=['out1', 'out0'],
                             output_keys=["listen_name"])

    def execute(self, userdata):
        try:
            # Log the execution stage
            rospy.loginfo(f'(GetName): Executing..')

            # Minimum confidence
            min_confidence = 0.5

            while (self.listen_counter < self.timeout) or not self.timeout_bool:

                # Increment the counter
                self.listen_counter += 1

                # Log the execution stage
                rospy.loginfo(f'(GetName): [{self.listen_counter}]Listening..')

                # listen for user
                res_obj = listen(intent=True)

                # print the response object if debug
                if self.response_debug:
                    print(res_obj)

                # TODO Have it be content aware?

                # Check the object & confidence
                if (res_obj.people != "") and (res_obj.confidence > min_confidence):
                    self.valid_out = True
                    break
                if self.speak_repeat:
                    prompt_user_repeat()

            if not self.valid_out:
                if self.speak_debug:
                    speak("(GetName): No name detected in the timeout period")
                raise Exception(
                    "(GetName): No name detected in the timeout period")

            # Store intent in userdata for later use
            name = res_obj.people
            userdata.listen_name = res_obj.people

            # Log the name
            rospy.loginfo(f'(GetName): {name}')

            # speak the name if debug
            if self.speak_debug:
                speak(f'(GetName): {name}')

            return "out1"
        except Exception as e:
            printclr(e, "red")
            return "out0"


class GetObject(smach.State):
    """ 
    smach.StateMachine.add('GET_OBJECT',
                               GetObject(speak_debug=speak_debug,
                                         response_debug=response_debug,
                                         timeout=2),
                               transitions={'out1': 'GET_LOCATION',
                                            'out0': 'END'},
                               remapping={'listen_object': 'object'})
    """

    def __init__(self,
                 speak_debug=False,
                 response_debug=False,
                 speak_repeat=True,
                 timeout=0):

        # Init class variables
        self.speak_debug = speak_debug
        self.response_debug = response_debug
        self.listen_counter = int(0)
        self.timeout = timeout
        self.speak_repeat = speak_repeat
        self.valid_out = False  # check if the output is valid

        if timeout == 0:
            self.timeout_bool = False
        else:
            self.timeout_bool = True

        # Smach Parameters
        smach.State.__init__(self,
                             outcomes=['out1', 'out0'],
                             input_keys=['listen_object'],
                             output_keys=["listen_object"])

        self.counter = 0

    def execute(self, userdata):
        try:

            # Log the execution stage
            rospy.loginfo(f'(GetObject): Executing..')

            # Minimum confidence
            min_confidence = 0.5

            while (self.listen_counter < self.timeout) or not self.timeout_bool:

                # Increment the counter
                self.listen_counter += 1

                # Log the execution stage
                rospy.loginfo(
                    f'(GetObject): [{self.listen_counter}]Listening..')

                # listen for user
                res_obj = listen(intent=True)

                # print the response object if debug
                if self.response_debug:
                    print(res_obj)

                # TODO Have it be content aware?

                # Check the object & confidence
                if (res_obj.object != "") and (res_obj.confidence > min_confidence):
                    self.valid_out = True
                    break
                if self.speak_repeat:
                    prompt_user_repeat()

            if not self.valid_out:
                if self.speak_debug:
                    speak("(GetObject): No object detected in the timeout period")
                raise Exception(
                    "(GetObject): No object detected in the timeout period")

            # Store intent in userdata for later use
            userdata.listen_object = res_obj.object

            # Log the name
            rospy.loginfo(f'(GetObject): {userdata.listen_object}')

            # speak the object if debug
            if self.speak_debug:
                speak(f'(GetObject): {userdata.listen_object}')

            return "out1"
        except Exception as e:
            printclr(e, "red")
            return "out0"


class GetLocation(smach.State):
    """ 
    smach.StateMachine.add('GET_LOCATION',
                               GetLocation(speak_debug=speak_debug,
                                           response_debug=response_debug,
                                           timeout=2),
                               transitions={'out1': 'END',
                                            'out0': 'END'},
                               remapping={'listen_location': 'location'})
    """

    def __init__(self,
                 speak_debug=False,
                 response_debug=False,
                 speak_repeat=True,
                 timeout=0):

        # Init class variables
        self.speak_debug = speak_debug
        self.response_debug = response_debug
        self.listen_counter = int(0)
        self.timeout = timeout
        self.speak_repeat = speak_repeat
        self.valid_out = False  # check if the output is valid

        if timeout == 0:
            self.timeout_bool = False
        else:
            self.timeout_bool = True

        # Smach Parameters
        smach.State.__init__(self,
                             outcomes=['out1', 'out0'],
                             input_keys=['listen_location'],
                             output_keys=["listen_location"])

        self.counter = 0

    def execute(self, userdata):
        try:

            # Log the execution stage
            rospy.loginfo(f'(GetLocation): Executing..')

            # Minimum confidence
            min_confidence = 0.5

            while (self.listen_counter < self.timeout) or not self.timeout_bool:

                # Increment the counter
                self.listen_counter += 1

                # Log the execution stage
                rospy.loginfo(
                    f'(GetLocation): [{self.listen_counter}]Listening..')

                # listen for user
                res_obj = listen(intent=True)

                # print the response object if debug
                if self.response_debug:
                    print(res_obj)

                # TODO Have it be content aware?

                # Check the object & confidence
                if (res_obj.room != "") and (res_obj.confidence > min_confidence):
                    self.valid_out = True
                    break
                if self.speak_repeat:
                    prompt_user_repeat()

            if not self.valid_out:
                if self.speak_debug:
                    speak("(GetLocation): No location detected in the timeout period")
                raise Exception(
                    "(GetLocation): No location detected in the timeout period")

            # Store intent in userdata for later use
            userdata.listen_location = res_obj.room

            # Log the name
            rospy.loginfo(f'(GetLocation): {userdata.listen_location}')

            # speak the location if debug
            if self.speak_debug:
                speak(f'(GetLocation): {userdata.listen_location}')

            return "out1"
        except Exception as e:
            printclr(e, "red")
            return "out0"


def main():
    speak_debug = False
    response_debug = False

    rospy.init_node('utils_nlp')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['END'])
    sm.userdata.intent = ""
    sm.userdata.name = ""
    sm.userdata.object = ""
    sm.userdata.location = ""

    # open the container
    with sm:
        smach.StateMachine.add('GET_ENTITIES',
                               GetEntities(intent=True,
                                           object=True,
                                           speak_debug=speak_debug,
                                           response_debug=response_debug,
                                           timeout=2),
                               transitions={'out1': 'END',
                                            'out0': 'END'},
                               remapping={'listen_intent': 'intent',
                                          'listen_name': 'name',
                                          'listen_object': 'object',
                                          'listen_location': 'location'})
        # smach.StateMachine.add('WAKEWORD',
        #                        WakeWord(),
        #                        transitions={'out1':'SPEAK'})

        # smach.StateMachine.add('SPEAK',
        #                        Speak(text="Hello, what can I do for you?",
        #                              response_debug=response_debug),
        #                        transitions={'out1': 'GET_INTENT',
        #                                     'out0': 'END'})

        # smach.StateMachine.add('GET_INTENT',
        #                        GetIntent(speak_debug=speak_debug,
        #                                  response_debug=response_debug,
        #                                  timeout=2),
        #                        transitions={'out1': 'GET_NAME',
        #                                     'out0': 'END'},
        #                        remapping={'listen_intent': 'intent'})

        # smach.StateMachine.add('GET_NAME',
        #                        GetName(speak_debug=speak_debug,
        #                                response_debug=response_debug,
        #                                timeout=2),
        #                        transitions={'out1': 'GET_OBJECT',
        #                                     'out0': 'END'},
        #                        remapping={'listen_name': 'name'})

        # smach.StateMachine.add('GET_OBJECT',
        #                        GetObject(speak_debug=speak_debug,
        #                                  response_debug=response_debug,
        #                                  timeout=2),
        #                        transitions={'out1': 'GET_LOCATION',
        #                                     'out0': 'END'},
        #                        remapping={'listen_object': 'object'})

        # smach.StateMachine.add('GET_LOCATION',
        #                        GetLocation(speak_debug=speak_debug,
        #                                    response_debug=response_debug,
        #                                    timeout=2),
        #                        transitions={'out1': 'END',
        #                                     'out0': 'END'},
        #                        remapping={'listen_location': 'location'})

    # Execute SMACH plan
    outcome = sm.execute()
    print(sm.userdata.__dict__["_data"])


if __name__ == '__main__':
    main()
