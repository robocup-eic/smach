# for testing the smach2023 package, do not do other shit in here bitches

from utils import WakeWord, rospy,roslib,smach,smach_ros

if __name__ == '__main__':
    rospy.init_node('test_nlp')
    sm = smach.StateMachine(outcomes=['END'])
    with sm:
        smach.StateMachine.add('WAKEWORD',
                               WakeWord(),
                               transitions={'out1':'END'})
    outcome = sm.execute()