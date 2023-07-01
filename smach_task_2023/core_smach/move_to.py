# State machine for move_to

import os
import signal
import roslib
import rospy
import smach
import smach_ros
import nlp_client
import threading
from ratfin import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import *
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Pose, Point, Quaternion
from navigate_SEX import Navi_SEX


def print_available_userdata(userdata):
    print(userdata)

class MoveTo(smach.State):
    """
    TemplateVersion 1.1.0
    MoveTo state

    BASIC FUNCTION:
    Move from A to B with CV to find object/person
    1. [NLP] room, object, furniture, person, furniture_adjective,
        - [room, object, furniture, person] one must not be null
    2. [NAVIGATION] Move to the specified location
    3. [COMPUTER VISION] Detect the object/person. If not detected, repeat step 2
    3.1 [NLP] Announce "Unable to detect the object/person. Please try again" --> Loop
    4. [NLP] Announce "I have reached the location" --> out1

    """
    def __init__(self,
                 log : bool = False,
                 timeout_tries: int = 0, # 0 means infinite tries
                 target: str = None
                 ):
        # Raise exceptions if any entity parameter is not of type bool
        if not isinstance(log, bool):
            raise ValueError("Argument 'log' must be of type bool")
        if target is None:
            raise ValueError("Argument 'target_pose' must not be None")

        rospy.init_node('nav_test', anonymous=False)
        rospy.on_shutdown(self.shutdown)
        #tell the action client that we want to spin a thread by default
        self.move_base = actionlib.SimpleActionClient("move_base",
                                                      MoveBaseAction)
        rospy.loginfo("wait for the action server to come up")
        #allow up to 5 seconds for the action server to come up
        self.move_base.wait_for_server(rospy.Duration(5))
        # Initialize the state
        smach.State.__init__(self,
                                outcomes=['out1','out0'],
                            #  outcomes=['out1','out2','loop','undo','timeout'],
                             input_keys=['room','furniture','data3'],
                             output_keys=['data1','data3'])

        # timout configuration, (don't change)
        if timeout_tries == 0:
            self.timeout_bool = False
        else:
            self.timeout_bool = True
        self.tries_counter = int(0) # counter for the number of tries
        self.timeout_tries = timeout_tries
        self.target_pose = Navi_SEX().read_yaml(target)
        

    def execute(self, userdata):
        if self.target is None:
            rospy.logerr("Target not found")
            return 'out0'
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.get_rostime()
        goal.target_pose.pose = self.target_pose
        self.move_base.send_goal(goal)
        print("Waiting for result")
        success = self.move_base.wait_for_result(rospy.Duration(60))
        print("Return result: ", success)
        state = self.move_base.get_state()
        result = False
        if success and state == GoalStatus.SUCCEEDED:
            # We made it!
            result = True
        else:
            self.move_base.cancel_goal()
            self.goal_sent = False
        return result

    def shutdown(self):
        stop_goal = MoveBaseGoal()
        self.move_base.send_goal(stop_goal)
        rospy.loginfo("Stop")