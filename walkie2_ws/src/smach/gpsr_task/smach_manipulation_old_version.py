#!/usr/bin/env python

import roslib
import rospy
import smach
import smach_ros

# import for Pick_client
from geometry_msgs.msg import Pose
from tf.transformations import quaternion_from_euler
import math
from cr3_moveit_control.srv import * # dont forget to import file into the folder





class Standby(smach.State):
    def __init__(self):
        rospy.loginfo('Initiating state Standby')
        smach.State.__init__(self,outcomes=['continue_save_data'])

    def execute(self, userdata):
        rospy.loginfo('Executing state Standby')

        return 'continue_save_data'


class Save_data(smach.State):
    def __init__(self):
        rospy.loginfo('Initiating state Save_data')
        smach.State.__init__(self,outcomes=['continue_navigation'])

    def execute(self, userdata):
        rospy.loginfo('Executing state Save_data')
        return 'continue_navigation'


class Navigation(smach.State):
    def __init__(self):
        rospy.loginfo('Initiating state Navigation')
        smach.State.__init__(self,outcomes=['continue_get_pose'])

    def execute(self, userdata):
        rospy.loginfo('Executing state Navigation')
        return 'continue_get_pose'


class Get_pose(smach.State):
    def __init__(self):
        rospy.loginfo('Initiating state Get_pose')
        smach.State.__init__(self,outcomes=['continue_pick_client'])
       
    
    def execute(self, userdata):
        rospy.loginfo('Executing state Get_pose')

        return 'continue_pick_client'


class Pick_client(smach.State):
    def __init__(self):
        rospy.loginfo('Initiating state Pick_client')
        #smach.State.__init__(self,outcomes=['continue_something'])
        # for testing purpose
        smach.State.__init__(self,outcomes=['continue_aborted'])
        #grasp_pose = (x,y,z,row,pitch,yaw)
        grasp_pose = (1,1,1,1,1,1)
        
    def execute(self, userdata):
        rospy.loginfo('Executing state Pick_client')

        def pick_service(goal_pose, pick_side):
            rospy.wait_for_service('pick_gen2_success')
            try:
                pick = rospy.ServiceProxy('pick_gen2_success', pick_gen2)
                res = pick(goal_pose, pick_side)
                return res.success_grasp
            except rospy.ServiceException as e:
                print("Service call failed: %s"%e)

        # # pick side depends on the object shape which we will write and if-else condition to seperate it.
        # # pose_goal is the one that i got from get pose state.
        # finished_picking_object = pick_service(pose_goal,pick_side)
        # if finished_picking_object == True:
        #     return 'continue_aborted'
        # if finished_picking_object == False:
        #     return 'continue_aborted'
        q = quaternion_from_euler(row, pitch, yaw) # these row pitch yaw recieve from Get_pose state
        pose_goal = Pose()
        pose_goal.orientation.x = q[0]
        pose_goal.orientation.y = q[1]
        pose_goal.orientation.z = q[2]
        pose_goal.orientation.w = q[3]

        pose_goal.position.x = X # these X Y Z recieve from Get_pose state
        pose_goal.position.y = Y
        pose_goal.position.z = Z

        pick_done = pick_service(pose_goal,"Top")

        if pick_done == True:
            return 'continue_aborted'
        else:
            # do something when the picking process is not complete
            return 'continie_aborted'


def main():
    rospy.init_node('smach_GPSR')
    sm = smach.StateMachine(outcomes=['Aborted'])

    with sm:
        smach.StateMachine.add('Standby',Standby(),transitions={'continue_save_data':'Save_data'})
        smach.StateMachine.add('Save_data',Save_data(),transitions={'continue_navigation':'Navigation'})
        smach.StateMachine.add('Navigation',Navigation(),transitions={'continue_get_pose':'Get_pose'})
        smach.StateMachine.add('Get_pose', Get_pose(), transitions={'continue_pick_client'})
        smach.StateMachine.add('Pick_client',Pick_client(),transitions={'continue_to_aborted':'Aborted'})

        #sis = smach_ros.IntrospectionServer('Patter',sm,'/ROOT')
        #sis.start()

        outcome = sm.execute()

        #rospy.spin()
        #sis.stop()

if __name__ == '__main__':
    main()