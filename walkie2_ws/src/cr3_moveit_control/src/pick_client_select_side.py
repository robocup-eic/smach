#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Pose
from tf.transformations import quaternion_from_euler
import math
from cr3_moveit_control.srv import *

def to_rad(deg):
    return deg * math.pi / 180.0

def pick_service(goal_pose, pick_side):
    rospy.wait_for_service('pick_service_select_side')
    try:
        pick = rospy.ServiceProxy('pick_service_select_side', PickWithSide)
        res = pick(goal_pose, pick_side)
        return res.success_grasp
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

if __name__ == "__main__":
    while not rospy.is_shutdown():
        x = input("X ")
        y = input("y ")
        z = input("z ")
        row = to_rad(input("row (default 0) "))
        pitch = to_rad(input("pitch (default 0) "))
        yaw = to_rad(input("yaw (default 0) "))
        q = quaternion_from_euler(row, pitch, yaw)
        pick_side_input = raw_input("side ( front, top, left, right) :")
        pose_goal = Pose()
        pose_goal.orientation.x = q[0]
        pose_goal.orientation.y = q[1]
        pose_goal.orientation.z = q[2]
        pose_goal.orientation.w = q[3]

        pose_goal.position.x = x
        pose_goal.position.y = y
        pose_goal.position.z = z
        pick_side = pick_side_input
        print(pick_side_input)
        #pose_goal.pick_side = pick_side
        rospy.loginfo(pose_goal, pick_side)
        success = pick_service(pose_goal, pick_side)
        print(success)
