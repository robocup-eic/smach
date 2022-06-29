#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Pose
from tf.transformations import quaternion_from_euler
import math
from cr3_moveit_control.srv import *

def to_rad(deg):
    return deg * math.pi / 180.0

def pick_service(goal_pose):
    rospy.wait_for_service('cr3_pick')
    try:
        pick = rospy.ServiceProxy('cr3_pick', cr3_pick)
        res = pick(goal_pose)
        return res.success_grasp
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

if __name__ == "__main__":
    while not rospy.is_shutdown():
        x = input("X ")
        y = input("y ")
        z = input("z ")
        row = to_rad(input("row (default -90) "))
        pitch = to_rad(input("pitch (default -45) "))
        yaw = to_rad(input("yaw (default 90) "))
        # -M_PI / 2, -M_PI / 4, +M_PI / 2
        # row = -1*math.pi/2
        # pitch = -1*math.pi/4
        # yaw = math.pi/2
        q = quaternion_from_euler(row, pitch, yaw)

        pose_goal = Pose()
        pose_goal.orientation.x = q[0]
        pose_goal.orientation.y = q[1]
        pose_goal.orientation.z = q[2]
        pose_goal.orientation.w = q[3]

        pose_goal.position.x = x
        pose_goal.position.y = y
        pose_goal.position.z = z

        rospy.loginfo(pose_goal)
        success = pick_service(pose_goal)
        print(success)
