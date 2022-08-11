# cr3 command
import moveit_commander
import rospy

def set_home_walkie():
            group_name = "arm"
            move_group = moveit_commander.MoveGroupCommander(group_name)
            joint_goal = move_group.get_current_joint_values()
            print(joint_goal)
            # joint_goal[0] = 0.0
            # joint_goal[1] = 0.0
            # joint_goal[2] = 2.267
            # joint_goal[3] = 0.875
            # joint_goal[4] = 3.14
            # joint_goal[5] = 2.355
            joint_goal[0] = 0.0
            joint_goal[1] = 0.0
            joint_goal[2] = 0.0
            joint_goal[3] = 0.0
            joint_goal[4] = 0.0
            joint_goal[5] = 2.355
            move_group.go(joint_goal, wait=True)
            print(joint_goal)
            move_group.stop()

set_home_walkie()