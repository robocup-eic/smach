# cr3 command
from codecs import raw_unicode_escape_decode
from tkinter import Y
import moveit_commander
import rospy
from geometry_msgs.msg import Pose, Quaternion
from tf.transformations import quaternion_from_euler
from visualization_msgs.msg import Marker
from moveit_msgs.msg import DisplayTrajectory


def set_home_walkie():
            group_name = "arm"
            move_group = moveit_commander.MoveGroupCommander(group_name)
            joint_goal = move_group.get_current_joint_values()
            print(joint_goal)
            joint_goal[0] = 0.0
            joint_goal[1] = 0.0
            joint_goal[2] = 2.267
            joint_goal[3] = 0.875
            joint_goal[4] = 3.14
            joint_goal[5] = 2.355
            
            
            # joint_goal[0] = 0.0
            # joint_goal[1] = 0.0
            # joint_goal[2] = 0.0
            # joint_goal[3] = 0.0
            # joint_goal[4] = 0.0
            # joint_goal[5] = 0.0
            move_group.go(joint_goal, wait=True)
            print(joint_goal)
            move_group.stop()

def go_to_pose_goal():
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    a =rospy.Publisher("posem",Marker,queue_size=1)
    group_name = "arm"
    move_group = moveit_commander.MoveGroupCommander(group_name)
    robot = moveit_commander.RobotCommander()
    display_trajectory_publisher = rospy.Publisher(
                                    '/move_group/display_planned_path',
                                    DisplayTrajectory, queue_size=1)

    ## BEGIN_SUB_TUTORIAL plan_to_pose
    ##
    ## Planning to a Pose Goal
    ## ^^^^^^^^^^^^^^^^^^^^^^^
    ## We can plan a motion for this group to a desired pose for the
    ## end-effector:
    pose_goal = Pose()
    q = quaternion_from_euler(0,1.57,0)
    pose_goal.orientation = Quaternion(*q)
    # pose_goal.orientation.w = -0.704
    # pose_goal.orientation.z = 0.704
    # pose_goal.orientation.w = -0.5

    pose_goal.position.x = 0.5
    pose_goal.position.y = -0.2
    pose_goal.position.z = 0.9
    move_group.set_planner_id("Informed RRT*")
    move_group.set_planning_time(5)
    move_group.set_pose_target(pose_goal)
    m = Marker()
    m.header.frame_id = "base_footprint"
    m.header.stamp = rospy.Time.now()
    m.type = Marker.ARROW
    m.action = Marker.ADD
    m.pose = pose_goal
    m.scale.x = 0.1
    m.scale.y = 0.05
    m.scale.z = 0.05
    m.color.a = 1
    m.color.r = 1
    m.color.g = 0
    m.color.b = 0

  
    rospy.sleep(5)
    a.publish(m)

    plan = move_group.plan()

    display_trajectory = DisplayTrajectory()
    display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory.append(plan)
    display_trajectory_publisher.publish(display_trajectory)
    rospy.sleep(5)

    raw_input(move_group.get_planner_id())

    move_group.execute(plan, wait=True)


    ## Now, we call the planner to compute the plan and execute it.



# set_home_walkie()
rospy.init_node("dwdwdw")
if __name__=='__main__':
  try:
    go_to_pose_goal()
  except rospy.ROSInterruptException:
    pass