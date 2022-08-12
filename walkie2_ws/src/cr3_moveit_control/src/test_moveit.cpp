#include "ros/ros.h"
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <ros/ros.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <geometry_msgs/Pose.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

// include quaternian transformation
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <string>
#include <vector>

//===============================================================
static const std::string PLANNING_GROUP_ARM = "arm";
bool success = false;
const double PREGRASP_OFFSET = 0.20; // cr3 link6 to end of gripper offset
const double GRASP_OFFSET = 0.10;
ros::Publisher gripper_command_publisher;
const double FRONT_ORIENT[3] = {0, -M_PI/2, 0};
const double TOP_ORIENT[3] = {0, -M_PI, -M_PI/4};
const double LEFT_ORIENT[3] = {0, -M_PI / 2.0, -M_PI / 2.0};
const double RIGHT_ORIENT[3] = {0, -M_PI / 2.0, M_PI / 2.0};
const double GRIPPER_ORIENT = -M_PI / 4.0; // Please use -M_PI / 4.0
//===========================================================================================

void set_home_walkie2()
{
  moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP_ARM);
  // Raw pointers are frequently used to refer to the planning group for improved performance.
  const robot_state::JointModelGroup *joint_model_group = move_group_interface.getCurrentState()->getJointModelGroup(PLANNING_GROUP_ARM);
  moveit::core::RobotStatePtr current_state = move_group_interface.getCurrentState();

  // Next get the current set of joint values for the group.
  std::vector<double> joint_group_positions;
  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

  moveit_visual_tools::MoveItVisualTools visual_tools("cr3_base_link");
  visual_tools.deleteAllMarkers();

  // Now, let's modify one of the joints, plan to the new joint space goal and visualize the plan.
  joint_group_positions[0] = 0.0;   // radians
  joint_group_positions[1] = 0.0;   // radians
  joint_group_positions[2] = 2.267; // radians
  joint_group_positions[3] = 0.875; // radians
  joint_group_positions[4] = 2.966; // radians
  joint_group_positions[5] = 2.355; // radians
  move_group_interface.setJointValueTarget(joint_group_positions);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO("Plan %s", success ? "success" : "failure");

  // visualize plan
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();

  bool success_execute =
      (move_group_interface.execute(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO("Execute %s", success_execute ? "success" : "failure");
  success = success && success_execute;
}

int main(void)
{
    set_home_walkie2();
}



void move(geometry_msgs::Pose goal_pose)
{
  moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP_ARM);
  const moveit::core::JointModelGroup *joint_model_group = move_group_interface.getCurrentState()->getJointModelGroup(PLANNING_GROUP_ARM);

  moveit_visual_tools::MoveItVisualTools visual_tools("base_link");
  visual_tools.deleteAllMarkers();
  // set target pose
  geometry_msgs::Pose target_pose;
  target_pose.orientation.w = goal_pose.orientation.w;
  target_pose.orientation.x = goal_pose.orientation.x;
  target_pose.orientation.y = goal_pose.orientation.y;
  target_pose.orientation.z = goal_pose.orientation.z;
  target_pose.position.x = goal_pose.position.x;
  target_pose.position.y = goal_pose.position.y;
  target_pose.position.z = goal_pose.position.z;

  ROS_INFO_STREAM(target_pose);
  move_group_interface.setPoseTarget(target_pose);
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO("Plan %s", success ? "success" : "failure");

  // visualize plan
  visual_tools.publishAxisLabeled(target_pose, "pose 1");
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();

  bool success_execute = (move_group_interface.execute(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO("Execute %s", success_execute ? "success" : "failure");
  success = success && success_execute;
}