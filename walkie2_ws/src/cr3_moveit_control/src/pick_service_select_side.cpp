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

// service
#include "cr3_moveit_control/PickWithSide.h"
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

// transform pose
void transform_pose(geometry_msgs::Pose &goal_pose){
  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf2_listener(tf_buffer);
  geometry_msgs::TransformStamped base_link_to_base_link_cr3;
  // get transform from base_link to cr3_base_link
  base_link_to_base_link_cr3 = tf_buffer.lookupTransform("base_link", "cr3_base_link", ros::Time(0), ros::Duration(1.0));

  geometry_msgs::PoseStamped robot_pose;
  robot_pose.pose.position.x = goal_pose.position.x;
  robot_pose.pose.position.y = goal_pose.position.y;
  robot_pose.pose.position.z = goal_pose.position.z;
  robot_pose.pose.orientation.x = goal_pose.orientation.x;
  robot_pose.pose.orientation.y = goal_pose.orientation.y;
  robot_pose.pose.orientation.z = goal_pose.orientation.z;
  robot_pose.pose.orientation.w = goal_pose.orientation.w;

  tf2::doTransform(robot_pose, robot_pose, base_link_to_base_link_cr3); // robot_pose is the PoseStamped I want to transform

  goal_pose = robot_pose.pose;

}

// Move the arm

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

void move_cartesian(geometry_msgs::Pose &current_pose, float x, float y, float z)
{
  namespace rvt = rviz_visual_tools;
  moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP_ARM);
  moveit_visual_tools::MoveItVisualTools visual_tools("base_link");
  visual_tools.deleteAllMarkers();

  std::vector<geometry_msgs::Pose> waypoints;
  waypoints.push_back(current_pose);

  geometry_msgs::Pose target_pose = current_pose;
  target_pose.position.x += -1*x; // cr3_base_link opposite with base_link
  target_pose.position.y += -1*y; // cr3_base_link opposite with base_link
  target_pose.position.z += z;
  waypoints.push_back(target_pose);

  moveit_msgs::RobotTrajectory trajectory;
  const double jump_threshold = 0.0;
  const double eef_step = 0.01;
  double fraction = move_group_interface.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

  // visualize plan
  visual_tools.deleteAllMarkers();
  visual_tools.publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);
  for (std::size_t i = 0; i < waypoints.size(); ++i)
    visual_tools.publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rvt::SMALL);

  bool success_execute = (move_group_interface.execute(trajectory) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO("Execute %s", success_execute ? "success" : "failure");
  success = success_execute;

  current_pose = target_pose;
}

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
  joint_group_positions[4] = 1.507; // radians
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

void retreat()
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
  joint_group_positions[4] = 0; // radians
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

// Pick execution
bool pick_server(cr3_moveit_control::PickWithSide::Request &req,
                 cr3_moveit_control::PickWithSide::Response &res)
{
  double pos_x, pos_y, pos_z, ori_x, ori_y, ori_z, ori_w;
  pos_x = req.geo_req.position.x;
  pos_y = req.geo_req.position.y;
  pos_z = req.geo_req.position.z;
  ori_x = req.geo_req.orientation.x;
  ori_y = req.geo_req.orientation.y;
  ori_z = req.geo_req.orientation.z;
  ori_w = req.geo_req.orientation.w;
  std::string side;
  side = req.pick_side;
  ROS_INFO("request: x=%lf, y=%lf, z=%lf, ox=%lf, oy=%lf, oz=%lf, ow=%lf", pos_x, pos_y, pos_z, ori_x, ori_y, ori_z, ori_w);
  ROS_INFO("the side to pick up: side=%s", side);
  // initialize the quaternion angle
  tf2::Quaternion q_orig, q_rot, q_new, q_gripper_rot;
  tf2::convert(req.geo_req.orientation, q_orig); // calcultate the q_orig from the requested geo_request orientation.

  // home
  set_home_walkie2();

  // Pregrasp
  geometry_msgs::Pose pose;
  std_msgs::Bool gripper_command_msg;
  res.success_grasp = success;
  
  if (side == "front")
  {
    // Cartesian coordinate
    pose.position.x = pos_x + PREGRASP_OFFSET;
    pose.position.y = pos_y;
    pose.position.z = pos_z;
    // Calculate angle
    q_rot.setRPY(FRONT_ORIENT[0], FRONT_ORIENT[1], FRONT_ORIENT[2]); // front in walkie2
    q_gripper_rot.setEuler(0, GRIPPER_ORIENT, 0);
    q_new = q_gripper_rot * q_rot * q_orig;
    q_new.normalize();
    geometry_msgs::Pose new_pose = pose;
    tf2::convert(q_new, new_pose.orientation);
    pose.orientation.x = new_pose.orientation.x;
    pose.orientation.y = new_pose.orientation.y;
    pose.orientation.z = new_pose.orientation.z;
    pose.orientation.w = new_pose.orientation.w;

    transform_pose(pose);
    move(pose);

    if (!success)
    {
      return false;
    }

    // open gripper
    gripper_command_msg.data = false;
    gripper_command_publisher.publish(gripper_command_msg);

    // grasps pose
    move_cartesian(pose, -1 * GRASP_OFFSET, 0, 0);
    if (!success)
    {
      return false;
    }

    // close gripper
    gripper_command_msg.data = true;
    gripper_command_publisher.publish(gripper_command_msg);

    // lift
    move_cartesian(pose, 0, 0, 0.1);
    if (!success)
    {
      return false;
    }

    // retreat
    retreat();
    if (!success)
    {
      return false;
    }

  }
  else if (side == "top")
  {
    // Cartesian coordinate
    pose.position.x = pos_x;
    pose.position.y = pos_y;
    pose.position.z = pos_z + PREGRASP_OFFSET;
    // Calculate angle
    q_rot.setRPY(TOP_ORIENT[0], TOP_ORIENT[1], TOP_ORIENT[2]);
    q_new = q_rot * q_orig;
    q_new.normalize();
    geometry_msgs::Pose new_pose = pose;
    tf2::convert(q_new, new_pose.orientation);
    pose.orientation.x = new_pose.orientation.x;
    pose.orientation.y = new_pose.orientation.y;
    pose.orientation.z = new_pose.orientation.z;
    pose.orientation.w = new_pose.orientation.w;
    move(pose);
    if (!success)
    {
      return false;
    }

    // open gripper
    gripper_command_msg.data = false;
    gripper_command_publisher.publish(gripper_command_msg);

    // grasps pose
    move_cartesian(pose, 0, 0, -1 * GRASP_OFFSET);
    if (!success)
    {
      return false;
    }

    // close gripper
    gripper_command_msg.data = true;
    gripper_command_publisher.publish(gripper_command_msg);

    // lift
    move_cartesian(pose, 0, 0, 0.1);
    if (!success)
    {
      return false;
    }
  }
  else if (side == "left")
  {
    // Cartesian coordinate
    pose.position.x = pos_x;
    pose.position.y = pos_y - PREGRASP_OFFSET;
    pose.position.z = pos_z;
    // Calculate angle
    q_rot.setRPY(LEFT_ORIENT[0], LEFT_ORIENT[1], LEFT_ORIENT[2]);
    q_new = q_rot * q_orig;
    q_new.normalize();
    geometry_msgs::Pose new_pose = pose;
    tf2::convert(q_new, new_pose.orientation);
    pose.orientation.x = new_pose.orientation.x;
    pose.orientation.y = new_pose.orientation.y;
    pose.orientation.z = new_pose.orientation.z;
    pose.orientation.w = new_pose.orientation.w;
    move(pose);
    if (!success)
    {
      return false;
    }

    // open gripper
    gripper_command_msg.data = false;
    gripper_command_publisher.publish(gripper_command_msg);

    // grasps pose
    move_cartesian(pose, 0, 1 * GRASP_OFFSET, 0);
    if (!success)
    {
      return false;
    }

    // close gripper
    gripper_command_msg.data = true;
    gripper_command_publisher.publish(gripper_command_msg);

    // lift
    move_cartesian(pose, 0, 0, 0.1);
    if (!success)
    {
      return false;
    }
  }
  else if (side == "right")
  {
    // Cartesian coordinate
    pose.position.x = pos_x;
    pose.position.y = pos_y + PREGRASP_OFFSET;
    pose.position.z = pos_z;
    // Calculate angle
    q_rot.setRPY(RIGHT_ORIENT[0], RIGHT_ORIENT[1], RIGHT_ORIENT[2]);
    q_new = q_rot * q_orig;
    q_new.normalize();
    geometry_msgs::Pose new_pose = pose;
    tf2::convert(q_new, new_pose.orientation);
    pose.orientation.x = new_pose.orientation.x;
    pose.orientation.y = new_pose.orientation.y;
    pose.orientation.z = new_pose.orientation.z;
    pose.orientation.w = new_pose.orientation.w;
    move(pose);
    if (!success)
    {
      return false;
    }

    // open gripper
    gripper_command_msg.data = false;
    gripper_command_publisher.publish(gripper_command_msg);

    // grasps pose
    move_cartesian(pose, 0, -1 * GRASP_OFFSET, 0);
    if (!success)
    {
      return false;
    }

    // close gripper
    gripper_command_msg.data = true;
    gripper_command_publisher.publish(gripper_command_msg);

    // lift
    move_cartesian(pose, 0, 0, 0.1);
    if (!success)
    {
      return false;
    }
  }
  else
  {
    return false;
  }

  // home
  set_home_walkie2();
  res.success_grasp = success;
  ROS_INFO(res.success_grasp ? "success_pick : true" : "success_pick : false");
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pick_service_select_side");
  ros::AsyncSpinner spinner(0);
  spinner.start();
  ros::NodeHandle nh;
  gripper_command_publisher = nh.advertise<std_msgs::Bool>("/cr3_gripper_command", 10);
  ros::ServiceServer service = nh.advertiseService("pick_service_select_side", pick_server);
  ros::waitForShutdown();
  return 0;
}
