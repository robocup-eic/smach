#include "ros/ros.h"
#include "cr3_moveit_control/cr3_pick.h"
#include <boost/filesystem.hpp>
#include <jsoncpp/json/json.h>
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

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <vector>
//===============================================================
static const std::string PLANNING_GROUP_ARM = "arm";
bool success = false;
bool has_trial = false;
int number_of_trial = 7;

ros::Publisher gripper_command_publisher;
//===============================================================

// Generate Angle of Grasp Pose
// Calculation Algorithm
//========================================================================================
//========================================================================================
void compute_grasp(std::vector<geometry_msgs::Pose> &pose_array, const geometry_msgs::Pose pose, int trial)
{
  tf2::Quaternion q_orig, q_rot, q_new;
  tf2::convert(pose.orientation, q_orig);

  float yaw_array[7];
  // Algorithm from 0 +30 -30 +60 -60 +90 -90
  yaw_array[0] = 0;
  yaw_array[1] = 0 + float(M_PI) / 6.0;
  yaw_array[2] = 0 - float(M_PI) / 6.0;
  yaw_array[3] = 0 + float(M_PI) / 3.0;
  yaw_array[4] = 0 - float(M_PI) / 3.0;
  yaw_array[5] = 0 + float(M_PI) / 2.0;
  yaw_array[6] = 0 - float(M_PI) / 2.0;

  for (int i = 0; i < trial; i++)
  {
    q_rot.setRPY(0,0,yaw_array[i]);
    q_new = q_rot * q_orig;
    q_new.normalize();

    geometry_msgs::Pose new_pose = pose;
    tf2::convert(q_new, new_pose.orientation);
    pose_array.push_back(new_pose);
  }
}
//===========================================================================================
//===========================================================================================

void move_with_trial(std::vector<geometry_msgs::Pose> &pose_array, int trial)
{

  moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP_ARM);

  // Joint model group
  const moveit::core::JointModelGroup *joint_model_group = move_group_interface.getCurrentState()->getJointModelGroup(PLANNING_GROUP_ARM);
  moveit_visual_tools::MoveItVisualTools visual_tools("base_link");

  visual_tools.deleteAllMarkers();
  // set target pose
  geometry_msgs::Pose target_pose;
  success = false;

  // random the target pose
  //  to print an angle
  // int angle[7] = {0, 30, -30, 60, -60, 90, -90};

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  for (int i = 0; i < trial; i++)
  {
    move_group_interface.setPoseTarget(pose_array[i]);
    // ROS_INFO("target planning set order %d and %d degree done", number + 1, angle[number]);
    ROS_INFO("trial %d", i + 1);
    //=======================================================================================================//
    success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO("Plan %s", success ? "success" : "failure");
    //=======================================================================================================//
    // visualize plan
    visual_tools.publishAxisLabeled(target_pose, "pose");
    visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
    visual_tools.trigger();
    // TODO = fill try catch instead
    if (success == true)
    {
      break;
    }
  }

  bool success_execute =
      (move_group_interface.execute(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO("Execute %s", success_execute ? "success" : "failure");
  success = success && success_execute;
}

void move(geometry_msgs::Pose goal_pose) {
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

  move_group_interface.setPoseTarget(target_pose);
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO("Plan %s", success ? "success" : "failure");

  // visualize plan
  visual_tools.publishAxisLabeled(target_pose, "pose 1");
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();

  bool success_execute =
      (move_group_interface.execute(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO("Execute %s", success_execute ? "success" : "failure");
  success = success && success_execute;
}


void move_cartesian(geometry_msgs::Pose &current_pose, float x, float y, float z) {
  namespace rvt = rviz_visual_tools;
  moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP_ARM);
  moveit_visual_tools::MoveItVisualTools visual_tools("base_link");
  visual_tools.deleteAllMarkers();

  std::vector<geometry_msgs::Pose> waypoints;
  waypoints.push_back(current_pose);

  geometry_msgs::Pose target_pose = current_pose;
  target_pose.position.x += x;
  target_pose.position.y += y;
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

void set_home_walkie2(void)
{
  moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP_ARM);
  // Raw pointers are frequently used to refer to the planning group for improved performance.
  const robot_state::JointModelGroup* joint_model_group = move_group_interface.getCurrentState()->getJointModelGroup(PLANNING_GROUP_ARM);
  moveit::core::RobotStatePtr current_state = move_group_interface.getCurrentState();

  // Next get the current set of joint values for the group.
  std::vector<double> joint_group_positions;
  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

  moveit_visual_tools::MoveItVisualTools visual_tools("base_link");
  visual_tools.deleteAllMarkers();

  // Now, let's modify one of the joints, plan to the new joint space goal and visualize the plan.
  joint_group_positions[0] = 0.0;  // radians
  joint_group_positions[1] = 0.0;  // radians
  joint_group_positions[2] = 2.267;  // radians
  joint_group_positions[3] = 0.875;  // radians
  joint_group_positions[4] = 1.507;  // radians
  joint_group_positions[5] = 2.355;  // radians
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

bool pick_server(cr3_moveit_control::cr3_pick::Request &req,
                      cr3_moveit_control::cr3_pick::Response &res)
{

  double pos_x, pos_y, pos_z, ori_x, ori_y, ori_z, ori_w;
  pos_x = req.geo_req.position.x;
  pos_y = req.geo_req.position.y;
  pos_z = req.geo_req.position.z;
  ori_x = req.geo_req.orientation.x;
  ori_y = req.geo_req.orientation.y;
  ori_z = req.geo_req.orientation.z;
  ori_w = req.geo_req.orientation.w;
  ROS_INFO("request: x=%lf, y=%lf, z=%lf, ox=%lf, oy=%lf, oz=%lf, ow=%lf", pos_x, pos_y, pos_z, ori_x, ori_y, ori_z, ori_w);

  // pregrasp
  geometry_msgs::Pose pose;
  if (has_trial){
    //============= Declare Advanced Pointer for Calculation =====================
    pose.position.x = pos_x + 0.1; 
    pose.position.y = pos_y;
    pose.position.z = pos_z;
    pose.orientation.x = ori_x;
    pose.orientation.y = ori_y;
    pose.orientation.z = ori_z;
    pose.orientation.w = ori_w;

    // Initialized roll pith yawn angle
    std::vector<geometry_msgs::Pose> pose_array = {};
    compute_grasp(pose_array, pose, number_of_trial);

    ////////////// move group interface /////////////////
    move_with_trial(pose_array, number_of_trial);
  } else{
    pose.position.x = pos_x + 0.1; // plus with some offset
    pose.position.y = pos_y;
    pose.position.z = pos_z;
    pose.orientation.x = ori_x;
    pose.orientation.y = ori_y;
    pose.orientation.z = ori_z;
    pose.orientation.w = ori_w;

    move(pose);
  }
  if (!success){
    return false;
  }

  // open gripper
  std_msgs::Bool gripper_command_msg;
  gripper_command_msg.data = false;
  gripper_command_publisher.publish(gripper_command_msg);

  // grasps pose
  move_cartesian(pose, -0.1, 0, 0);
  if (!success){
    return false;
  }

  // close gripper
  gripper_command_msg.data = true;
  gripper_command_publisher.publish(gripper_command_msg);

  // lift
  pose.position.x = pos_x;

  move_cartesian(pose, 0, 0, 0.1);
  if (!success){
    return false;
  }

  // home
  //TODO ice#elec
  set_home_walkie2();

  // check whether it is true then return "success value"
  res.success_grasp = success;
  ROS_INFO(res.success_grasp ? "true" : "false");
  return true;
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "pick_service_server");
  ros::AsyncSpinner spinner(0);
  spinner.start();
  ros::NodeHandle nh;
  gripper_command_publisher = nh.advertise<std_msgs::Bool>("/cr3_gripper_command", 10);
  ros::ServiceServer service = nh.advertiseService("cr3_pick", pick_server);

  ros::waitForShutdown();
  return 0;
}
