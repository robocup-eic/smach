#include "ros/ros.h"
#include "cr3_moveit_control/cr3_place.h"
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

// #include <stdio.h>
// #include <algorithm>
// #include <unistd.h>
// #include <cmath>

//===============================================================
static const std::string PLANNING_GROUP_ARM = "arm";
bool success = false;

ros::Publisher gripper_command_publisher;
//===============================================================

float to_rad(float deg)
{
  return deg * M_PI / 180.0;
}

double distance(geometry_msgs::Pose target_pose1, geometry_msgs::Pose current_collision_object_pos)
{
  double x = abs(target_pose1.position.x - current_collision_object_pos.position.x);
  double y = abs(target_pose1.position.y - current_collision_object_pos.position.y);
  return sqrt(pow(x, 2) + pow(y, 2));
}

double smallest_distance(geometry_msgs::Pose target_pose1, std::vector<geometry_msgs::Pose> current_collision_object_pos)
{
  int number_of_collision_object = current_collision_object_pos.size();
  double all_distance[number_of_collision_object];
  ROS_INFO_STREAM(number_of_collision_object);

  for (int i=0;i<number_of_collision_object;i++)
  { all_distance[i] = distance(target_pose1, current_collision_object_pos[i]); }

  std::sort(all_distance, all_distance + number_of_collision_object);
  ROS_INFO_STREAM(all_distance[0]);
  return all_distance[0];
}

void move(geometry_msgs::Pose &POSITION, std::vector<geometry_msgs::Pose> current_collision_object_pos, geometry_msgs::Pose &MID_TABLE) {
  moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP_ARM);
  const moveit::core::JointModelGroup *joint_model_group = move_group_interface.getCurrentState()->getJointModelGroup(PLANNING_GROUP_ARM);

  moveit_visual_tools::MoveItVisualTools visual_tools("base_cr3_joint");
  visual_tools.deleteAllMarkers();
  geometry_msgs::Pose target_pose1;
  //Step1 Execute along X-Y coordinate
  bool success = false;

  //construct quaternion angle
  tf2::Quaternion quat;
  quat.setRPY(-M_PI / 2, -M_PI / 4, M_PI / 2);
  target_pose1.orientation.w = quat.getW();
  target_pose1.orientation.x = quat.getX();
  target_pose1.orientation.y = quat.getY();
  target_pose1.orientation.z = quat.getZ();
  target_pose1.position.x = POSITION.position.x;
  target_pose1.position.y = POSITION.position.y;
  target_pose1.position.z = POSITION.position.z;

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  geometry_msgs::Pose base_link_pose;

  float max_table_y = target_pose1.position.y;
  while(!success)
  {
    target_pose1.position.y += 0.05;
    double distance_between_obj = smallest_distance(target_pose1, current_collision_object_pos);
    if((0.1 < distance_between_obj) && (distance(base_link_pose, target_pose1) < 0.63))
    {
      move_group_interface.setPoseTarget(target_pose1);
      success= (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    }

    if(target_pose1.position.y > max_table_y)
    {
      target_pose1 = MID_TABLE;
      move_group_interface.setPoseTarget(target_pose1);
      success= (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    }
  }

  visual_tools.publishAxisLabeled(target_pose1, "pose 1");
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();

  bool success_execute = (move_group_interface.execute(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO("Execute %s", success_execute ? "success" : "failure");
  success = success && success_execute;

  POSITION = target_pose1;
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
  // joint_group_positions[4] = 1.507;  // radians
  joint_group_positions[4] = 3.14;  // radians
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

void addCollisionObject( float dimension_radius, float dimension_high,
                        float position_x, float position_y, float position_z , std::string object )
{
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  // create vector to hold 1 object
  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.resize(1);

  // collision_table
  collision_objects[0].id = object;
  collision_objects[0].header.frame_id = "base_cr3_joint";

  // collsion_table dimension
  collision_objects[0].primitives.resize(1);
  collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].CYLINDER;
  collision_objects[0].primitives[0].dimensions.resize(2);
  collision_objects[0].primitives[0].dimensions[0] = dimension_high;
  collision_objects[0].primitives[0].dimensions[1] = dimension_radius;
  

  // collsion_table poses
  collision_objects[0].primitive_poses.resize(1);
  collision_objects[0].primitive_poses[0].position.x = position_x;
  collision_objects[0].primitive_poses[0].position.y = position_y;
  collision_objects[0].primitive_poses[0].position.z = position_z;
  
  collision_objects[0].operation = collision_objects[0].ADD;

  planning_scene_interface.applyCollisionObjects(collision_objects);
}

void addCollisionTable( float dimension_x, float dimension_y, float dimension_z,
                        float position_x, float position_y, float position_z , std::string object )
{
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  // create vector to hold 1 object
  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.resize(1);

  // collision_table
  collision_objects[0].id = object;
  collision_objects[0].header.frame_id = "base_cr3_joint";

  // collsion_table dimension
  collision_objects[0].primitives.resize(1);
  collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
  collision_objects[0].primitives[0].dimensions.resize(3);
  collision_objects[0].primitives[0].dimensions[0] = dimension_x;
  collision_objects[0].primitives[0].dimensions[1] = dimension_y;
  collision_objects[0].primitives[0].dimensions[2] = dimension_z;

  // collsion_table poses
  collision_objects[0].primitive_poses.resize(1);
  collision_objects[0].primitive_poses[0].position.x = position_x;
  collision_objects[0].primitive_poses[0].position.y = position_y;
  collision_objects[0].primitive_poses[0].position.z = position_z;
  
  collision_objects[0].operation = collision_objects[0].ADD;

  planning_scene_interface.applyCollisionObjects(collision_objects);
}

void removeCollisionObject(std::string object)
{
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  // create vector to hold 1 object
  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.resize(1);

  // collision_table
  collision_objects[0].id = object;
  collision_objects[0].operation = collision_objects[0].REMOVE;

  planning_scene_interface.applyCollisionObjects(collision_objects);
}

bool place_server(cr3_moveit_control::cr3_place::Request &req,
                      cr3_moveit_control::cr3_place::Response &res)
{
  set_home_walkie2();

  // calculate collision table and space to place
  // *table11-----------------------------table12
  // |                                          |
  // |                                          |
  // |   place11----------------------*place12  |
  // |   |                                  |   |
  // |   |                                  |   |
  // |   |                                  |   |
  // |   |                                  |   |
  // |   *place12----------------------place22  |
  // |                                          |
  // |                                          |
  // table21-----------------------------*table22
  //
  //                     z ---> y
  //                     | 
  //                     v
  //                     x
    // x axis
  // move cartesian along z axis
    // condition --> corner.x are in -x axis
  double creat_collision_table11_x, creat_collision_table12_x, creat_collision_table21_x, creat_collision_table22_x;
  double place_within11_x, place_within12_x, place_within21_x, place_within22_x;

  double calculate_collision_table_x[4] = {req.corner11.x, req.corner12.x, req.corner21.x, req.corner22.x};
  std::sort(calculate_collision_table_x, calculate_collision_table_x + 4);
  creat_collision_table11_x = calculate_collision_table_x[0];
  creat_collision_table12_x = calculate_collision_table_x[0];
  creat_collision_table21_x = calculate_collision_table_x[3];
  creat_collision_table22_x = calculate_collision_table_x[3];

  place_within11_x = calculate_collision_table_x[1];
  place_within12_x = calculate_collision_table_x[1];
  place_within21_x = calculate_collision_table_x[2];
  place_within22_x = calculate_collision_table_x[2];

    // y axis
    // condition --> corner.y 2 point are in -y axis and another point are in +y axis
  double creat_collision_table11_y, creat_collision_table12_y, creat_collision_table21_y, creat_collision_table22_y;
  double place_within11_y, place_within12_y, place_within21_y, place_within22_y;

  double calculate_collision_table_y[4] = {req.corner11.y, req.corner12.y, req.corner21.y, req.corner22.y};
  std::sort(calculate_collision_table_y, calculate_collision_table_y + 4);
  creat_collision_table11_y = calculate_collision_table_y[0];
  creat_collision_table12_y = calculate_collision_table_y[3];
  creat_collision_table21_y = calculate_collision_table_y[0];
  creat_collision_table22_y = calculate_collision_table_y[3];

  place_within11_y = calculate_collision_table_y[1];
  place_within12_y = calculate_collision_table_y[2];
  place_within21_y = calculate_collision_table_y[1];
  place_within22_y = calculate_collision_table_y[2];

    // z axis
    // condition --> 0 < z
    double high;
    high = req.high.z;

  // add collsion table
  double dimension_x, dimension_y, dimension_z;
  double position_x, position_y, position_z;

  dimension_x = abs(creat_collision_table11_x - creat_collision_table21_x);
  dimension_y = abs(creat_collision_table11_y - creat_collision_table12_y);
  dimension_z = high;

  position_x = (creat_collision_table11_x + creat_collision_table21_x) / 2;
  position_y = (creat_collision_table21_y + creat_collision_table22_y) / 2;
  position_z = dimension_z / 2;

  addCollisionTable(dimension_x, dimension_y, dimension_z,
                    position_x, position_y, position_z, "collision table1");

  // add collsion object
  for (int i=0;i<req.collision_object_pos.size();i++)
  {
    addCollisionObject(0.05, 0.2,
                      req.collision_object_pos[i].position.x,                //position x of collision object
                      req.collision_object_pos[i].position.y,                //position y of collision object 
                      dimension_z + 0.1,                                     //position z of collision object
                      "collision object" + boost::to_string(i));             //collision object name
  }

  // find pose to preplace
  geometry_msgs::Pose pose;
  tf2::Quaternion myQuaternion;

  pose.position.x = place_within21_x - 0.1;
  pose.position.y = place_within21_y;
  pose.position.z = high + 0.15;

  myQuaternion.setRPY( -1 * M_PI / 2, -1 * M_PI / 4, M_PI / 2 );
  pose.orientation.x = myQuaternion.getX();
  pose.orientation.y = myQuaternion.getY();
  pose.orientation.z = myQuaternion.getZ();
  pose.orientation.w = myQuaternion.getW();

  geometry_msgs::Pose mid_table;

  mid_table.position.x = place_within21_x - 0.1;
  mid_table.position.y = (place_within21_y + place_within22_y) / 2.0;
  mid_table.position.z = high + 0.25;
  mid_table.orientation.x = myQuaternion.getX();
  mid_table.orientation.y = myQuaternion.getY();
  mid_table.orientation.z = myQuaternion.getZ();
  mid_table.orientation.w = myQuaternion.getW();
  
  // move to preplace pose
  move(pose, req.collision_object_pos, mid_table);
  if (!success){
    return false;
  }
  
  if(pose != mid_table)
  {
    // move cartesian along z axis
    move_cartesian(pose, 0, 0, -0.1);
    if (!success){
      return false;
    }

    // open gripper
    std_msgs::Bool gripper_command_msg;
    gripper_command_msg.data = false;
    gripper_command_publisher.publish(gripper_command_msg);

    // move cartesian out of object
    move_cartesian(pose, 0.1, 0, 0.1);
    if (!success){
      return false;
  }
  else
  {
    // open gripper
    std_msgs::Bool gripper_command_msg;
    gripper_command_msg.data = false;
    gripper_command_publisher.publish(gripper_command_msg);
  }

    // close gripper
  gripper_command_msg.data = true;
  gripper_command_publisher.publish(gripper_command_msg);
  
  // set home walkie2
  set_home_walkie2();

  // check whether it is true then return "success value"
  res.success_place = success;
  ROS_INFO(res.success_place ? "true" : "false");
  return true;
  }
}

int main(int argc, char** argv){

  ros::init(argc, argv, "place_service_server");
  ros::AsyncSpinner spinner(0);
  spinner.start();
  ros::NodeHandle nh;
  gripper_command_publisher = nh.advertise<std_msgs::Bool>("/cr3_gripper_command", 10);
  ros::ServiceServer service = nh.advertiseService("cr3_place", place_server);
  ros::waitForShutdown();
  return 0;
}