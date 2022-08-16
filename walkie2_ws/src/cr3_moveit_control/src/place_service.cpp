#include "ros/ros.h"
#include "cr3_moveit_control/cr3_place.h"
#include <boost/filesystem.hpp>
#include <jsoncpp/json/json.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
// #include <ros/time.h>
 #include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/Marker.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <geometry_msgs/Pose.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <vector>

#include <stdio.h>
// #include <algorithm>
// #include <unistd.h>
// #include <cmath>

//===============================================================
static const std::string PLANNING_GROUP_ARM = "arm";
bool success = false;

ros::Publisher gripper_command_publisher;
ros::Publisher marker_pub;
//===============================================================

float to_rad(float deg)
{
  return deg * M_PI / 180.0;
}

void move(geometry_msgs::Pose &POSITION) {
  moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP_ARM);
  const moveit::core::JointModelGroup *joint_model_group = move_group_interface.getCurrentState()->getJointModelGroup(PLANNING_GROUP_ARM);

  moveit_visual_tools::MoveItVisualTools visual_tools("base_footprint");
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
  target_pose1.position.y = POSITION.position.y + 0.05;
  target_pose1.position.z = POSITION.position.z;

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  
  move_group_interface.setPoseTarget(target_pose1);
  success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

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
  moveit_visual_tools::MoveItVisualTools visual_tools("base_footprint");
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

  moveit_visual_tools::MoveItVisualTools visual_tools("base_footprint");
  visual_tools.deleteAllMarkers();

  // Now, let's modify one of the joints, plan to the new joint space goal and visualize the plan.
  joint_group_positions[0] = 0.0;  // radians
  joint_group_positions[1] = 0.0;  // radians
  joint_group_positions[2] = 2.267;  // radians
  joint_group_positions[3] = 0.875;  // radians
  joint_group_positions[4] = 3.142;  // radians
  // joint_group_positions[4] = 0.0;  // radians
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
  collision_objects[0].header.frame_id = "cr3_base_link";

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
  collision_objects[0].header.frame_id = "cr3_base_link";

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
  pose.position.y = 0.0;
  pose.position.z = high + 0.15;

  myQuaternion.setRPY( -1 * M_PI / 2, -1 * M_PI / 4, M_PI / 2 );
  pose.orientation.x = myQuaternion.getX();
  pose.orientation.y = myQuaternion.getY();
  pose.orientation.z = myQuaternion.getZ();
  pose.orientation.w = myQuaternion.getW();

  // transform base_footprint to base_footprint
  ROS_INFO_STREAM(pose);
  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf2_listener(tf_buffer);
  geometry_msgs::TransformStamped base_footprint_to_base_footprint;

  base_footprint_to_base_footprint = tf_buffer.lookupTransform("base_footprint", "cr3_base_link", ros::Time(0), ros::Duration(1.0) );

  tf2::doTransform(pose, pose, base_footprint_to_base_footprint); // robot_pose is the PoseStamped I want to transform
  ROS_INFO_STREAM(pose);

  // pub marker
  visualization_msgs::Marker marker;
  marker.header.frame_id = "/base_footprint";
  marker.header.stamp = ros::Time::now();
  marker.type = visualization_msgs::Marker::ARROW;;
  marker.action = visualization_msgs::Marker::ADD;
  
  // marker.pose.position.x = pose.position.x;
  // marker.pose.position.y = pose.position.y;
  // marker.pose.position.z = pose.position.z;
  // marker.pose.orientation.x = pose.orientation.x;
  // marker.pose.orientation.y = pose.orientation.y;
  // marker.pose.orientation.z = pose.orientation.z;
  // marker.pose.orientation.w = pose.orientation.w;
  marker.pose = pose;
  // Set the scale of the marker -- 1x1x1 here means 1m on a side
  // marker.scale.x = 0.1;
  // marker.scale.y = 0.1;
  // marker.scale.z = 0.1;
  marker.scale.x = 1.0;
  marker.scale.y = 1.0;
  marker.scale.z = 1.0;

  marker.lifetime = ros::Duration();
  // Publish the marker
  marker_pub.publish(marker);

  ROS_INFO_STREAM("move to preplace pose");
  // move to preplace pose
  move(pose);
  if (!success){
    return false;
  }

  // ROS_INFO_STREAM("move cartesian along z axis");
  // // scanf("%d", &a);
  // // move cartesian along z axis
  // move_cartesian(pose, 0, 0, -0.1);
  // if (!success){
  //   return false;
  // }

  // ROS_INFO_STREAM("open gripper");
  // // scanf("%d", &a);szxdcf vgbhn j
  // // open gripper
  // std_msgs::Bool gripper_command_msg;
  // gripper_command_msg.data = false;
  // gripper_command_publisher.publish(gripper_command_msg);

  // ROS_INFO_STREAM("move cartesian out of object");
  // // scanf("%d", &a);
  // // move cartesian out of object
  // move_cartesian(pose, 0.1, 0, 0.1);
  // if (!success){
  //   return false;
  // }

  // ROS_INFO_STREAM("close gripper");
  // // scanf("%d", &a);
  // // close gripper
  // gripper_command_msg.data = true;
  // gripper_command_publisher.publish(gripper_command_msg);

  // ROS_INFO_STREAM("set home walkie2");
  // // scanf("%d", &a);
  // // set home walkie2
  // set_home_walkie2();

  // check whether it is true then return "success value"
  res.success_place = success;
  ROS_INFO(res.success_place ? "true" : "false");
  return true;
}

int main(int argc, char** argv){

  ros::init(argc, argv, "place_service_server");
  ros::AsyncSpinner spinner(0);
  spinner.start();
  ros::NodeHandle nh;
  gripper_command_publisher = nh.advertise<std_msgs::Bool>("/cr3_gripper_command", 10);
  marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  ros::ServiceServer service = nh.advertiseService("cr3_place", place_server);
  ros::waitForShutdown();
  return 0;
}