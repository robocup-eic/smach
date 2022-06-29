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
#include <vector>

static const std::string PLANNING_GROUP_ARM = "arm";
static const std::string APP_DIRECTORY_NAME = ".cr3_simulation";

static const std::vector<double> OBJECT_POSITION = {0.5, 0, 0.5};
const double tau = 2 * M_PI;

void kan_place(moveit::planning_interface::MoveGroupInterface &move_group_interface, double* POSITION) {
  const moveit::core::JointModelGroup *joint_model_group =
    move_group_interface.getCurrentState()->getJointModelGroup(PLANNING_GROUP_ARM);

  moveit_visual_tools::MoveItVisualTools visual_tools("base_link");
  visual_tools.deleteAllMarkers();
  geometry_msgs::Pose target_pose1;
  //Step1 Execute along X-Y coordinate
  bool success = false;
  ROS_INFO("INIITIATED STEP1");

  //construct quaternion angle
  tf2::Quaternion quat;
  quat.setRPY(-M_PI / 2, -M_PI / 4, -M_PI / 2);
  target_pose1.orientation.w = quat.getW();
  target_pose1.orientation.x = quat.getX();
  target_pose1.orientation.y = quat.getY();
  target_pose1.orientation.z = quat.getZ();
  target_pose1.position.x = POSITION[0];
  target_pose1.position.y = POSITION[1];
  target_pose1.position.z = POSITION[2]+0.3;
  move_group_interface.setPoseTarget(target_pose1);
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO("Plan %s", success ? "success" : "failure");
  visual_tools.publishAxisLabeled(target_pose1, "pose 1");
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();
  //Step2 Execute along Z coordinate

  ROS_INFO("INITIATED STEP2");
  geometry_msgs::Pose target_pose2;
  target_pose2.orientation.w = quat.getW();
  target_pose2.orientation.x = quat.getX();
  target_pose2.orientation.y = quat.getY();
  target_pose2.orientation.z = quat.getZ();
  target_pose2.position.x = POSITION[0];
  target_pose2.position.y = POSITION[1];
  target_pose2.position.z = POSITION[2]+0.05; //Prevent the collision
  move_group_interface.setPoseTarget(target_pose2);  
  success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO("Plan %s", success ? "success" : "failure");

  visual_tools.publishAxisLabeled(target_pose2, "pose2");
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();
}

void addCollisionObjects(moveit::planning_interface::PlanningSceneInterface &planning_scene_interface) {
  // create vector to hold 3 object
  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.resize(2);

  // table1
  collision_objects[0].id = "table1";
  collision_objects[0].header.frame_id = "base_link";

  // table1 dimension
  collision_objects[0].primitives.resize(1);
  collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
  collision_objects[0].primitives[0].dimensions.resize(3);
  collision_objects[0].primitives[0].dimensions[0] = 0.2;
  collision_objects[0].primitives[0].dimensions[1] = 0.4;
  collision_objects[0].primitives[0].dimensions[2] = 0.4;

  // table1 poses
  collision_objects[0].primitive_poses.resize(1);
  collision_objects[0].primitive_poses[0].position.x = 0.5;
  collision_objects[0].primitive_poses[0].position.y = 0;
  collision_objects[0].primitive_poses[0].position.z = 0.2;

  collision_objects[0].operation = collision_objects[0].ADD;

  // object
  collision_objects[1].header.frame_id = "base_link";
  collision_objects[1].id = "object1";

  collision_objects[1].primitives.resize(1);
  collision_objects[1].primitives[0].type = collision_objects[1].primitives[0].BOX;
  collision_objects[1].primitives[0].dimensions.resize(3);
  collision_objects[1].primitives[0].dimensions[0] = 0.02;
  collision_objects[1].primitives[0].dimensions[1] = 0.02;
  collision_objects[1].primitives[0].dimensions[2] = 0.2;

  collision_objects[1].primitive_poses.resize(1);
  collision_objects[1].primitive_poses[0].position.x = OBJECT_POSITION[0];
  collision_objects[1].primitive_poses[0].position.y = OBJECT_POSITION[1];
  collision_objects[1].primitive_poses[0].position.z = OBJECT_POSITION[2];

  collision_objects[1].operation = collision_objects[1].ADD;

  planning_scene_interface.applyCollisionObjects(collision_objects);
}

int main(int argc, char** argv){
  namespace fs = boost::filesystem;
  ROS_INFO("RUNNING robot_control_node");

  ros::init(argc, argv, "robot_placing_node");
  ROS_INFO("dahvfbvskjvbkfvbhk");
  ros::NodeHandle nh;
  ros::Publisher gripper_command_publisher = nh.advertise<std_msgs::Bool>("/gripper_command", 10);

  ros::AsyncSpinner spinner(1);
  spinner.start();
  // declare interface for moveit
  ROS_INFO("This is the trap");
  moveit::planning_interface::MoveGroupInterface move_group_arm(PLANNING_GROUP_ARM);
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  ROS_INFO("This is the trap2");
  ros::Publisher planning_scene_diff_publisher = nh.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
  ROS_INFO("Fishfishfishfishfish");
  // rviz visual and moveit visual
  moveit_visual_tools::MoveItVisualTools visual_tools("base_link");
  visual_tools.deleteAllMarkers();

  ros::WallDuration sleep_t(0.5);
  while (planning_scene_diff_publisher.getNumSubscribers() < 1) {
    sleep_t.sleep();
  }
  moveit_msgs::PlanningScene planning_scene;

  // read JSON files from ~/.cr3_simulation
  fs::path home(getenv("HOME"));
  if (fs::is_directory(home)) {
    fs::path app_directory(home);
    app_directory /= APP_DIRECTORY_NAME;

    if (!fs::exists(app_directory) && !fs::is_directory(app_directory)) {
      ROS_WARN_STREAM(app_directory << " does not exist");

      // Create .cr3_simulation directory
      std::string path(getenv("HOME"));
      path += "/.cr3_simulation";
      ROS_INFO("Creating %s collision objects directory.", path);
      try {
        boost::filesystem::create_directory(path);
      } catch (const std::exception &) {
        ROS_ERROR("%s directory could not be created."
                  "Please create this directory yourself "
                  "if you want to specify collision objects.",
                  path.c_str());
        return -1;
      }
    }

    std::vector<moveit_msgs::CollisionObject> collision_objects;
    ROS_INFO_STREAM(app_directory << " is a directory containing:");
    for (auto &entry : boost::make_iterator_range(fs::directory_iterator(app_directory), {})) {
      ROS_INFO_STREAM(entry);

      std::ifstream file_stream(entry.path().string(), std::ifstream::binary);
      if (file_stream) {
        Json::Value root;
        file_stream >> root;

        moveit_msgs::CollisionObject collision_object = extractObstacleFromJson(root, entry.path().stem().string());
        collision_objects.push_back(collision_object);
      } else {
        ROS_WARN_STREAM("could not open file " << entry.path());
      }
    }

    // Publish the collision objects to the scene
    for (const auto &collision_object : collision_objects) {
      collision_object.header.frame_id = move_group_arm.getPlanningFrame();
      planning_scene.world.collision_objects.push_back(collision_object);
    }

    ROS_INFO_STREAM("# collision objects " << planning_scene.world.collision_objects.size());
    planning_scene.is_diff = true;
    planning_scene_diff_publisher.publish(planning_scene);

    ROS_INFO("robot_control_node is ready");

    // visual_tools.trigger();
    // visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to add collision object");

    /////////////// planning scene interface /////////////

    // addCollisionObjects(planning_scene_interface);

    visual_tools.trigger();
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to pregrasp");

    ////////////// move group interface /////////////////

    // ros::WallDuration(3.0).sleep();

    // pregrasp
    // the position for panda_link8 = object pose - (length of cube/2 - distance b/w panda_link8 and palm of eef (0.058)
    // - some extra padding) - (desired offset for pregasp)
    // add the pointer for the convinience for calculation

    double* OBJ_POS;
    OBJ_POS = (double *) malloc (3 * sizeof(double));
    //Modify at here
    OBJ_POS[0] = 0.2;
    OBJ_POS[1] = -0.4;
    OBJ_POS[2] = 0.3;

    kan_place(move_group_arm, OBJ_POS);
    ROS_INFO("Dog Dog Dog");
    //left the loop  when the loop is ended
    ros::waitForShutdown();

    return 0;
  }
}