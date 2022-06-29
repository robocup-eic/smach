#include <boost/filesystem.hpp>
#include <jsoncpp/json/json.h>
#include <ros/ros.h>
#include <vector>

#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>

static const std::string PLANNING_GROUP_ARM = "arm";
std::string APP_DIRECTORY_DEFAULT = "ros/robocup-manipulation/cr3_ws/src/cr3_moveit_control/config/constraints"; //default

moveit_msgs::CollisionObject extractObstacleFromJson(Json::Value &root, std::string name)
{
    moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id = "world";
    collision_object.id = name;

    const Json::Value dimensions = root["dimensions"];
    ROS_INFO_STREAM("Extracted dimensions: " << dimensions);
    // Define a box to add to the world.
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = dimensions["x"].asDouble();
    primitive.dimensions[1] = dimensions["y"].asDouble();
    primitive.dimensions[2] = dimensions["z"].asDouble();

    const Json::Value position = root["position"];
    ROS_INFO_STREAM("Extracted position: " << position);

    const Json::Value orientation = root["orientation"];
    ROS_INFO_STREAM("Extracted orientation: " << orientation);
    // Define a pose for the box (specified relative to frame_id)
    geometry_msgs::Pose box_pose;
    box_pose.orientation.w = orientation["w"].asDouble();
    box_pose.orientation.x = orientation["x"].asDouble();
    box_pose.orientation.y = orientation["y"].asDouble();
    box_pose.orientation.z = orientation["z"].asDouble();

    // MoveIt! planning scene expects the center of the object as position.
    // We add half of its dimension to its position
    box_pose.position.x = position["x"].asDouble() + primitive.dimensions[0] / 2.0;
    box_pose.position.y = position["y"].asDouble() + primitive.dimensions[1] / 2.0;
    box_pose.position.z = position["z"].asDouble() + primitive.dimensions[2] / 2.0;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;

    return std::move(collision_object);
}

int main(int argc, char **argv)
{
    namespace fs = boost::filesystem;
    ROS_INFO("RUNNING constraint publisher");

    ros::init(argc, argv, "constraint_publisher");

    ros::NodeHandle nh("~");

    ros::AsyncSpinner spinner(1);
    spinner.start();

    moveit::planning_interface::MoveGroupInterface move_group_arm(PLANNING_GROUP_ARM);

    ros::Publisher planning_scene_diff_publisher = nh.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);

    // rviz visual and moveit visual
    moveit_visual_tools::MoveItVisualTools visual_tools("base_link");
    visual_tools.deleteAllMarkers();

    ros::WallDuration sleep_t(0.5);
    while (planning_scene_diff_publisher.getNumSubscribers() < 1)
    {
        sleep_t.sleep();
    }
    moveit_msgs::PlanningScene planning_scene;

    // read JSON files from ~/.cr3_simulation
    // fs::path home(getenv("HOME"));
    // fs::path app_directory(home);

    std::string app_directory;
    // nh.param<std::string>("constraints_dir", constraints_dir, APP_DIRECTORY_DEFAULT);
    nh.getParam("constraints_dir", app_directory);
    // ROS_INFO_STREAM(app_directory);
    // ROS_INFO_STREAM(constraints_dir);
    // app_directory = constraints_dir;


    if (!fs::exists(app_directory) && !fs::is_directory(app_directory))
    {
        ROS_WARN_STREAM(app_directory << " does not exist");
        return -1;
    }

    std::vector<moveit_msgs::CollisionObject> collision_objects;
    ROS_INFO_STREAM(app_directory << " is a directory containing:");
    for (auto &entry : boost::make_iterator_range(fs::directory_iterator(app_directory), {}))
    {
        ROS_INFO_STREAM(entry);

        std::ifstream file_stream(entry.path().string(), std::ifstream::binary);
        if (file_stream)
        {
            Json::Value root;
            file_stream >> root;

            moveit_msgs::CollisionObject collision_object = extractObstacleFromJson(root, entry.path().stem().string());
            collision_objects.push_back(collision_object);
        }
        else
        {
            ROS_WARN_STREAM("could not open file " << entry.path());
        }
    }

    // Publish the collision objects to the scene
    for (const auto &collision_object : collision_objects)
    {
        collision_object.header.frame_id = move_group_arm.getPlanningFrame();
        planning_scene.world.collision_objects.push_back(collision_object);
    }

    ROS_INFO_STREAM("# collision objects " << planning_scene.world.collision_objects.size());
    planning_scene.is_diff = true;
    planning_scene_diff_publisher.publish(planning_scene);

    visual_tools.trigger();

    return 0;
}