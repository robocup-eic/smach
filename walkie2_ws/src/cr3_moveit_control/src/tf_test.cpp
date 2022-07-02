#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "tf_test");
  ros::NodeHandle nh;

  tf2_ros::Buffer tf_buffer;
tf2_ros::TransformListener tf2_listener(tf_buffer);
geometry_msgs::TransformStamped base_link_cr3_base_link;

base_link_cr3_base_link = tf_buffer.lookupTransform("base_link", "cr3_base_link", ros::Time(0), ros::Duration(1.0) );

ROS_INFO_STREAM(base_link_cr3_base_link);

// tf2::doTransform(robot_pose, robot_pose, base_link_cr3_base_link); // robot_pose is the PoseStamped I want to transform

    ros::spin();
  ros::waitForShutdown();
  return 0;
}