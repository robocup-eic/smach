//http://clopema.felk.cvut.cz/redmine/projects/clopema/wiki/Sending_trajectory_to_the_controller
//http://wiki.ros.org/actionlib_tutorials/Tutorials/SimpleActionServer%28ExecuteCallbackMethod%29
//http://wiki.ros.org/actionlib_tutorials/Tutorials/SimpleActionServer%28GoalCallbackMethod%29
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <vector>
#include "cr3_lowlevel_pkg/JointCommand.h"

class RobotTrajectoryFollower
{
protected:

  ros::NodeHandle nh_;
  // NodeHandle instance must be created before this line. Otherwise strange error may occur.
  actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> as_;
  trajectory_msgs::JointTrajectory goal_;
  control_msgs::FollowJointTrajectoryResult result_;
  control_msgs::FollowJointTrajectoryFeedback feedback_;
  std::string action_name_;

  // publisher
  ros::Publisher pub = nh_.advertise<cr3_lowlevel_pkg::JointCommand>("cr3_arm_command", 100);
  cr3_lowlevel_pkg::JointCommand msg;
  ros::Rate loop_rate;

public:

  RobotTrajectoryFollower(std::string name) :
    as_(nh_, name, false),
    action_name_(name),
    loop_rate(10)
  {
    as_.registerGoalCallback(boost::bind(&RobotTrajectoryFollower::goalCB, this));
    as_.registerPreemptCallback(boost::bind(&RobotTrajectoryFollower::preemptCB, this));

    as_.start();
    
  }

  ~RobotTrajectoryFollower(void)//Destructor
  {
  }

  void goalCB()
  {
    // accept the new goal
    goal_ = as_.acceptNewGoal()->trajectory;

    int count = 0;
    int trajectory_len = goal_.points.size();
    while (ros::ok() && count < trajectory_len)
    {

      std::vector<double> joint_positions = goal_.points[count].positions;
      // int joint_number = joint_positions.size();
      // for (int i = 0; i < joint_number; i++){
      //   ROS_INFO("goal is :%f", joint_positions[i]);
      // }

      // publish to cr3_controller node
      msg.joint_commands = joint_positions;
      pub.publish(msg);

      ros::spinOnce();

      loop_rate.sleep();
      ++count;
    }

    // send succeed to result action
    result_.SUCCESSFUL;
    as_.setSucceeded(result_);
    
  }

  void preemptCB()
  {
    ROS_INFO("%s: Preempted", action_name_.c_str());
    // set the action state to preempted
    as_.setPreempted();
  }

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "trajectory_follower_server");

  RobotTrajectoryFollower RobotTrajectoryFollower("/cr3_arm_controller/follow_joint_trajectory");

  ros::spin();

  return 0;
}