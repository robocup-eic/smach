#!/usr/bin/env python

import roslib
import rospy
import tf2_ros
from nav_msgs.msg import Odometry
from math import pi
import tf
import tf2_msgs

from geometry_msgs.msg import PoseStamped, Twist ,Vector3, TransformStamped
from std_msgs.msg import Bool,Int64

# Brings in the SimpleActionClient
import actionlib

# Brings in the .action file and messages used by the move base action
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


class Follow_person():  
    def __init__(self):
        rospy.loginfo('Initiating state Follow_person')
        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.target_lost = False
        self.is_stop = False
        self.cmd_pub = rospy.Publisher("/walkie/cmd_vel",Twist,queue_size=1)

        self.target_sub = rospy.Subscriber("target_lost", Bool , self.target_lost_callback)
        self.stop_sub = rospy.Subscriber("stop", Bool , self.is_stop_callback)

        rospy.sleep(1)
        rospy.loginfo("Waiting for Action Server")
        self.client.wait_for_server()
        rospy.loginfo("Action server is up, we can send new goal!")

    def target_lost_callback(self,msg):
        self.target_lost = msg.data
        # if target is not lost, executing.data is True
    
    def is_stop_callback(self,msg):
        self.is_stop = msg.data

    def execute(self):
        rospy.loginfo('Executing state Follow_person')

        pose = TransformStamped()
        while not rospy.is_shutdown():
            
            try:
                pose = self.tfBuffer.lookup_transform('base_footprint','human_base_footprint',rospy.Time.now()-rospy.Duration.from_sec(0.5))

                goal = MoveBaseGoal()
                goal.target_pose.header.frame_id = "base_footprint"
                goal.target_pose.header.stamp = rospy.Time.now()-rospy.Duration.from_sec(0.5)
                goal.target_pose.pose.position.x = pose.transform.translation.x
                goal.target_pose.pose.position.y = pose.transform.translation.y
                goal.target_pose.pose.orientation = pose.transform.rotation

                rospy.loginfo("Sending new goal: Quarternion is {}, {}, {}, {}".format(pose.transform.rotation.w,pose.transform.rotation.x,pose.transform.rotation.y,pose.transform.rotation.z))

                self.client.send_goal(goal)

                if  self.is_stop == True:
                    # Cancel all remaining goals
                    rospy.loginfo("Cancel the current goal")
                    self.wait = self.client.cancel_goal()

                    rospy.loginfo("Stopping the robot")
                    cancel = Twist()
                    

                    cancel.linear.x = 0.0
                    cancel.linear.y = 0.0
                    cancel.angular.z = 0.0
                    
                    self.cmd_pub.publish(cancel)
                    rospy.loginfo("Robot is stopped")

                if self.target_lost:
                    # wait untill the robot reaches the lastest goal
                    rospy.loginfo("Target is lost")
                    wait = self.client.wait_for_result(rospy.Duration.from_sec(10.0))

                    rospy.loginfo("Perform the rotation to find the target")
                    rotate = Twist()
                    rotate.angular.z = pi/30
                    rospy.sleep(0.5)

                    
                else:
                    # pass
                    rospy.loginfo("Waiting for result")
                    wait = self.client.wait_for_result(rospy.Duration.from_sec(1.0))
              
            except Exception as e:
                pass
                
if __name__=='__main__':

    rospy.init_node('person_follwer')
    
    client = Follow_person()
   
    # rospy.sleep(2.5)
    try:
        client.execute()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
            
        
        
        
    rospy.spin()