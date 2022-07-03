#!/usr/bin/env python
import rospy
from std_msgs.msg import Bool

def lift_cb(data) :
   while (not data.data) : pass

def lift_command() :
   lift_pub = rospy.Publisher('lift_command', Bool)
   lift_pub.publish(True)

   rospy.Subscriber("done", Bool, lift_cb)

if __name__ == "__main__":
   high = 0.85
   rospy.loginfo(high)
   high.z -= 0.49      #lift state is False
   rospy.loginfo(high)
   if (0.25 < high.z) : 
      lift_command()
      high.z -= 0.23  #lift state is True
      rospy.loginfo(high)