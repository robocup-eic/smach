#shebangs

#!/usr/bin/env python

# this is a rospkg for converting Joy messages to Twist messages

import rospy
from std_msgs.msg import String
#import datatype Joy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
import math as m
"""
header: 
  seq: 9010
  stamp: 
    secs: 1687529370
    nsecs: 257777923
  frame_id: "/dev/input/js0"
axes: [-0.0, -0.0, -0.0, -0.0, 1.0, 1.0, -0.0, -0.0]
buttons: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
"""

# define a function to receive the Joy message
def joy_trans(data):
    # data.axes[0] # L stick x  1-->-1
    # data.axes[1] # L stick y  1-->-1
    # data.axes[2] # R stick x  1-->-1
    # data.axes[3] # R stick y  1-->-1
    # data.axes[4] # LT : 1 = rest
    # data.axes[5] # RT : 1 = rest
    # data.buttons[0] # A



    # trigger speed control the rotation speed
    independant_vel = False


    # Joystick L position (axes[0] and axes[1])
    x = data.axes[0] 
    y = data.axes[1]

    # LT Trigger position (axes[2])
    lt_trigger = (data.axes[5]-1)/2
    rt_trigger = -(data.axes[4]-1)/2

    # Use Pythagoras' theorem to determine the direction of the joystick
    # magnitude = (x**2 + y**2)**0.5

    # Normalize x and y values to get direction
    # direction_x = x / magnitude if magnitude != 0 else 0
    # direction_y = abs(y / magnitude if magnitude != 0 else 0)

    # Summation of all the triggers
    sum_trigger = (rt_trigger+lt_trigger )  #rt_trigger 

    
    if data.buttons[0]:
      # Calculate linear velocity
      if independant_vel:
        # Calculate angular velocity around the z-axis
        angular_z = x * 0.5 * sum_trigger
        linear_x = (sum_trigger * 0.5 )- abs(angular_z)  # map the trigger to linear velocity
      else:
        # Calculate angular velocity around the z-axis
        angular_z = x * 0.5
        linear_x = (sum_trigger * 0.5 ) # map the trigger to linear velocity
      # Initialize Twist message
      twist = Twist()
      # Publish to cmd_vel
      twist.linear.x = linear_x
      twist.angular.z = angular_z


      cmd_vel_pub.publish(twist)
    else:
       # Initialize Twist message
      twist = Twist()
      # Publish to cmd_vel
      twist.linear.x = 0
      twist.angular.z = 0
      



if __name__ == '__main__':
    try:
        rospy.init_node("gta_mf", anonymous=True)
        # create a publisher for cmd_vel
        cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        sub = rospy.Subscriber("joy", Joy, joy_trans) # subscribe to the topic "joy"
        # Run the node
        print("running node")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


# chmod +x gta_jot_trans.py


