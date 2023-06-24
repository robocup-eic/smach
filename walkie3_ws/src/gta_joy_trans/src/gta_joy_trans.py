#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger, TriggerRequest
import threading

zeros_publishing = True

def publish_zeros():
    cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    r = rospy.Rate(10)  # 10hz
    while not rospy.is_shutdown():
        if zeros_publishing:
            twist = Twist()
            twist.linear.x = 0
            twist.angular.z = 0
            cmd_vel_pub.publish(twist)
        r.sleep()

def joy_trans(data):
    global zeros_publishing

    x = data.axes[0]
    y = data.axes[1]

    lt_trigger = (data.axes[5]-1)/2
    rt_trigger = -(data.axes[4]-1)/2

    sum_trigger = (rt_trigger+lt_trigger)

    if data.buttons[0]:
        zeros_publishing = False

        angular_z = x * 0.5
        linear_x = (sum_trigger * 0.5)

        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z

        cmd_vel_pub.publish(twist)
    else:
        zeros_publishing = True

    # Check button[2] for motor control service
    if data.buttons[11]:
        try:
            motor_service = rospy.ServiceProxy('/MotorOn', Trigger)
            resp = motor_service(TriggerRequest())
            print("Service call succeeded with response: " + str(resp.success))
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)

if __name__ == '__main__':
    try:
        rospy.init_node("gta_mf", anonymous=True)

        cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        zeros_thread = threading.Thread(target=publish_zeros)
        zeros_thread.start()

        sub = rospy.Subscriber("joy", Joy, joy_trans)
        print("running node")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
