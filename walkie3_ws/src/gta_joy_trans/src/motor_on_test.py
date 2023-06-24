#!/usr/bin/env python

import rospy
from std_srvs.srv import Trigger, TriggerResponse

motor_on = False  # variable to hold the current state of the motor

def motor_on_off(req):
    global motor_on
    motor_on = not motor_on  # flip the motor state
    print("Motor on") if motor_on else print("Motor off")
    return TriggerResponse(success=motor_on, message="Motor state changed")

if __name__ == "__main__":
    rospy.init_node('motor_service_server')
    s = rospy.Service('MotorOn', Trigger, motor_on_off)
    rospy.spin()
