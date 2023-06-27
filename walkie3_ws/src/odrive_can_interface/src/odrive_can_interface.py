#! /usr/bin/python3
import can
import cantools
import rospy
import threading
import signal
import sys
import math

from std_srvs.srv import Trigger, TriggerResponse
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rospkg


#bus = can.ThreadSafeBus(interface= "seeedstudio", channel="/dev/ttyUSB0", bitrate= 500000)
file = rospkg.RosPack().get_path('odrive_can_interface')+'/src/odrive-cansimple.dbc'
db = cantools.database.load_file(file)
axisID_left = 0x1
axisID_right = 0x0
Axis_State_IDLE = db.encode_message('Axis0_Set_Axis_State', {'Axis_Requested_State': 1})
Axis_State_CLOSED_LOOP_CONTROL = db.encode_message('Axis0_Set_Axis_State', {'Axis_Requested_State': 8})
flag = False


def diff_drive_controller(V,w): # V: linear velocity, w: angular velocity
    b = 0.33 # wheel base in m
    r = 0.085 # wheel radius in m
    wheel_dist = 2*math.pi*r

    angular_to_linear = w*(b/2)
    left_linear_val  = (V - angular_to_linear) * (60/wheel_dist) #RPM???
    right_linear_val = (V + angular_to_linear) * (60/wheel_dist) #RPM???

    return left_linear_val/60, -right_linear_val/60

def instantaneous_vel(w_left, w_right): # a linear velocity of an axis in turn/s
    b = 0.33 # wheel base in m
    r = 0.085# wheel radius in m
    v_left = 2*math.pi*r * w_left 
    v_right = 2*math.pi*r * w_right

    V = (v_left + v_right)/2

    w = (v_right - v_left)/b
    
    return V, w

def odrive_command(data):
    V = data.linear.x
    w = data.angular.z
    vel_left, vel_right = diff_drive_controller(V,w)
    data_left = db.encode_message('Axis0_Set_Input_Vel', {'Input_Vel': vel_left, 'Input_Torque_FF': 0})
    data_right = db.encode_message('Axis1_Set_Input_Vel', {'Input_Vel': vel_right, 'Input_Torque_FF': 0})
    msg_left = can.Message(arbitration_id=0x0d | axisID_left << 5, is_extended_id=False, data=data_left)
    msg_right = can.Message(arbitration_id=0x0d | axisID_right << 5, is_extended_id=False, data=data_right)
    bus.send(msg_left)
    bus.send(msg_right)

def trigger_response(request):
    global flag
    flag = not flag

    if flag:
        msg_left = can.Message(arbitration_id=0x07 | axisID_left << 5, is_extended_id=False, data= Axis_State_CLOSED_LOOP_CONTROL)
        msg_right = can.Message(arbitration_id=0x07 | axisID_right << 5, is_extended_id=False, data= Axis_State_CLOSED_LOOP_CONTROL)
        bus.send(msg_left)
        bus.send(msg_right)
    else :
        msg_left = can.Message(arbitration_id=0x07 | axisID_left << 5, is_extended_id=False, data= Axis_State_IDLE)
        msg_right = can.Message(arbitration_id=0x07 | axisID_right << 5, is_extended_id=False, data= Axis_State_IDLE)
        bus.send(msg_left)
        bus.send(msg_right)
    return TriggerResponse(success = True, message="Motor_State_Changed")

def signal_handler(sig, frame):
    msg_left = can.Message(arbitration_id= 0x07 | axisID_left <<5, is_extended_id=False, data = Axis_State_IDLE)
    msg_right = can.Message(arbitration_id= 0x07 | axisID_right << 5, is_extended_id=False, data = Axis_State_IDLE)
    bus.send(msg_left)
    bus.send(msg_right)
    rospy.signal_shutdown('You pressed Ctrl+C!')
    sys.exit(0)
    
def read_can_data(bus):
    Odom_msg = Odometry()
    Odom_publisher = rospy.Publisher('/motor_odom', Odometry, queue_size= 1)
    w_left = float()
    w_right = float()
    while not rospy.is_shutdown():
        msg = bus.recv() # receive a message from CAN bus
        if msg.arbitration_id == 0x09 | axisID_left << 5:
            msg = db.decode_message('Axis0_Get_Encoder_Estimates', msg.data)
            w_left = msg['Vel_Estimate']
        elif msg.arbitration_id == 0x09 | axisID_right << 5:
            msg = db.decode_message('Axis1_Get_Encoder_Estimates', msg.data)
            w_right = -msg['Vel_Estimate']
        V, w = instantaneous_vel(w_left, w_right)
        Odom_msg.header.stamp = rospy.Time.now()
        Odom_msg.header.frame_id = "odom"
        Odom_msg.child_frame_id = "base_link"

        Odom_msg.twist.twist.linear.x = V
        Odom_msg.twist.twist.angular.z = w

        Odom_publisher.publish(Odom_msg)

def odrive_can_interface():
    rospy.Subscriber("cmd_vel", Twist, odrive_command)
    s = rospy.Service('/MotorOn', Trigger, trigger_response)
    signal.signal(signal.SIGINT, signal_handler)
    rospy.spin()

if __name__ == '__main__':
    try:
        with can.ThreadSafeBus(interface= "socketcan", channel="can0", bitrate= 500000) as bus:
            rospy.init_node('odrive_can_interface', anonymous=True)
            
            msg_left = can.Message(arbitration_id=0x07 | axisID_left << 5, is_extended_id=False, data= Axis_State_CLOSED_LOOP_CONTROL)
            msg_right = can.Message(arbitration_id=0x07 | axisID_right << 5, is_extended_id=False, data= Axis_State_CLOSED_LOOP_CONTROL)
            bus.send(msg_left)
            bus.send(msg_right)
            thread = threading.Thread(target=read_can_data, args=(bus,))
            thread.start()
            odrive_can_interface()
    except rospy.ROSInterruptException:
        pass