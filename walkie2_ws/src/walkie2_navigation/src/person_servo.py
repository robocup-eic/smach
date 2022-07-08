#!/usr/bin/python
"""
Gets the position of the blob and it commands to steer the wheels

Subscribes to 
    /human/rel_coor
    
Publishes commands to 
    /person_follow_cmd

"""
import time
import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import String, Int16

K_LAT_DIST_TO_STEER     = 0.025
START_OFFSET = 30


def saturate(value, min, max):
    if value <= min: return(min)
    elif value >= max: return(max)
    else: return(value)

class ChasePerson():
    def __init__(self):
        
        self.rel_x         = 0.0
        self.rel_y         = 0.0
        self.abs_x         = 0.0
        self.abs_y         = 0.0
        self.abs_z         = 0.0
        self._time_detected = 0.0

        self.cmd = ''
        
        self.sub_center = rospy.Subscriber("/human/rel_coor", Point, self.rel_update_coor)
        rospy.loginfo("Rel Subscribers set")
        
        self.sub_cmd = rospy.Subscriber("/realsense_follow_cmd",String, self.set_cmd)
        rospy.loginfo("Command Subscribers set")
        
        self.pub_realsense_relative_yaw = rospy.Publisher("/realsense_yaw_relative_command", Int16, queue_size=1)
        rospy.loginfo("Publisher set")

        self.pub_realsense_absolute_pitch = rospy.Publisher("/realsense_pitch_absolute_command", Int16, queue_size=1)
        self.pub_realsense_absolute_yaw = rospy.Publisher("/realsense_yaw_absolute_command", Int16, queue_size=1)
        rospy.loginfo("Pubslisher realsense set home")
        
        self._message = Int16()
        
    def realsense_set_home(self):
        self.pub_realsense_absolute_pitch.publish(-35)
        self.pub_realsense_absolute_yaw.publish(10)
        time.sleep(1)

    @property
    def is_detected(self): return(time.time() - self._time_detected < 1.0)

    def set_cmd(self,message):
        self.cmd = message.data
    
        
    def rel_update_coor(self, message):
        self.rel_x = message.x
        self.rel_y = message.y
        self._time_detected = time.time()
        rospy.loginfo("Human detected: %.1f  %.1f "%(self.rel_x, self.rel_y))

    def get_control_action(self):
        """
        Based on the current ranges, calculate the command
        
        Steer will be added to the commanded throttle
        throttle will be multiplied by the commanded throttle
        """
        steer_action    = 0
        
        if self.is_detected:
            #--- Apply steering, proportional to how close is the object
            steer_action   = -1*K_LAT_DIST_TO_STEER*self.rel_x
            steer_action   = saturate(steer_action, -90, 90)
        else:
            steer_action   = 0

        return int(steer_action)
        
    def run(self):
        
        #--- Set the control rate
        rate = rospy.Rate(5)

        try:
            while not rospy.is_shutdown():
                #-- Get the control action 
                steer_action = self.get_control_action() 
                
                # rospy.loginfo("angular_z = {}".format(steer_action))

                if self.cmd != 'follow':
                    self.realsense_set_home()
                    steer_action = 0      

                rospy.loginfo("angular_z = {}".format(steer_action))
                
                #-- update the message
                self._message.data = steer_action
                
                #-- publish it
                self.pub_realsense_relative_yaw.publish(self._message)

                rate.sleep()
        except KeyboardInterrupt:
            return
            
if __name__ == "__main__":

    rospy.init_node('chase_person')

    chase_person = ChasePerson()
    chase_person.run()            