#!/usr/bin/python
"""
Gets the position of the blob and it commands to steer the wheels

Subscribes to 
    /blob/point_blob
    
Publishes commands to 
    /dkcar/control/cmd_vel    

"""
import time
import rospy
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import String, Int32MultiArray


LINEAR_THRESHOLD = 1.3
K_LAT_DIST_TO_STEER     = 0.001
K_LAT_DIST_TO_THROTTLE     = 0.1

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

        self.sub_position = rospy.Subscriber("/human/abs_coor", Point, self.abs_update_coor)
        rospy.loginfo("Abs Subscribers set")
        
        self.sub_cmd = rospy.Subscriber("/human/follow_cmd",String, self.set_cmd)
        rospy.loginfo("Command Subscribers set")

        self.sub_cmd = rospy.Subscriber("/human/distances",Int32MultiArray, self.set_min_distance)
        rospy.loginfo("Distances Subscribers set")
        
        self.pub_twist = rospy.Publisher("/walkie2/cmd_vel", Twist, queue_size=5)
        rospy.loginfo("Publisher set")
        
        self._message = Twist()

        # For stopping robot if a person suddenly appears
        self.min_distance = 999
        
    @property
    def is_detected(self): return(time.time() - self._time_detected < 1.0)

    def set_cmd(self,message):
        self.cmd = message.data
    
    def set_min_distance(self, message):
        self.min_distance = min(message.data)

    def rel_update_coor(self, message):
        self.rel_x = message.x
        self.rel_y = message.y
        self._time_detected = time.time()
        # rospy.loginfo("Human detected: %.1f  %.1f "%(self.rel_x, self.rel_y))

    def abs_update_coor(self, message):
        self.abs_x = message.x
        self.abs_y = message.y
        self.abs_z = message.z

    

    def get_control_action(self):
        """
        Based on the current ranges, calculate the command
        
        Steer will be added to the commanded throttle
        throttle will be multiplied by the commanded throttle
        """
        steer_action    = 0.0
        throttle_action = 0.0
        
        if self.is_detected:
            #--- Apply steering, proportional to how close is the object
            steer_action   = -1*K_LAT_DIST_TO_STEER*self.rel_x
            steer_action   = saturate(steer_action, -0.5, 0.5)

            if self.abs_z > LINEAR_THRESHOLD:
                throttle_action = self.abs_z*K_LAT_DIST_TO_THROTTLE
                throttle_action = saturate(throttle_action, 0, 0.4)
            else:
                throttle_action = 0.0
        else:
            steer_action    = 0.0
            throttle_action = 0.0

        return (steer_action, throttle_action)
        
    def run(self):
        
        #--- Set the control rate
        rate = rospy.Rate(5)

        try:
            while not rospy.is_shutdown():
                #-- Get the control action 
                steer_action, throttle_action   = self.get_control_action() 

                if self.cmd != 'follow':
                    steer_action = 0
                    throttle_action = 0

                elif self.min_distance < LINEAR_THRESHOLD:
                    steer_action = 0
                    throttle_action = 0

                else:

                    rospy.loginfo("linear x = {}, angular_z = {}".format(throttle_action, steer_action))
                    
                    #-- update the message
                    self._message.linear.x  = throttle_action
                    self._message.angular.z = steer_action
                    
                    #-- publish it
                    self.pub_twist.publish(self._message)

                rate.sleep()
        except KeyboardInterrupt:
            return
            
if __name__ == "__main__":

    rospy.init_node('chase_person')

    chase_person = ChasePerson()
    chase_person.run()            