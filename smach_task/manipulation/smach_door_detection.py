#!/usr/bin/env python

import rospy
import smach
import smach_ros
import pyrealsense2 as rs2

class Close_or_Open(smach.State):
    def __init__(self):
        rospy.loginfo('Initiating state Close_or_Open')
        smach.State.__init__(self, outcomes= ['continue_A'])
        
        # initiate variables
        self.rs = rs2.pipeline()
        self.round = 10
        self.close_distance = 1

    def execute(self,userdata):
        def check_frame():
            # get x,y pixel in center
            self.rs.start()
            init_frame = self.rs.wait_for_frames().get_depth_frame().as_depth_frame()
            width = init_frame.get_width()
            height = init_frame.get_height()
            print(width/2, height/2)
            print(width, height)
            self.rs.stop()

            return (width//2, height//2)
        
        def distance_check(x, y):
            # get depth from x,y pixel
            # round_check: knowing that the door is really open
            round_check = 0
            self.rs.start()
            while True:
                rospy.sleep(0.5)
                dpt_frame = self.rs.wait_for_frames().get_depth_frame().as_depth_frame()
                distance = dpt_frame.get_distance(x, y)
                print(distance)

                if distance < 0.4:
                    continue

                if round_check == self.round:
                    break
                
                if distance > self.close_distance:
                    round_check += 1
                else:
                    round_check = 0
                
            self.rs.stop()
            
            return True
        
        #--------------------------------------------start----------------------------------------
        rospy.loginfo('start init frame')
        x,y = check_frame()

        rospy.loginfo('start distance checking')
        distance_check(x, y)

        return 'continue_A'

def main():
    rospy.init_node('smach_Door_checking')
    sm = smach.StateMachine(outcomes=['SUCCEEDED'])

    with sm:
        smach.StateMachine.add('Close_or_Open', Close_or_Open(),
                                transitions= {'continue_A' : 'SUCCEEDED'})

    outcome = sm.execute()

if __name__ == "__main__":
    main()