#!/usr/bin/env python

import rospy
import tf

if __name__ == '__main__':
    rospy.init_node('test_link')

    listener = tf.TransformListener()
    broadcaster = tf.TransformBroadcaster()

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            (pos, ori) = listener.lookupTransform('/realsense_yaw', '/realsense_pitch', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
	
	broadcaster.sendTransform((pos[0],pos[1],pos[2]),(ori[0],ori[1],ori[2],ori[3]),rospy.Time.now(),"camera_link","realsense_pitch")
        rate.sleep()

