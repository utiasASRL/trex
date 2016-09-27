#!/usr/bin/env python  
import roslib
import rospy
import tf
from asrl__sensors__trex.msg import *
from asrl__control__tether_management.msg import *
from asrl__drivers__labjack.msg import *
from asrl__drivers__labjack.srv import *
import numpy as np

def handle_pitch_pose(msg):
	br = tf.TransformBroadcaster()
	br.sendTransform((-0.19, 0, 0),tf.transformations.quaternion_from_euler(1.57, 3.93 + np.radians(msg.Deg), 0),rospy.Time.now(),"pitch_link","arm_lower_link")

def pitchBroadcast():
	rospy.init_node('tf_pitch_broadcaster')
	rospy.Subscriber("/tether_pitch", pitchPot, handle_pitch_pose)
	rospy.spin()
  
if __name__ == '__main__':
	try:		
		pitchBroadcast()
	except rospy.ROSInterruptException: pass
