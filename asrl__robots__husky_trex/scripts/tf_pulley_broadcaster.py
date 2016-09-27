#!/usr/bin/env python  
import roslib
import rospy
import tf
from asrl__sensors__trex.msg import *
from asrl__control__tether_management.msg import *
from asrl__drivers__labjack.msg import *
from asrl__drivers__labjack.srv import *
import numpy as np

def handle_pulley_pose(msg):
	br = tf.TransformBroadcaster()
	br.sendTransform((-0.0787, 0, 0.05849),tf.transformations.quaternion_from_euler(1.57, msg.Angle, 0),rospy.Time.now(),"pulley_link","arm_upper_link")

def pulleyBroadcast():
	rospy.init_node('tf_pulley_broadcaster')
	rospy.Subscriber("/tether_length", lengthEncoder, handle_pulley_pose)
	rospy.spin()
  
if __name__ == '__main__':
	try:		
		pulleyBroadcast()
	except rospy.ROSInterruptException: pass
