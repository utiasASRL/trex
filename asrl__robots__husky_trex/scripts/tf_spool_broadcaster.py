#!/usr/bin/env python  
import roslib
import rospy
import tf
from asrl__sensors__trex.msg import *
from asrl__control__tether_management.msg import *
from asrl__drivers__labjack.msg import *
from asrl__drivers__labjack.srv import *
import numpy as np

encoderOffset = -0.349 # offset of magnetic encoder measurement in radians w/r/t arm (PREV =0.5784)

def handle_spool_pose(msg):
	br = tf.TransformBroadcaster()
	# negative msg.Angle is used to correct rotation direction (encoder mounted upsidedown)
	br.sendTransform((0, 0, 0.0977),tf.transformations.quaternion_from_euler(0, 0, -(msg.Angle-np.pi-encoderOffset)),rospy.Time.now(),"spool_link","bearing_link")

def spoolBroadcast():
	rospy.init_node('tf_spool_broadcaster')
	rospy.Subscriber("/motor_encoder", motorEncoder, handle_spool_pose)
	rospy.spin()

if __name__ == '__main__':
	try:		
		spoolBroadcast()
	except rospy.ROSInterruptException: pass
