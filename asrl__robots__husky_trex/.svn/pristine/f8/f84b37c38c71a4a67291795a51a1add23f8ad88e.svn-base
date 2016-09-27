#!/usr/bin/env python  
import roslib
import rospy
import tf
from asrl__sensors__trex.msg import *
from asrl__control__tether_management.msg import *
from asrl__drivers__labjack.msg import *
from asrl__drivers__labjack.srv import *
import numpy as np

def handle_arm_pose(msg):
	br = tf.TransformBroadcaster()
  # settings for husky centric URDF (will publish tf to arm)
	br.sendTransform((0, 0, 0),tf.transformations.quaternion_from_euler(0, 0, np.radians(msg.Deg)),rospy.Time.now(),"bearing_link","body_link")
  # settings for tether arm centric URDF (will publish tf to husky, negative corrects the rotation)
	#br.sendTransform((0, 0, -0.1827),tf.transformations.quaternion_from_euler(0, 0, -np.radians(msg.Deg)),rospy.Time.now(),"husky_link","body_link")

def armBroadcast():
	rospy.init_node('tf_arm_broadcaster')
	rospy.Subscriber("/tether_angle", angleEncoder, handle_arm_pose)
	rospy.spin()
  
if __name__ == '__main__':
	try:		
		armBroadcast()
	except rospy.ROSInterruptException: pass
