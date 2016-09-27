#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from labjack import ljm
from asrl__sensors__trex.msg import *
from asrl__control__tether_management.msg import *
from asrl__drivers__labjack.msg import *
from asrl__drivers__labjack.srv import *
import numpy

cwName = "EIO4"
ccwName = "EIO6"
buttonNames = [cwName, ccwName]

def buttonControlToLabJack():
	global settingsPub
	request = writeNames() 
	request.names =
	request.numFrames = 2
	request.values = [0]
	buttonPub.publish(request)

if __name__ == '__main__':
			
	#ROS PARAMS
	buttonPub = rospy.Publisher('/labjack_interface_node/write_names',writeNames,queue_size=10);
	rospy.init_node('button_control', anonymous=True)
	r = rospy.Rate(10) # 10hz

	while not rospy.is_shutdown():
		try:
			buttonControlToLabJack()
			r.sleep()
		except rospy.ROSInterruptException: pass
