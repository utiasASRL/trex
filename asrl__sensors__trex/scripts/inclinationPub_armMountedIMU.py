#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Imu
from asrl__sensors__trex.msg import *
from asrl__sensors__trex.srv import *
import numpy as np

##################### main function ############################

def callback(data):
	global inclinationPub
	# Get Accelerations
	x = data.linear_acceleration.x
	y = data.linear_acceleration.y
	z = data.linear_acceleration.z
	
	# Obtain Inclination
	vehicleAngleRad = np.subtract(np.arccos(np.divide(z, np.sqrt(np.sum([np.square(x), np.square(y), np.square(z)])))),(np.pi/2))
	vehicleAngleDeg = np.around(np.degrees(vehicleAngleRad), decimals=1)
	
	# Generate Msg / Publish
	inclinationMessage = inclination()
	inclinationMessage.inclinationDeg = vehicleAngleDeg
	inclinationMessage.inclinationRad = vehicleAngleRad
	inclinationMessage.header.stamp = rospy.Time.now()	
	inclinationPub.publish(inclinationMessage)
	#print('publishing inclination')
	#print(vehicleAngleDeg)	

##################### subscribers and publishers ##############

def publishInclination():
	global inclinationPub
	rospy.init_node('inclination', anonymous=True)
	inclinationPub = rospy.Publisher('/inclination', inclination, queue_size=10)
	rospy.Subscriber("/imu0", Imu, callback)
	rospy.spin()
 
###################### main loop ###############################

if __name__ == '__main__':
	try:
		publishInclination()
	except rospy.ROSInterruptException: pass
