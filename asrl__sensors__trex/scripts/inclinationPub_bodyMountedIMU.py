#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Imu
from asrl__sensors__trex.msg import *
from asrl__sensors__trex.srv import *
import numpy as np


#TODO: Use URDF transformations to compute orientation between imu and arm.
#import tf
#listener = tf.TransformListener()
#try:
#	(trans,rot) = listener.lookupTransform('/imu0', '/husky_link', rospy.Time(0))
#except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
#	continue




##################### parameters ###############################

inclinationOffsetRad = np.radians(13)  # [rad] camera is pointing downwards on the rover


##################### main function ############################

def callback(data):
	global inclinationPub
	# Get Accelerations in IMU frame
	x_imu = data.linear_acceleration.x
	y_imu = data.linear_acceleration.y
	z_imu = data.linear_acceleration.z
	a_imu = np.array([x_imu, y_imu, z_imu])

	#np.set_printoptions(precision=2)
	#print('x_imu: \t\t[%f]\n' % x_imu)
	#print('y_imu: \t\t[%f]\n' % y_imu)
	#print('z_imu: \t\t[%f]\n' % z_imu)

	# Transform Acceleration from IMU to Body frame
	rotMatrixImu2Body = np.array([[1,0,0],[0, np.cos(inclinationOffsetRad), -np.sin(inclinationOffsetRad)],[0,np.sin(inclinationOffsetRad), np.cos(inclinationOffsetRad)]])
	a_body = np.dot(rotMatrixImu2Body,a_imu)	
	
	x_body = -a_body[2]
	y_body = a_body[0]
	z_body = -a_body[1]

	#print('x_body: \t[%f]\n' % x_body)
	#print('y_body: \t[%f]\n' % y_body)
	#print('z_body: \t[%f]\n' % z_body)

	# Obtain Inclination
	vehicleAngleRad = np.sign(x_body) * (np.arccos(np.divide(-z_body, np.sqrt(np.sum([np.square(x_body), np.square(y_body), np.square(z_body)])))))
#	vehicleAngleRad = (-1) * (np.arccos(np.divide(z_body, np.sqrt(np.sum([np.square(x_body), np.square(y_body), np.square(z_body)]))))-np.pi)
	vehicleAngleDeg = np.around(np.degrees(vehicleAngleRad), decimals=1)

	# Generate Msg / Publish
	inclinationMessage = inclination()
	inclinationMessage.inclinationDeg = vehicleAngleDeg
	inclinationMessage.inclinationRad = vehicleAngleRad
	inclinationMessage.header.stamp = rospy.Time.now()	
	inclinationPub.publish(inclinationMessage)
        #print('Inclination of the body: \t\t [%f]\n' %bodyAngleDeg)



##################### side functions ############################

def getArmAngle(armAngleReading):
	global armAngleDeg, armAngleRad
	armAngleDeg = armAngleReading.Deg
	armAngleRad = np.radians(armAngleDeg)


##################### subscribers and publishers ##############

def publishInclination():
	global inclinationPub
	rospy.init_node('inclination', anonymous=True)
	inclinationPub = rospy.Publisher('/inclination', inclination, queue_size=10)
	rospy.Subscriber("/imu0", Imu, callback)
	rospy.Subscriber("/tether_angle", angleEncoder, getArmAngle)
	rospy.spin()
 
###################### main loop ###############################

if __name__ == '__main__':
	try:
		publishInclination()
	except rospy.ROSInterruptException: pass
