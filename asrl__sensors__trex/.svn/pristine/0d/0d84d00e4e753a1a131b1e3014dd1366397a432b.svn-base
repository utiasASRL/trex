#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Imu
from asrl__sensors__trex.msg import *
from asrl__sensors__trex.srv import *
import numpy as np
import tf


#########################################################################################
#										        #
# This node publishes the angle between the robot arm and the gravity vector projected 	#
# in the xy-plane of the body of the TReX robot. The gravity vector is measured by the	#
# IMU in the VisionSensor of TReX.							#
#											#
# 	GravityAngle		RobotBodyFrame		      ArmAngle 			#
#				      							#
#	      0			      |				 0			#
#	      			      |							#
#	90   	  270		      o------> y_body	   90         270		#
#				      |							#
#	     180		      |			        180			#
#				      v							#
#				      x_body						#
#											#
#			ArmGravityAngle := ArmAngle - GravityAngle  			#
#											#
# At flat ground the components of the gravity vector in the xy-plane of the body frame #
# of TReX are really small and quickly varying. The ArmGravityAngle has no meaning in 	#
# this case. We don't care about that because at small inclinations the ArmGravityAngle	#
# direction is not important for the tether controller.					#
#											#
#########################################################################################




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
	
	#print('\n\n\n')
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


#	#Alternative Way:
#    	listener = tf.TransformListener()
#	try:
#		(trans,rot) = listener.lookupTransform('/imu0', '/husky_link', rospy.Time(0))
#	except (tf.LookupException, tf.ConnectivityException):
#		trans = 0
#	print('translation: ',trans)
#	print('rotation: ',rot)


	# Find Orientation of Gravity Vector in xy-Plane of the Robot Body
	gravityAngleRad = np.arctan2(-y_body,x_body) # Calculates 4-quadrant atan2(y/x)	
	if gravityAngleRad < 0:
		gravityAngleRad = gravityAngleRad + 2*np.pi #Make gravityAngle in [0,2pi]
	gravityAngleDeg = np.degrees(gravityAngleRad)
	
	#print('gravityAngleDeg: \t[%f]\n' % gravityAngleDeg)
	#print('armAngleDeg: \t\t[%f]\n' % armAngleDeg)

	# Compute Angle between Arm and Inplane-Projection of the Gravity-Vector
	armGravityAngleRad = armAngleRad - gravityAngleRad
	if armGravityAngleRad < 0:
		armGravityAngleRad = armGravityAngleRad + 2*np.pi #Make armGravityAngle in [0,2pi]
	armGravityAngleDeg = np.degrees(armGravityAngleRad)
	
	#print('armGravityAngleDeg: \t[%f]\n' % armGravityAngleDeg)

	# Generate Msg / Publish
	armGravityAngleMessage = armGravityAngle()
	armGravityAngleMessage.armGravityAngleDeg = armGravityAngleDeg
	armGravityAngleMessage.armGravityAngleRad = armGravityAngleRad
	armGravityAngleMessage.header.stamp = rospy.Time.now()	
	armGravityAnglePub.publish(armGravityAngleMessage)

	

##################### side functions ############################

def getArmAngle(armAngleReading):
	global armAngleDeg, armAngleRad
	armAngleDeg = armAngleReading.Deg
	armAngleRad = np.radians(armAngleDeg)

def getInclination(inclinationReading):
	global currentInclinationRad 
	currentInclinationRad = inclinationReading.inclinationRad	


##################### subscribers and publishers ##############

def publishArmGravityAngle():
	global armGravityAnglePub
	rospy.init_node('armGravityAngle', anonymous=True)
	armGravityAnglePub = rospy.Publisher('/armGravityAngle', armGravityAngle, queue_size=10)
	rospy.Subscriber("/imu0", Imu, callback)
	rospy.Subscriber("/tether_angle", angleEncoder, getArmAngle)
	rospy.Subscriber("/inclination", inclination, getInclination)
	rospy.spin()
 
###################### main loop ###############################

if __name__ == '__main__':
	try:
		publishArmGravityAngle()
	except rospy.ROSInterruptException: pass
