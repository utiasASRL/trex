#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Empty
from nav_msgs.msg import Odometry
from asrl__sensors__trex.msg import *
from asrl__control__tether_management.msg import *
from asrl__drivers__labjack.msg import *
from asrl__drivers__labjack.srv import *
import numpy as np
import sys

# intialize variables
currentTension = 0
secondsLast = 0
lastTetherLength = str(sys.argv) #TODO Does this work?
trueTetherVelocityLast = 0 
voltsInterval = 0.2 # volts to increase or decrease by
lengthEncoderOffset = 0.275 # m offset of encoder from vehicle rotation axis
vehiclePositionLast = np.array([ [0],[0],[0] ])
anchorPositionEstimate = np.array([ [0],[0],[0] ])

# moved from listener function 
tensionFloor = 1.6 # volts, minimum tension for taut rope
tensionCeiling = 5.5 #TODO: MAKE SURE THIS IS SET CORRECTLY FOR DOME TEST  # volts, maximum tension coimputed from free hanging vehicle 
motorVoltsMax = 5 # volts, motor controller takes 0-5v
motorVoltsMin = 0.01 # min operable voltage
vehicleVelocityThresh = 0.04 # vehicle m/s required before action is taken
tetherVelocityThresh = 0.01 # tether m/s required before action is taken

def getTension(forceReading):
	global currentTension
	currentTension = forceReading.Vout

def getArmAngle(armAngleReading):
	global armAngleDeg, armAngleRad
	armAngleDeg = armAngleReading.Deg
	armAngleRad = np.radians(armAngleDeg)

def getTetherLength(tetherLengthReading):
	global currentTetherLength,currentTetherVelocity,anchorPositionEstimate, armAngleRad
	currentTetherLength = tetherLengthReading.Meters
	currentTetherVelocity = tetherLengthReading.Velocity
	
def getVelocity(velocity):
	global linearVelocity, angularVelocity
	linearVelocity = -(velocity.twist.twist.linear.x) #making negative to fix offset
	angularVelocity = velocity.twist.twist.angular.z

def getPose(odometer):
	global vehcilePosition, vehcileOrientation
	vehcilePosition = odometer.pose.pose.position # has variables x,y,z
	vehcileOrientation = odometer.pose.pose.orientation # has variables x,y,z,w

def listener(emptyMessage):
	global currentTension,vehiclePositionLast,lastTetherLength,armAngleRad,currentTetherLength,linearVelocity,angularVelocity,vehcilePosition, vehcileOrientation,secondsLast,pub,currentTetherVelocity

	controlMessage = motorCommand()

	####### projective geometry method of determining tether rate of change ########
	# get current time	
	time = rospy.Time.now()
	# convert time to seconds
	seconds = time.to_sec() 
	# compute sample rate instantaneously
	sampleRate = (seconds - secondsLast)
	secondsLast = seconds
	
	lastTetherLength = currentTetherLength + lastTetherLength
	
	anchorPositionEstimate = np.array([ [lastTetherLength*np.cos(armAngleRad)],[-lastTetherLength*np.sin(armAngleRad)],[armAngleRad] ]) #TODO where should this go, eventually we want to estimate the entire time, but now we want to call once and set

	vehiclePositionLast = np.array([ [vehcilePosition.x],[vehcilePosition.y],[vehcilePosition.z] ])

	# TODO How to handle this in 3D, assuming flat terrain we can just use the Yaw
	vehicleQuaternion = (
		vehcileOrientation.x,
		vehcileOrientation.y,
		vehcileOrientation.z,
		vehcileOrientation.w)
	vehicleEuler = tf.transformations.euler_from_quaternion(vehicleQuaternion)
	vehicleRoll = vehicleEuler[0]
	vehiclePitch = vehicleEuler[1]
	vehicleYaw = vehicleEuler[2]
	
	# Generate a vector for vehicle velocity based on the current linear and angular velocity (the derivative gives heading)
	vehicleVelocityVector = np.array( [[np.cos(vehicleYaw)*linearVelocity],[-np.sin(vehicleYaw)*linearVelocity],[angularVelocity]] )

	# Estimate future vehicle position using the last position and propagating forward with the derivative of velocity	
	vehiclePositionEstimate = np.add(vehiclePositionLast,(sampleRate*vehicleVelocityVector))

	# Generate a vecotr for the tether using the anchor point and last vehicle position	
	currentTetherVector = np.subtract(anchorPositionEstimate,vehiclePositionEstimate) #np.subtract(anchorPositionEstimate,vehiclePositionLast[0:2:1])

	# Compute the dot product of velocity and tether vector normalized by by the tether vector to find rate of change for tether # it is necessary to use the transpose operator ".T" to compute the dot (inner) product with itself, otherwise shape mismatch occurs:
	requiredTetherVelocity = np.asscalar(np.dot(vehicleVelocityVector.T,currentTetherVector)) / np.linalg.norm(currentTetherVector) #TODO watch division may result in zero denominator, check proximity to zero, normalize beforehand
	
	# Compute velocity difference to use in incremental control
	velocityError = requiredTetherVelocity - currentTetherVelocity # TODO Check sign of current tether velocity

	#print('Required Velocity:\t[%f]\nCurrent Velocity:\t[%f]\n\n' % (requiredTetherVelocity,currentTetherVelocity))

	velocityKp = velocityError * Kp
	integrator = (decayRate * integrator) + velocityError
	velocityKi = integrator * Ki
	velocityKd = (velocityError - velocityErrorLast) * Kd
	velocityErrorLast = velocityError
	velocityScaler = velocityKp + velocityKi + velocityKd
	if velocityScaler > 1 :
		velocityScaler = 1
	elif velocityScaler < -1 :
		velocityScaler = -1
	if abs(velocityScaler*motorVoltsMax) < motorVoltsMin:
		motorVoltCommand = 0
	else:
		motorVoltCommand = abs(voltsScaler*motorVoltsMax)
	
	# send motor commands
	if voltsScaler > 0 :
		controlMessage.motorDir = 0 #CW In
		controlMessage.desiredVoltage = motorVoltCommand
	elif voltsScaler < 0 :
		controlMessage.motorDir = 1 # CCW Out
		controlMessage.desiredVoltage = motorVoltCommand
	else:
		controlMessage.desiredVoltage = 0

	# publish messages
	controlMessage.header.stamp = rospy.Time.now()
	pub.publish(controlMessage)

def tensionControl():
	global pub
	rospy.init_node('tensionControl', anonymous=True)
	pub = rospy.Publisher('/motorControlInterface/read_motorCommand', motorCommand, queue_size=10)	
	rospy.Subscriber("/tether_tension", forceCell, getTension)
	rospy.Subscriber("/tether_angle", angleEncoder, getArmAngle)
	rospy.Subscriber("/tether_length", lengthEncoder, getTetherLength)
	rospy.Subscriber("/vehicle/out/odometry", Odometry, getVelocity)
	rospy.Subscriber("/stereo_odometer/odometry", Odometry, getPose)
	rospy.Subscriber("/joy_handoff", Empty, listener)		
	#rospy.init_node('tensionControl', anonymous=True) #why does this occur twice
	rospy.spin()
  
if __name__ == '__main__':
	try:		
		tensionControl()
	except rospy.ROSInterruptException: pass


# class TensionCommand
#   currentArmAngle
#   currentTetherLength
#   tethercallback(tetherLength)
#		currentTether = tetherLength
#		tetherLengthSet = true
#   anglecallback(armangle)
#		currentAngle = angleLength
#		armAngleSet = true
#		if(armAngleSet == true and tetherLengthSet == true && initialValuesSet == false)
#			setDefaultValues()
#			initialValuesSet = true
#   bool armAngleSet = false
#   bool tetherLengthSet = false
#   bool initialValuesSet = false
#
#def getInitialValues():
#	global currentTetherLength, currentArmAngle
#	vehiclePositionLast = np.array([ [currentTetherLength * np.cos(currentArmAngle)] , [currentTetherLength * np.sin(currentArmAngle),currentArmAngle] ]) 
#	anchorPositionEstimate = np.subtract(vehiclePositionLast,np.array([ [currentTetherLength * np.cos(currentArmAngle)] , [currentTetherLength * np.sin(currentArmAngle),currentArmAngle] ]) )

#	# first check if tension is within threshold
#	if currentTension < tensionFloor :
#		controlMessage.motorDir = 0 # CW In
#		controlMessage.desiredVoltage = motorVoltsMaxIn
#		print('Under Tension Limit: [%f]\n' % (currentTension))
#	elif currentTension > tensionCeiling :
#		controlMessage.motorDir = 1 # CCW Out
#		controlMessage.desiredVoltage = motorVoltsMaxOut
#		print('Over Tension Limit: [%f]\n' % (currentTension))
#
#	else :
#		# define full speed instance to occur when angle is 180 or 0
#		if 170.0 <= armAngle <= 190.0:  #convert to radians
#			if linearVelocity > vehicleVelocityThresh :
#				controlMessage.motorDir = 1 # CCW Out
#				controlMessage.desiredVoltage = motorVoltsMaxOut
#			elif linearVelocity < -vehicleVelocityThresh :
#				controlMessage.motorDir = 0 # CW In
#				controlMessage.desiredVoltage = motorVoltsMaxIn
#			else :
#				controlMessage.motorDir = 0 # reset
#				controlMessage.desiredVoltage = 0
#			print('Full Speed Range: ~180 deg\n')
#
#		elif armAngle >= 350.0 or armAngle <= 10.0 :	#convert to radians
#			if linearVelocity > vehicleVelocityThresh :
#				controlMessage.motorDir = 0 # CW In
#				controlMessage.desiredVoltage = motorVoltsMaxIn
#			elif linearVelocity < -vehicleVelocityThresh :
#				controlMessage.motorDir = 1 # CWW Out
#				controlMessage.desiredVoltage = motorVoltsMaxOut
#			else :
#				controlMessage.motorDir = 0 # reset
#				controlMessage.desiredVoltage = 0
#			print('Full Speed Range: ~0 deg\n')
#
#		# motor does not not operate in the range of 90 or 270
#		elif 80.0 <= armAngle <= 100.0 or 260.0 <= armAngle <= 280.0 :
#			controlMessage.motorDir = 0 # reset direction to zero
#			controlMessage.desiredVoltage = 0
#			print('Holding Range: ~90 ~270 deg\n')
#
#		# set velocity according to caclulated (above) tether velocity
#		else :		
#
#
#			if requiredTetherVelocity > tetherVelocityThresh and abs(linearVelocity) > vehicleVelocityThresh :
#				controlMessage.motorDir = 0 #CW In					
#				controlMessage.desiredVoltage = 2.5
#				print('State: Reel Out\n')
#
#			elif requiredTetherVelocity < -tetherVelocityThresh and abs(linearVelocity) > vehicleVelocityThresh :
#				controlMessage.motorDir = 1 #CCW Out		
#				controlMessage.desiredVoltage = 2.5
#				print('State: Reel In\n')
#	
#			else :
#				controlMessage.motorDir = 0 #Reset
#				controlMessage.desiredVoltage = 0
#
#
#
# WORKING VERSION / BUT NOT INTUITIVE
#			if requiredTetherVelocity < -tetherVelocityThresh :
#				controlMessage.motorDir = 1 #CWW Out							
#				controlMessage.desiredVoltage = 2.5
#				print(' State: Reel Out\n')
#
#			elif requiredTetherVelocity > tetherVelocityThresh :
#				controlMessage.motorDir = 0 #CW In		
#				controlMessage.desiredVoltage = 2.0
#				print(' State: Reel In\n')
#			else :
#				controlMessage.motorDir = 0 #Reset
#				controlMessage.desiredVoltage = 0


	# check what commands are sent
	#print('Current Motor Volts:\t[%f]\nDirection:\t[%i]\n\n' % (controlMessage.desiredVoltage, controlMessage.motorDir))

	# reset paramaters for next loop
	#vehiclePositionLast = vehiclePositionEstimate
	#lastTetherLength = currentTetherLength
	#secondsLast = seconds
