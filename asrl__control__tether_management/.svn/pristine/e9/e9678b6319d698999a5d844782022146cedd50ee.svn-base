#!/usr/bin/env python

#######################################################
########### Simple Assistive Controller ###############
#######################################################

############### import libraries and messages #########

import rospy
from std_msgs.msg import String, Empty
from nav_msgs.msg import Odometry
from asrl__sensors__trex.msg import *
from asrl__control__tether_management.msg import *
from asrl__drivers__labjack.msg import *
from asrl__drivers__labjack.srv import *
import numpy as np

############### Tunable Parameters ####################

motorVoltsMax = 3.25 # range: 0:5 #3.25 for dome demo
motorVoltsMin = 0.01 # min operable voltage
sensedVoltsMin = 0 # set to zero to avoid control issues
vehicleMax = 0.1 # top vehicle speed in m/s 
cw_gain = 1 #TODO sets additional gain for reeling in (prevents sag). Set to 1 if nomial was 1.1
ccw_gain = 0.6 #TODO sets reduced gain for reeling out (prevents drift). Set to 1 if nomial was 0.4
#TODO can this be imported ~/base/linear_max_speed_meters_per_second

# Operable Arm Angle for Motor Actions
angleRange = 170 # degrees
range1A = 180 - (angleRange/2)
range1B = 180 + (angleRange/2)
range2A = 360 - (angleRange/2)
range2B =   0 + (angleRange/2)

# PID Parameters 
decayRate = 0.90 # sets integrator to decayRate over time
integrator = 0
Kp = 0.6 # last setting 0.6 
Ki = 0.12 # last setting 0.12 
Kd = 0 # last setting 0.1

################ initialize variables #################

setVoltsLast = 0
voltsErrorLast = 0

############### side functions #######################

def getTension(forceReading): # get current tension in volts
	global currentVolts
	currentVolts = forceReading.Vout

def getArmAngle(armAngleReading):
	global armAngleDeg, armAngleRad
	armAngleDeg = armAngleReading.Deg
	armAngleRad = np.radians(armAngleDeg)
	
def getVelocity(velocity):
	global linearVelocity, angularVelocity
	linearVelocity = (velocity.twist.twist.linear.x)
	angularVelocity = velocity.twist.twist.angular.z

############### main function #######################

def listener(emptyMessage): # listens for empty joy message
	global currentVolts, controlPub, tensionPub, voltsErrorLast, integrator, setVoltsLast, linearVelocity, angularVelocity, armAngleDeg, armAngleRad
	
	# messages to publish
	controlMessage = motorCommand()

	################# Voltage Select ####################

	# set minimum tension of tether
	if currentVolts < sensedVoltsMin :
		setVolts = sensedVoltsMin
		# PID
		voltsError = setVolts - currentVolts
		voltsScaleKp = voltsError * Kp
		integrator = (decayRate * integrator) + voltsError
		voltsScaleKi = integrator * Ki
		voltsScaleKd = (voltsError - voltsErrorLast) * Kd
		voltsErrorLast = voltsError
		voltsScaler = voltsScaleKp + voltsScaleKi + voltsScaleKd
		
		if voltsScaler > 1 :
			voltsScaler = 1
		elif voltsScaler < -1 :
			voltsScaler = -1
		if abs(voltsScaler*motorVoltsMax) < motorVoltsMin:
			motorVoltCommand = 0
		else :
			motorVoltCommand = voltsScaler*motorVoltsMax
		#print('Tension Mode: Active\n')

	# choose a proper set voltage based on arm angle	
	else :
		if range1A <= armAngleDeg <= range1B :
			direction = 1
			#print('Arm Angle: 180\n')
		elif armAngleDeg >= range2A or armAngleDeg <= range2B :
			direction = -1
			#print('Arm Angle: 0\n')
		else :
			direction = 0

		if linearVelocity <= 0 :
			offset = ccw_gain #TODO this adds a gain based on direction of reeling
		else :
			offset = cw_gain #TODO
		
		motorVoltCommand = ( linearVelocity / vehicleMax ) * motorVoltsMax * direction * abs(np.cos(armAngleRad))*offset #TODO

		print('motorVoltCommand: [%f]\n' % motorVoltCommand )
		if motorVoltCommand >= motorVoltsMax :
			motorVoltCommand = motorVoltsMax
		elif motorVoltCommand <= -motorVoltsMax :
			motorVoltCommand = -motorVoltsMax

		if abs(motorVoltCommand) < motorVoltsMin:
			motorVoltCommand = 0

	############### send motor commands ###############

	if motorVoltCommand > 0 :
		controlMessage.motorDir = 0 #CW In
		controlMessage.desiredVoltage = abs(motorVoltCommand)

	elif motorVoltCommand < 0 :
		controlMessage.motorDir = 1 # CCW Out
		controlMessage.desiredVoltage = abs(motorVoltCommand)

	else:
		controlMessage.desiredVoltage = 0

	############ generate message content #############

	controlMessage.header.stamp = rospy.Time.now()
	controlPub.publish(controlMessage)

	################ print statements #################
	#print('Direction (0=in,1=out): [%i]\n' % controlMessage.motorDir)
	#print('Current Tension:\t[%f]\n'% currentVolts)	
	#print('Desired Motor Volts\t[%f]\n\n' % (controlMessage.desiredVoltage))

############ subscribers and publishers #############

def tensionControl():
	global controlPub,tensionPub
	rospy.init_node('tensionControl', anonymous=True)
	controlPub = rospy.Publisher('/motorControlInterface/read_motorCommand', motorCommand, queue_size=10)
	tensionPub = rospy.Publisher('/tensionCommand', tensionCommand, queue_size=10)
	rospy.Subscriber("/tether_angle", angleEncoder, getArmAngle)
	rospy.Subscriber("/vehicle/out/odometry", Odometry, getVelocity)	
	rospy.Subscriber("/tether_tension", forceCell, getTension)
	rospy.Subscriber("/joy_handoff", Empty, listener)	
	rospy.init_node('tensionControl', anonymous=True)
	rospy.spin()

#################### main loop ######################
  
if __name__ == '__main__':
	try:		
		tensionControl()
	except rospy.ROSInterruptException: pass
