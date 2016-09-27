#!/usr/bin/env python

#######################################################
########### Smart Hysterisis Controller ###############
#######################################################

############### import libraries and messages #########

import rospy
from std_msgs.msg import String, Empty
from sensor_msgs.msg import Joy
from asrl__sensors__trex.msg import *
from asrl__control__tether_management.msg import *
from asrl__drivers__labjack.msg import *
from asrl__drivers__labjack.srv import *
import numpy as np

############### Tunable Parameters ####################

motorVoltsMax = 5 # range: 0:5
motorVoltsMin = 0.01 # min operable voltage
setVoltsIn = 3.4 # 2.3 v = vicon, 2.45 = robot garage
setVoltsOut = 1.95

# PID Parameters 
decayRate = 0.90 # sets integrator to decayRate overtime
integrator = 0
Kp = 0.46 # last setting 0.6 
Ki = 0 # last setting 0.12 
Kd = 0 # last setting 0.1 


################ initialize variables #################

voltsErrorLast = 0
setVoltsLast = setVoltsOut
setScale = 0

############### side functions #######################

def getTension(forceReading): # get current tension in volts
	global currentVolts
	currentVolts = forceReading.Vout

def getDirection(data): # get the joy command direction
	global buttonForward,buttonBack
	buttonForward = data.axes[5] # Left Trigger
	buttonBack = data.axes[4] # Right Trigger

############### main function #######################

def listener(emptyMessage): # listens for empty joy message
	global currentVolts,controlPub,tensionPub,voltsErrorLast,integrator,setScale,buttonForward,buttonBack,setVoltsLast
	
	# messages to publish
	controlMessage = motorCommand()
	tensionMessage = tensionCommand()

	#################### PID ##########################
	
	#if setScale > 0 :
	#	setVolts = setVoltsIn
	#else :
	#	setVolts = setVoltsOut

	if buttonForward < 1 and buttonBack == 1:
		setVolts = setVoltsOut
		setVoltsLast = setVolts 
	elif buttonBack < 1 and buttonForward == 1:
		setVolts = setVoltsIn
		setVoltsLast = setVolts
	else :
		setVolts = setVoltsLast
	#print('SetVolts\t[%f]\n'% setVolts)

	
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
		motorVoltCommand = abs(voltsScaler*motorVoltsMax)

	############### send motor commands ###############

	if voltsScaler > 0 :
		controlMessage.motorDir = 0 #CW In
		setScale = 1 # in		

		if abs(motorVoltCommand * 1.5) < motorVoltsMax:
			controlMessage.desiredVoltage = motorVoltCommand
		else:
			controlMessage.desiredVoltage = motorVoltsMax

	elif voltsScaler < 0 :
		controlMessage.motorDir = 1 # CCW Out
		setScale = 0 # out

		controlMessage.desiredVoltage = motorVoltCommand

	else:
		controlMessage.desiredVoltage = 0

	############ generate message content #############

	controlMessage.header.stamp = rospy.Time.now()
	tensionMessage.header.stamp = rospy.Time.now()
	controlPub.publish(controlMessage)
	tensionPub.publish(tensionMessage)

	################ print statements #################
	#print('Direction (0=in,1=out): [%i]\n' % controlMessage.motorDir)
	#print('Desired Tension:\t[%f]\n'% tensionMessage.setVolts)
	#print('Current Tension:\t[%f]\n'% currentVolts)	
	#print('Tension Error:\t\t[%f]\n'% voltsError)
	#print('PID Scaler:\t\t[%f]\n' % voltsScaler)
	#print('Desired Motor Volts\t[%f]\n\n' % (controlMessage.desiredVoltage))

############ subscribers and publishers #############

def tensionControl():
	global controlPub,tensionPub
	rospy.init_node('tensionControl', anonymous=True)
	controlPub = rospy.Publisher('/motorControlInterface/read_motorCommand', motorCommand, queue_size=10)
	tensionPub = rospy.Publisher('/tensionCommand', tensionCommand, queue_size=10)	
	rospy.Subscriber("/tether_tension", forceCell, getTension)
	rospy.Subscriber("/joy_handoff", Empty, listener)
	rospy.Subscriber("joy", Joy, getDirection)
	rospy.init_node('tensionControl', anonymous=True)
	rospy.spin()

#################### main loop ######################
  
if __name__ == '__main__':
	try:		
		tensionControl()
	except rospy.ROSInterruptException: pass
