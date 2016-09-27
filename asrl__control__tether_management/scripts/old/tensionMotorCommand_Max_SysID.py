#!/usr/bin/env python

#######################################################
########### Select Flat/Steep Controller ##############
#######################################################

############### import libraries and messages #########

import rospy
from std_msgs.msg import String, Empty
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Joy
from asrl__sensors__trex.msg import *
from asrl__control__tether_management.msg import *
from asrl__drivers__labjack.msg import *
from asrl__drivers__labjack.srv import *
import numpy as np




################ initialize variables #################
voltsErrorLast = 0
setVoltsLast = 1
setScale = 0
lastDirection = 1 # reelou (slow speed)



############### side functions #######################

def getTension(forceReading): # get current tension in volts
	global currentVolts
	currentVolts = forceReading.Vout

def getDirection(data): # get the joy command direction
	global driveDirection, lastDirection
	#TODO: set to parameter in case control input changes
	joyLinearCommand = data.axes[4] # Right Joystick (vertical copmponent)

	# check the desired drive direction
	# If joy stick exceeds 0.5 or -0.5 on a scale of [-1:1] set the controller
	if joyLinearCommand <= -0.5 and lastDirection == 0:
		driveDirection = 1 # backward/reelout
	elif joyLinearCommand >= 0.5 and lastDirection == 1:
		driveDirection = 0 # forward/reelin
	else:
		driveDirection = lastDirection
	lastDirection = driveDirection

def getArmAngle(armAngleReading):
	global armAngleDeg, armAngleRad
	armAngleDeg = armAngleReading.Deg
	armAngleRad = np.radians(armAngleDeg)
	
def getVelocity(velocity):
	global linearVelocity, angularVelocity
	linearVelocity = velocity.twist.twist.linear.x
	angularVelocity = velocity.twist.twist.angular.z

def getContolType(controlSelect):
	global setControlType
	setControlType = controlSelect.tetherControlSet


############### main function #######################

def listener(emptyMessage): # listens for empty joy message
	global currentVolts, controlPub, tensionPub


	# messages to publish
	controlMessage = motorCommand()
	tensionMessage = tensionCommand()

	controlMessage.desiredVoltage = 0
	
	
	############ generate message content #############

	controlMessage.header.stamp = rospy.Time.now()
	tensionMessage.header.stamp = rospy.Time.now()
	controlPub.publish(controlMessage)
	tensionPub.publish(tensionMessage)

	################ print statements #################
	print('Measured Tension: \t\t[%f]\n' % currentVolts)




############ subscribers and publishers #############

def tensionControl():
	global controlPub,tensionPub
	controlPub = rospy.Publisher('/motorControlInterface/read_motorCommand', motorCommand, queue_size=10)
	tensionPub = rospy.Publisher('/tensionCommand', tensionCommand, queue_size=10)
	rospy.Subscriber("/select_control", tetherControlType, getContolType)
	rospy.Subscriber("/tether_tension", forceCell, getTension)
	rospy.Subscriber("/joy_handoff", Empty, listener)
	rospy.Subscriber("joy", Joy, getDirection)
	rospy.Subscriber("/tether_angle", angleEncoder, getArmAngle)
	rospy.Subscriber("/vehicle/out/odometry", Odometry, getVelocity)
	rospy.init_node('tensionControl', anonymous=True)
	rospy.spin()

#################### main loop ######################
  
if __name__ == '__main__':
	try:		
		tensionControl()
	except rospy.ROSInterruptException: pass
