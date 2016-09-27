#!/usr/bin/env python

## Physics Based Model ##

import rospy
from std_msgs.msg import String, Empty
from asrl__sensors__trex.msg import *
from asrl__control__tether_management.msg import *
from asrl__drivers__labjack.msg import *
from asrl__drivers__labjack.srv import *
import numpy as np

# Adjustable Variables
motorVoltsMax = 5 # range: 0:5
motorVoltsMin = 0.01 # min operable voltage
sensedTensionMin = 20.0 # TODO can this be set dyamically
inclinationMin = 5 # in deg, later converted to radians
roverMass = 92.5 #kg

# Initialize
currentTension = 0
currentInclination = 0
tensionErrorLast = 0

# 2nd degree poly y=Ax^2+Bx+C
polyA = 9.798E-5
polyB = 0.021
polyC = 1.7 #cacluated as 1.602, tuned due to offset

# PID Parameters 
#(Zeigler-Nichols tuning method) NOT USED
#pscale = .45 # settings --  P:0.50, PI:0.45, PID:0.60 
#iscale = 1.2 # settings -- P:0, PI 1.20, PID:2.00
#kp = pscale*1.2 # pscale*Ku, Ku = first inducing oscillation (Ku = 1.2)
#Ki = (iscale*Kp)/1 # iscale*Kp / Pu, Pu is period of oscillation (Pu = 1 sec)
#Kd = (Kp*1)/8 # deriviative = Kp*Pu/8 
decayRate = 0.90 # sets integrator to decayRate overtime
integrator = 0
Kp = 0.02 # 0.6 used for volt PIDs
Ki = 0.004 # 0.12 used for volt PIDs, should be 20% of Kp
Kd = 0.0067 # 0.1 used for volt PIDs

def getTension(forceReading):
	global currentVolts
	currentVolts = forceReading.Vout # get tension cell volts

def getInclination(inclinationReading):
	global currentInclination
	currentInclination = abs(inclinationReading.vehicleAngleRad) # 0.2618 = 15 deg
	
def listener(emptyMessage):
	global currentTension,currentInclination,currentVolts,controlPub,tensionPub,tensionErrorLast,integrator

	#messages to publish	
	controlMessage = motorCommand()
	tensionMessage = tensionCommand()
	tensionMessage.currentTension = abs(np.divide(np.subtract(np.sqrt(-4*polyA*polyC+4*polyA*currentVolts+np.square(polyB)),polyB),(2*polyA)) ) # KgF
	
	# PID
	# check current inclination for control setting
	if currentInclination <= np.radians(inclinationMin) : # conidered as relative flat ground
		tensionMessage.desiredTension = sensedTensionMin
	else:
		tensionMessage.desiredTension = roverMass*np.sin(currentInclination)

	tensionError = tensionMessage.desiredTension - tensionMessage.currentTension
	tensionScaleKp = tensionError * Kp
	integrator = (decayRate * integrator) + tensionError
	tensionScaleKi = integrator * Ki
	tensionScaleKd = (tensionError - tensionErrorLast) * Kd
	tensionErrorLast = tensionError
	tensionScaler = tensionScaleKp + tensionScaleKi + tensionScaleKd
	if tensionScaler > 1 :
		tensionScaler = 1
	elif tensionScaler < -1 :
		tensionScaler = -1
	if abs(tensionScaler*motorVoltsMax) < motorVoltsMin:
		motorVoltCommand = 0
	else :
		motorVoltCommand = abs(tensionScaler*motorVoltsMax)

	# send motor commands
	if tensionScaler > 0 :
		controlMessage.motorDir = 0 #CW In
		controlMessage.desiredVoltage = motorVoltCommand
	elif tensionScaler < 0 :
		controlMessage.motorDir = 1 # CCW Out
		controlMessage.desiredVoltage = motorVoltCommand
	else:
		controlMessage.desiredVoltage = 0

	# generate message content
	controlMessage.header.stamp = rospy.Time.now()
	tensionMessage.header.stamp = rospy.Time.now()
	controlPub.publish(controlMessage)
	tensionPub.publish(tensionMessage)
	
	#print statements
	print('PID Parameters\n\tP\t[%f]\n\tI\t[%f]\n\tD\t[%f]\n' % (tensionScaleKp,tensionScaleKi,tensionScaleKd))
	print('Command Parameters\n\tsetTension\t[%f]\n\ttrueTension\t[%f]\n\terror\t[%f]\n' % (tensionMessage.desiredTension,tensionMessage.currentTension,tensionError))
	print('Control Output\n\tmotorVolts\t[%f]\n\n' % controlMessage.desiredVoltage)
	
def tensionControl():
	global controlPub,tensionPub
	rospy.init_node('tensionControl', anonymous=True)
	controlPub = rospy.Publisher('/motorControlInterface/read_motorCommand', motorCommand, queue_size=10)
	tensionPub = rospy.Publisher('/tensionCommand', tensionCommand, queue_size=10)	
	rospy.Subscriber("/tether_tension", forceCell, getTension)
	rospy.Subscriber("/trex/inclination", inclination, getInclination)
	rospy.Subscriber("/joy_handoff", Empty, listener)	
	rospy.init_node('tensionControl', anonymous=True)
	rospy.spin()
  
if __name__ == '__main__':
	try:		
		tensionControl()
	except rospy.ROSInterruptException: pass
