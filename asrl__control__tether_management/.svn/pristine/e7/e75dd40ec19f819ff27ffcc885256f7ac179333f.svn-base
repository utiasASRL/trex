#!/usr/bin/env python

## Average Hysterisis Fit Model ##

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
sensedVoltsMin = 2.25 # TODO can this be set dyamically
inclinationMin = 5 # in deg, later converted to radians

# Initialize
setVolts = 0
currentInclination = 0
voltsErrorLast = 0

# Caclulated from Angled Plane Tests
# linear appx y=Ax+B
polyA = 1.860
polyB = 1.848

# Adjustable PID Parameters 
#(Zeigler-Nichols tuning method) NOT USED
#pscale = .45 # settings --  P:0.50, PI:0.45, PID:0.60 
#iscale = 1.2 # settings -- P:0, PI 1.20, PID:2.00
#kp = pscale*1.2 # pscale*Ku, Ku = first inducing oscillation (Ku = 1.2)
#Ki = (iscale*Kp)/1 # iscale*Kp / Pu, Pu is period of oscillation (Pu = 1 sec)
#Kd = (Kp*1)/8 # deriviative = Kp*Pu/8 
decayRate = 0.90 # sets integrator to decayRate overtime
integrator = 0
Kp = 0.6 # last val setting 0.6 
Ki = 0.12 # last val  setting 0.15 
Kd = 0.1 # last val  setting 0.1 

def getTension(forceReading):
	global currentVolts
	currentVolts = forceReading.Vout
	
def getInclination(inclinationReading):
	global currentInclination
	currentInclination = abs(inclinationReading.vehicleAngleRad) #test at 15deg = 0.2618 rad

def listener(emptyMessage):
	global currentVolts,setVolts,currentInclination,controlPub,tensionPub,voltsErrorLast,integrator
	
	# messages to publish
	controlMessage = motorCommand()
	tensionMessage = tensionCommand()

	# PID
	# check current inclination for control setting
	if currentInclination <= np.radians(inclinationMin) : # conidered as relative flat ground
		tensionMessage.setVolts = sensedVoltsMin
	else:
		tensionMessage.setVolts = np.sum([polyA*currentInclination,polyB])
	voltsError = tensionMessage.setVolts - currentVolts

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
	
	# send motor commands
	if voltsScaler > 0 :
		controlMessage.motorDir = 0 #CW In
		controlMessage.desiredVoltage = motorVoltCommand
	elif voltsScaler < 0 :
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
	print('PID Parameters\n\tP\t[%f]\n\tI\t[%f]\n\tD\t[%f]\n' % (voltsScaleKp,voltsScaleKi,voltsScaleKd))
	print('Tension Parameters\n\tsetVolts\t[%f]\n\ttrueVolts\t[%f]\n\terror\t[%f]\n' % (tensionMessage.setVolts,currentVolts,voltsError))
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
