#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Empty
from asrl__sensors__trex.msg import *
from asrl__control__tether_management.msg import *
from asrl__drivers__labjack.msg import *
from asrl__drivers__labjack.srv import *
import numpy as np

currentTension = 0
currentInclination = 0
timeSinceLastForceReading = 0
#calculated 2nd degree polynomial coefficients from lift tests
polyA = 0.00009798
polyB = 0.021
polyC = 1.602
roverMass = 92.5 #kg
gravity = 9.8067 #m/s^2
# insert message timeout

def getTension(forceReading):
	global currentTension, cellVolts
	cellVolts = forceReading.Vout # get tension cell volts
	currentTension = gravity*abs(np.divide(np.subtract(np.sqrt(-4*polyA*polyC+4*polyA*cellVolts+np.square(polyB)),polyB),(2*polyA)) ) # N
		
	#print('current Tension %f\n\n' % currentTension)

def getInclination(inclinationReading):
	global currentInclination
	currentInclination = inclinationReading.vehicleAngleRad
	#print('current inclination %f\n\n' % currentInclination)

def listener(emptyMessage):
	global currentTension,currentInclination,cellVolts,pub
	controlMessage = motorCommand()
	desiredTension = roverMass*gravity*np.sin(abs(currentInclination)) #N
	tensionBuffer = 10.0 #N
	motorVoltage = 2.0 #volts, max is 5
	tensionDiff = currentTension - desiredTension #N
	print('current difference %f\n\n' % tensionDiff)
	if cellVolts >= poly C :
		if abs(tensionDiff) < tensionBuffer:
			controlMessage.desiredVoltage = 0.0
		elif tensionDiff < 0.0:
			controlMessage.motorDir = 0 #CW Reel In
			controlMessage.desiredVoltage = motorVoltage
		else:
			controlMessage.motorDir = 1 # CCW Pay Out
			controlMessage.desiredVoltage = motorVoltage
	else : # if voltage does not exceed steady volts reel in
		controlMessage.motorDir = 0 #CW Reel In
		controlMessage.desiredVoltage = motorVoltage

	controlMessage.header.stamp = rospy.Time.now()
	pub.publish(controlMessage)

def tensionControl():
	global pub
	rospy.init_node('tensionControl', anonymous=True)
	pub = rospy.Publisher('/motorControlInterface/read_motorCommand', motorCommand, queue_size=10)	
	rospy.Subscriber("/tether_tension", forceCell, getTension)
	rospy.Subscriber("/trex/inclination", inclination, getInclination)
	rospy.Subscriber("/joy_handoff", Empty, listener)	
	rospy.init_node('tensionControl', anonymous=True)
	rospy.spin()
  
if __name__ == '__main__':
	try:		
		tensionControl()
	except rospy.ROSInterruptException: pass
