#!/usr/bin/env python

## Averaged with Hysterisis Dead Band Model ##

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
sensedVoltsMin = 2.3 # 2.3 v for vicon setting for vicon floor #2.45 for robot garage floor # TODO can this be set dyamically, also should account for longer tether lengths ### 2.25 for angled test
sensedDeadband = 0.00 # for flat ground, set tension deadband ### 0.025 for angled test
inclinationMin = 5 # in deg, later converted to radians

#initialize 
setVolts = 0
currentInclination = 0
voltsErrorLast = 0
#voltBufferLast = 0
#loadStateLast = 0

# trend capture
#listLength = 3
#voltList = [0]*listLength
#trendBuffer = 0.01

# Caclulated from Angled Plane Tests
# Linearization -  linear appx y=Ax+B
polyA = 1.860
polyB = 1.848
# Loading Curve - 2nd degree poly y=Ax^2+Bx+C
polyA_load = -0.3037
polyB_load = 1.357
polyC_load = 1.756
# UnLoading Curve - 3rd degree poly y=Ax^3+Bx^2+Cx+D
polyA_unload = -0.7227
polyB_unload = 0.7271
polyC_unload = 2.156
polyD_unload = 1.893

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
	currentInclination = 0 #TODO abs(inclinationReading.vehicleAngleRad)
	
def linreg(X,Y): # perform linear regression on array
	N = len(X)
	Sx = Sy = Sxx = Syy = Sxy = 0.0
	for x, y in zip(X,Y):
		Sx = Sx + x
		Sy = Sy + y
		Sxx = Sxx + x*x
		Syy = Syy + y*y
		Sxy = Sxy + x*y
	det = Sxx * N - Sx * Sx
	return (Sxy * N - Sy * Sx)/det, (Sxx * Sy -Sx * Sxy)/det

def listener(emptyMessage):
	global currentVolts,currentInclination,controlPub,tensionPub,voltsErrorLast,integrator #,voltList,voltBufferLast,loadStateLast
	
	# messages to publish
	controlMessage = motorCommand()
	tensionMessage = tensionCommand()

	# update voltage trend list
	#voltList.pop(0) #delete item at index zero
	#voltList.append(currentVolts) #add latest measurement
	#voltTrend,discard = linreg(range(len(voltList)),voltList) #get trend (slope) of list
	
	# select loading/unloading condition
	#if abs(voltTrend) >= trendBuffer :
	#	if voltTrend > 0 : # Loading 
	#loadingCurve = np.sum([polyA_load*np.square(currentInclination),polyB_load*currentInclination,polyC_load])
	#tensionMessage.loadingState = 0
	#	elif voltTrend < 0 : # Unloading
	#unloadingCurve = np.sum([polyA_unload*np.power(currentInclination,3),polyB_unload*np.square(currentInclination),polyC_unload*currentInclination,polyD_unload])
	#tensionMessage.loadingState = 1
	#else:
	#	tensionMessage.voltBuffer = voltBufferLast
	#	tensionMessage.loadingState = loadStateLast
	#voltBufferLast = tensionMessage.voltBuffer	
	#loadStateLast = tensionMessage.loadingState

	# PID
	# check current inclination for control setting
	if currentInclination <= np.radians(inclinationMin) : # conidered as relative flat ground
		tensionMessage.setVolts = sensedVoltsMin
		loadingCurve = sensedVoltsMin - sensedDeadband
		unloadingCurve = sensedVoltsMin + sensedDeadband
	else:
		tensionMessage.setVolts = np.sum([polyA*currentInclination,polyB])
		loadingCurve = np.sum([polyA_load*np.square(currentInclination),polyB_load*currentInclination,polyC_load])
		unloadingCurve = np.sum([polyA_unload*np.power(currentInclination,3),polyB_unload*np.square(currentInclination),polyC_unload*currentInclination,polyD_unload])
	
	if  loadingCurve <= currentVolts <= unloadingCurve :
		voltsError = 0
	else:
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
	print('Command Parameters\n\tsetVolts\t[%f]\n\tcurrentVolts\t[%f]\n\tbufferVolts\t[%f]\n\terror\t[%f]\n' % (tensionMessage.setVolts,currentVolts,tensionMessage.voltBuffer,voltsError))
	print('Control Output\n\tmotorVolts\t[%f]\n\n' % (controlMessage.desiredVoltage))

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
