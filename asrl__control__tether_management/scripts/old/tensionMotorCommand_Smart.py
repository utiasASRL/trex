#!/usr/bin/env python

#######################################################
########### Smart Hysterisis Controller ###############
#######################################################

############### import libraries and messages #########

import rospy
from std_msgs.msg import String, Empty
from asrl__sensors__trex.msg import *
from asrl__control__tether_management.msg import *
from asrl__drivers__labjack.msg import *
from asrl__drivers__labjack.srv import *
import numpy as np

############### Tunable Parameters ####################

motorVoltsMax = 3.5 # range: 0:5
motorVoltsMin = 0.01 # min operable voltage
sensedVoltsMin = 2.3 # 2.3 v = vicon, 2.45 = robot garage
inclinationMin = np.radians(5) # in deg converted to rads
inclinationMax = np.radians(90) # in deg converted to rads
listLength = 50 # number of inclination values to store
inclineThresh = 0.0001 #setting for minimum change (prior = 5e-5)

# PID Parameters 
decayRate = 0.90 # sets integrator to decayRate overtime
integrator = 0
Kp = 0.4 # last setting 0.6 
Ki = 0 # last setting 0.12 
Kd = 0 # last setting 0.1 

# THESE ARE BEST FOR 0 - 90 (but comp expensive)
# Loading Curve - 3rd degree poly y=Ax^3+Bx^2+Cx+D
polyA_load =  -0.2332
polyB_load = 0.087
polyC_load = 1.55
polyD_load = 1.982

# UnLoading Curve - 5th degree poly y=Ax^5+Bx^4+Cx^3+Dx^2+Ex+F
polyA_unload = -0.8518
polyB_unload = 4.465
polyC_unload = -7.874
polyD_unload = 4.381
polyE_unload = 1.428
polyF_unload = 2.176

# THESE ARE BEST FOR 0 - 50 degrees
# Loading Curve - 3rd degree poly y=Ax^3+Bx^2+Cx+D
#polyA_load =  -0.5767
#polyB_load = 0.6607
#polyC_load = 1.3
#polyD_load = 2.005

# UnLoading Curve - 3rd degree poly y=Ax^3+Bx^2+Cx+D
#polyA_unload = -0.9258
#polyB_unload = 0.093
#polyC_unload = 2.466
#polyD_unload = 2.103

################ initialize variables #################

setVoltsLast = 0
currentInclination = 0
voltsErrorLast = 0
loadStateLast = 0
inclineList = [0]*listLength
loadingUpdate = bool(0) 

############### side functions #######################

def getTension(forceReading): # get current tension in volts
	global currentVolts
	currentVolts = forceReading.Vout
	
def getInclination(inclinationReading): # get absolute incline in rads
	global currentInclination
	currentInclination = abs(inclinationReading.vehicleAngleRad)
	
def linreg(X,Y): # linear regression function for incline list array
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

def voltSet(incline,state) : # determine tension reference voltage
	global currentVolts
	if state == 0 : # loading
		voltReference = np.sum([polyA_load*np.power(incline,3),polyB_load*np.square(incline),polyC_load*incline,polyD_load])
	elif state == 1 : # unloading
		voltReference = np.sum([polyA_unload*np.power(incline,5),polyB_unload*np.power(incline,4),polyC_unload*np.power(incline,3),polyD_unload*np.square(incline),polyE_unload*incline,polyF_unload])
	elif state == 2 : # flat
		voltReference = sensedVoltsMin
	elif state == 3 : # vertical
		voltReference = currentVolts 
		# stops controller while alternating between vertical loading and unloading states
	return voltReference

############### main function #######################

def listener(emptyMessage): # listens for empty joy message
	global currentVolts,currentInclination,controlPub,tensionPub,voltsErrorLast,integrator,inclineList,loadStateLast,setVoltsLast
	
	# messages to publish
	controlMessage = motorCommand()
	tensionMessage = tensionCommand()

	# update voltage trend list
	inclineList.pop(0) #delete item at index zero
	inclineList.append(currentInclination) #add latest measurement
	inclineTrend, yInt = linreg(range(len(inclineList)),inclineList) #get trend (slope) of list
	
	########## select loading/unloading condition #####

	if abs(inclineTrend) >= inclineThresh :
		
		loadingUpdate = bool(1)		
		if currentInclination <= inclinationMin : # Flat Terrain
			tensionMessage.loadingState = 2
		elif currentInclination >= inclinationMax :
			tensionMessage.loadingState = 3
		elif inclineTrend > 0 : # Loading 
			tensionMessage.loadingState = 0

		elif inclineTrend < 0 : # Unloading
			tensionMessage.loadingState = 1
		else :
			print('ERROR: No State Selected!')

		tensionMessage.setVolts = voltSet(currentInclination,tensionMessage.loadingState)
	
	else:

		loadingUpdate = bool(0)
		tensionMessage.setVolts = setVoltsLast
		tensionMessage.loadingState = loadStateLast

	setVoltsLast = tensionMessage.setVolts
	loadStateLast = tensionMessage.loadingState

	#################### PID ##########################

	if tensionMessage.loadingState == 3 :
		voltsError = 0
	else :
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

	############### send motor commands ###############

	if voltsScaler > 0 :
		controlMessage.motorDir = 0 #CW In
		controlMessage.desiredVoltage = motorVoltCommand

	elif voltsScaler < 0 :
		controlMessage.motorDir = 1 # CCW Out
		controlMessage.desiredVoltage = motorVoltCommand

	else:
		controlMessage.desiredVoltage = 0

	############ generate message content #############

	controlMessage.header.stamp = rospy.Time.now()
	tensionMessage.header.stamp = rospy.Time.now()
	controlPub.publish(controlMessage)
	tensionPub.publish(tensionMessage)

	################ print statements #################
	print('0=load,1=unload,2-3:na: [%i]\n' % tensionMessage.loadingState)
	print('Direction (0=in,1=out): [%i]\n' % controlMessage.motorDir)
	print('Currently Loading?:\t[%s]\n' % loadingUpdate)
	print('Inclination Trend:\t[%f]\n' % inclineTrend)
	print('Current Incline:\t[%f]\n' % np.degrees(currentInclination) )
	print('Desired Tension:\t[%f]\n'% tensionMessage.setVolts)
	print('Current Tension:\t[%f]\n'% currentVolts)	
	print('Tension Error:\t\t[%f]\n'% voltsError)
	print('PID Scaler:\t\t[%f]\n' % voltsScaler)
	#print('PID Parameters\n\tP\t[%f]\n\tI\t[%f]\n\tD\t[%f]\n' % (voltsScaleKp,voltsScaleKi,voltsScaleKd))
	#print('Command Parameters\n\tsetVolts\t[%f]\n\tcurrentVolts\t[%f]\n\terror\t[%f]\n' % (tensionMessage.setVolts,currentVolts,voltsError))
	print('Desired Motor Volts\t[%f]\n\n' % (controlMessage.desiredVoltage))

############ subscribers and publishers #############

def tensionControl():
	global controlPub,tensionPub
	rospy.init_node('tensionControl', anonymous=True)
	controlPub = rospy.Publisher('/motorControlInterface/read_motorCommand', motorCommand, queue_size=10)
	tensionPub = rospy.Publisher('/tensionCommand', tensionCommand, queue_size=10)	
	rospy.Subscriber("/tether_angle", angleEncoder, getArmAngle)	
	rospy.Subscriber("/tether_tension", forceCell, getTension)
	rospy.Subscriber("/trex/inclination", inclination, getInclination)
	rospy.Subscriber("/joy_handoff", Empty, listener)	
	rospy.init_node('tensionControl', anonymous=True)
	rospy.spin()

#################### main loop ######################
  
if __name__ == '__main__':
	try:		
		tensionControl()
	except rospy.ROSInterruptException: pass
