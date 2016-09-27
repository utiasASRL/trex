#!/usr/bin/env python

##########################################################################################
########### Max's Feedforward Steering Controller with PID Tension Feedback ##############
##########################################################################################

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


####################### System Parameters  ################### 

showPrintCommands = 1 # 1=ON, 0=OFF

spoolRadiusMax = .21 	# [m]
spoolRadiusMin = .04 	# [m]
tetherLengthMax = 42 	# [m]
angularVelocity2motorCommand = 150/(14 * np.pi) 	# [omegaSpoolMax ~pi/2 => 5V ]
roverMass = 92.5 	# [kg]
linearSpeedMax = 0.1 	# [m/s]

currentInclinationRadAvg = 0  					# [rad] Initalize inclination: this is required if IMU does not return measurements.
currentInclinationDegAvg = np.degrees(currentInclinationRadAvg)	# [deg] Initalize inclination in degrees, as well
inclineListLength = 20 	# number of inclination values to store
inclineList = list()


useArmGravityScaling = 1 # 1=ON, 0=OFF
# Explanation: The ArmGravityAngle is the angle between the robot arm and the projection of the gravity vector in the 
# xy-plane of the robot's body frame. Depending on this angle, the desiredTension is scaled.
# For ArmGravityAngle = 180, robot drives straight down-hill
# For ArmGravityAngle = 90 or 270, robot drives parallel to the slope
# For ArmGravityAngle = 0, the robots drives straight up-hill
currentArmGravityAngleRadAvg = np.pi 					# [rad] Initalize armGravityAngle: this is required if IMU does not return measurements.
currentArmGravityAngleDegAvg = np.degrees(currentArmGravityAngleRadAvg)	# [deg] Initalize armGravityAngle in degrees, as well
armGravityAngleListLength = 20	# number of armGravityAngle values to store
armGravityAngleList = list()





######################################## Controller Parameters  ###################################
#
#		     	      __________________________
#		     	     |		         	|
#	Inputs ------------> |  Feedforward-Controller  |------.      _____________
#			     |__________________________|      :     |             |
#			      __________________________       :---->|    Robot    |--- Output ---.
#			     |			        |      :     |_____________|              :				 
#	Inputs ----(+)-----> |    Feedback-Controller   |------'		   	          :
#		    ^	     |__________________________|	      ____	   	   	  :
#		    :					 	     |    |   	   	      	  :
#		    '------------------------------------------------| -1 |-----------------------'
#								     |____|
#
# 			The operationMode determines which controllers are active.
#
operationMode = 0 	# 0: both, 	1: feedforward (ff) only, 	2: feedback (fb) only



# Adaptive Setpoints for Tension Feedback-Controller	
# ----------------------------------------------------------------------------------------
# -------------     F_{V,ref} = max(A1 * F_{kg,ref} + A2 , tensionRefMin)    -------------
# -------------								     -------------
# -------------     with F_{kg,ref} = roverMass*sin(currentInclination)      -------------
# ----------------------------------------------------------------------------------------

tensionRefMin = 2 #2  	# [V] 	 Minimum desiredTension which is fed into Feedback-Controller
polyA1 = 0.03  		# [V/kg] make sure 2V@10kg to maintain a minimal tension, 		based on InclinationTest01.ods,
polyA2 = 1.7 		# [V]	 make sure 2V@10kg to maintain a minimal tension, 		based on InclinationTest01.ods,

# Turn the setpointAdaptation OFF to set the setpoint (desiredTension) of the Feedback-Controller manually
setpointAdaptation = 1	# 1=ON, 0=OFF
#desiredTension = 2.2 	# [V] Set this variable, if setpointAdaptation is Off



# Adaptive gains Kp_in & Kp_out for Tension Feedback-Controller
# --------------------------------------------------------------------
# -------     Kp_x = A * exp(-C* currentInclinationDegAvg)     -------	
# -------  IT'S IN DEGREE, TO MAKE THE FITTING MORE INTUITIVE  -------
# --------------------------------------------------------------------

loadingGainA = 5.5	# increase this for higher gain  @ higher traction / lower  tension / lower  inclination,	based on InclinationTest02.ods
loadingGainC = 0.046	# increase this for faster decay @ lower  traction / higher tension / higher inclination,	based on InclinationTest02.ods
unloadingGainA = 4.5	# increase this for higher gain  @ higher traction / lower  tension / lower  inclination,	based on InclinationTest02.ods
unloadingGainC = 0.046	# increase this for faster decay @ lower  traction / higher tension / higher inclination,	based on InclinationTest02.ods

# Turn the gainAdaptation OFF to set the gains Kp_in & Kp_out manually
gainAdaptation = 1	# 1=ON, 0=OFF
#Kp_in = 2 	# Set this variable, if gainAdaptation is OFF
#Kp_out = 1 	# Set this variable, if gainAdaptation is OFF





############### side functions #######################

def getTension(forceReading):  # get current tension in volts
	global tensionCurrent
	tensionCurrent = forceReading.Vout

def getArmAngle(armAngleReading):
	global armAngleDeg, armAngleRad
	armAngleDeg = armAngleReading.Deg
	armAngleRad = np.radians(armAngleDeg)

def getVelocity(velocity):
	global linearVelocity, angularVelocity
	linearVelocity = velocity.twist.twist.linear.x
	angularVelocity = velocity.twist.twist.angular.z

def getVelocityJoy(data): # get the joy command direction
	global joyLinearCommand
	joyLinearCommand = data.axes[4] # right joystick (vertical copmponent)

def getTetherLength(lengthReading):
	global tetherLength
	tetherLength = lengthReading.Meters

def getInclination(inclinationReading):
	global currentInclinationRad 
	currentInclinationRad = inclinationReading.inclinationRad	

def convertKG2V(desiredTensionKG, polyA1, polyA2): # Convert from KG2V to generate F_ref in V
	desiredTensionV = desiredTensionKG * polyA1 + polyA2
	return desiredTensionV

def getArmGravityAngle(armGravityAngleReading):
	global currentArmGravityAngleRad, currentArmGravityAngleDeg
	currentArmGravityAngleDeg = armGravityAngleReading.armGravityAngleDeg
	currentArmGravityAngleRad = np.radians(currentArmGravityAngleDeg)	


############### main function #######################

def listener(emptyMessage):  # listens for empty joy message
	global Kp_in, Kp_out, desiredTension, inclineList, currentInclinationRadAvg, currentInclinationDegAvg, armGravityAngleList, currentArmGravityAngleRadAvg, currentArmGravityAngleDegAvg
	 
#	print('\n\n\n\n\n\n')

	##################### Check if inclination is available / if yes: filter it #############################	

	if 'currentInclinationRad' in globals() :
		if len(inclineList) > inclineListLength : 	# if more than max. no of elements in list
			inclineList.pop(0) 			# delete item at index zero
		inclineList.append(currentInclinationRad) 	# add latest measurement
		del globals()['currentInclinationRad'] 		# delete global variable, to capture when IMU freezes

		currentInclinationRadAvg = np.mean(inclineList)
		currentInclinationDegAvg = np.degrees(currentInclinationRadAvg)
	else : 
		#print('Inclination not available! Using default / last known inclination: \t\t[%f]\n' % currentInclinationDegAvg)
		rospy.logwarn('Inclination not available! Using default / last known inclination: \t\t[%f]\n' % currentInclinationDegAvg)



	##################### Check if armGravityAngle is available / if yes: filter it #############################	
	
	if 'currentArmGravityAngleRad' in globals() :
		if len(armGravityAngleList) > armGravityAngleListLength : 		# if more than max. no of elements in list
			armGravityAngleList.pop(0) 					# delete item at index zero
		armGravityAngleList.append(currentArmGravityAngleRad) 			# add latest measurement
		del globals()['currentArmGravityAngleRad'] 				# delete global variable, to capture when IMU freezes
		
		currentArmGravityAngleRadAvg = np.mean(armGravityAngleList)
		currentArmGravityAngleDegAvg = np.degrees(currentArmGravityAngleRadAvg)
	else : 
		#print('ArmGravityAngle not available! Using default / last known armGravityAngle: \t\t[%f]\n' % currentArmGravityAngleRadAvg)
		rospy.logwarn('ArmGravityAngle not available! Using default / last known armGravityAngle: \t\t[%f]\n' % currentArmGravityAngleDegAvg)




	##################### Generating reference tension #############################

	#TODO: The armGravityScaling is currently realized by a multiplication with a cosine. Every other function with the same period could be used for scaling the reference tension. The function should be determined in field test. cos(x) or cos(x)*abs(cos(x))

	if setpointAdaptation == 1:

		desiredTensionKG = roverMass*np.sin(currentInclinationRadAvg)
		if useArmGravityScaling == 1:
			desiredTensionKG = desiredTensionKG * (-1) * np.cos(currentArmGravityAngleRadAvg) # multiplication with (-1) to make the scaling largest at 180deg
#			desiredTensionKG = desiredTensionKG * (-1) * np.cos(currentArmGravityAngleRadAvg) * np.abs(np.cos(currentArmGravityAngleRadAvg)) # for steeper decay
#			desiredTensionKG = desiredTensionKG * (-1) * (1-np.abs(2/np.pi*np.arccos(np.cos(currentArmGravityAngleRadAvg)))) # for triangular decay
		desiredTensionV = convertKG2V(desiredTensionKG, polyA1, polyA2)
		desiredTension = max(desiredTensionV,tensionRefMin)
	
	else: 
		#print('setpointAdaptation = OFF : Using Manual Reference Tension [V]: \t [%f]\n' % desiredTension)
		rospy.loginfo('setpointAdaptation = OFF : Using Manual Reference Tension [V]: \t [%f]\n' % desiredTension)

	
	

	############## Feedback: P-controller tracks reference tension #################

	tensionError = desiredTension - tensionCurrent 	# current error

	if tensionError < 0 : 	# tension is too high -> reel out/unloading

		if gainAdaptation == 1:
			Kp_out = unloadingGainA * np.exp(-unloadingGainC * currentInclinationDegAvg)
		else:
			#print('gainAdaptation = OFF :  Using manually defined Gain, Kp_out: \t [%f]\n' % Kp_out)
			rospy.loginfo('gainAdaptation = OFF :  Using manually defined Gain, Kp_out: \t [%f]\n' % Kp_out)
		motorVoltCommand_fb = Kp_out * tensionError

	else:	# tension is too low -> reel in/loading

		if gainAdaptation == 1:
			Kp_in = loadingGainA * np.exp(-loadingGainC * currentInclinationDegAvg)
		else:
			#print('gainAdaptation = OFF : Using manually defined Gain, Kp_in: \t [%f]\n' % Kp_in)
			rospy.loginfo('gainAdaptation = OFF : Using manually defined Gain, Kp_in: \t [%f]\n' % Kp_in)
		motorVoltCommand_fb = Kp_in * tensionError
	
	


	################## Feedforward: control inputs to tether command ###############
	
	joyLinearVelocity = linearSpeedMax * joyLinearCommand
	spoolRadius = (spoolRadiusMax - (spoolRadiusMax-spoolRadiusMin)/tetherLengthMax*tetherLength)

	angularVelocitySpool = - joyLinearVelocity * np.cos(armAngleRad) / spoolRadius # tetherLength [m], joyLinearVelocity [m/s]
	
	motorVoltCommand_ff = angularVelocitySpool * angularVelocity2motorCommand # angularVelocitySpool [rad/s], angularVelocity2motorCommand [Vs/rad]




	##################### Generate messages  #######################################
	
	###### Generate MotorCommand
	controlMessage = motorCommand() # messages to publish

	if operationMode == 1 : 	#use only feedforward
		motorVoltCommand = motorVoltCommand_ff
	elif operationMode == 2 : 	#use only feedback
		motorVoltCommand = motorVoltCommand_fb
	else: 				#use both
		motorVoltCommand = motorVoltCommand_ff + motorVoltCommand_fb

	# actuator saturation
	if abs(motorVoltCommand) > 5 :
		motorVoltCommand = np.sign(motorVoltCommand) * 5

	# translate to motorVoltage and direction
	if motorVoltCommand > 0 :
		controlMessage.motorDir = 0 #CW In
		controlMessage.desiredVoltage = abs(motorVoltCommand)
	elif motorVoltCommand < 0 :
		controlMessage.motorDir = 1 # CCW Out
		controlMessage.desiredVoltage = abs(motorVoltCommand)
	else:
		controlMessage.motorDir = 0
		controlMessage.desiredVoltage = 0

	controlMessage.header.stamp = rospy.Time.now()
	controlPub.publish(controlMessage)



	##### Generate ControllerStateMsg
	controlStateMessage = controlState() # messages to publish

	if operationMode == 1:
		controlStateMessage.operationMode =(' [Feedforward ONLY]')
	elif operationMode == 2:
		controlStateMessage.operationMode =(' [Feedback ONLY]')
	else:
		controlStateMessage.operationMode =(' [Feedforward + Feedback]')

	controlStateMessage.currentInclinationRadAvg = currentInclinationRadAvg
	controlStateMessage.currentInclinationDegAvg = currentInclinationDegAvg
	controlStateMessage.desiredTension = desiredTension
	if tensionError < 0 :
		controlStateMessage.Kp_out = Kp_out
	else :	
		controlStateMessage.Kp_in = Kp_in
	controlStateMessage.motorVoltCommand_fb = motorVoltCommand_fb
	controlStateMessage.spoolRadius = spoolRadius
	controlStateMessage.motorVoltCommand_ff = motorVoltCommand_ff
	
	controlStateMessage.motorVoltCommand = motorVoltCommand	

	controlStateMessage.header.stamp = rospy.Time.now()

	controlStatePub.publish(controlStateMessage)



	################ print statements #################

	if showPrintCommands == 1:
		print('\n\n\n\n\n\n')
		if operationMode == 1:
			print('Operation Mode: \t\t [Feedforward ONLY] \n')
		elif operationMode == 2:
			print('Operation Mode: \t\t [Feedback ONLY]\n')
		else:
			print('Operation Mode: \t\t [Feedforward + Feedback]\n')
		print(' ')
		print('Measured Tension [V]: \t\t[%f]\n' % tensionCurrent)
		print('Inclination [Deg]: \t\t[%f]\n' % currentInclinationDegAvg)
		print('ArmGravityAngle [Deg]: \t\t[%f]\n' % currentArmGravityAngleDegAvg)
		print('Reference Tension [V]:\t \t[%f]\n' % desiredTension)
		print('Tension Error [V]: \t\t[%f]\n' % tensionError)
		if tensionError < 0 :
			print('Feedback Gain Kp_out: \t\t[%f]\n' % Kp_out)
		else :
			print('Feedback Gain Kp_in: \t\t[%f]\n' % Kp_in)
		print('Motor Command FB [V]: \t\t[%f]\n' % motorVoltCommand_fb)
		print(' ')
		print('Velocity Input [m/s]: \t\t[%f]\n' % joyLinearVelocity)
		print('Tether Length [m]:  \t\t[%f]\n' % tetherLength)
		print('Spool Radius [m]:  \t\t[%f]\n' % spoolRadius)
		print('Cos(ArmAngle):  \t\t[%f]\n' % np.cos(armAngleRad))
		print('Angular Velocity [rad/s]: \t[%f]\n' % angularVelocitySpool)
		print('Motor Command FF [V]: \t\t[%f]\n' % motorVoltCommand_ff)
		print(' ')
		print('Motor Command [V]: \t\t[%f]\n' % motorVoltCommand)
	


############ subscribers and publishers #############

def tensionControl():
	global controlPub, controlStatePub
	controlPub = rospy.Publisher('/motorControlInterface/read_motorCommand', motorCommand, queue_size=10)
	controlStatePub = rospy.Publisher('/motorControlInterface/controlState', controlState, queue_size=10)
	rospy.init_node('tensionControl', anonymous=True)
	rospy.Subscriber("/tether_tension", forceCell, getTension)
	rospy.Subscriber("/tether_length", lengthEncoder  , getTetherLength)
	rospy.Subscriber("/joy_handoff", Empty, listener)
	rospy.Subscriber("/joy", Joy, getVelocityJoy)
	rospy.Subscriber("/tether_angle", angleEncoder, getArmAngle)
	rospy.Subscriber("/vehicle/out/odometry", Odometry, getVelocity)
	rospy.Subscriber("/inclination", inclination, getInclination)
	rospy.Subscriber("/armGravityAngle", armGravityAngle, getArmGravityAngle)
	rospy.spin()



#################### main loop ######################

if __name__ == '__main__':
	try:
		tensionControl()
	except rospy.ROSInterruptException:
		pass
