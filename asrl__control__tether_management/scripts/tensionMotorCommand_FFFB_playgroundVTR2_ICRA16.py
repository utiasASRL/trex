#!/usr/bin/env python

##########################################################################################

######### Max's Feedforward Steering Controller with PID Tension Feedback 2.0 ############

##########  THIS IS THE SECOND VERSION WITH PROPER FEEDBACK FORCE DEFINITION #############

##########################################################################################

############### import libraries and messages #########

import rospy
from std_msgs.msg import String, Empty
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from asrl__sensors__trex.msg import *
from asrl__control__tether_management.msg import *
from asrl__drivers__labjack.msg import *
from asrl__drivers__labjack.srv import *
import numpy as np


######## TODO ### TODO ### TODO ############
# Check slip condition on steep terrain.
# Is  it working?
# Is the slipCoefficient to high ? too low?
# What happens if you drive uphill?




####################### System Parameters  ################### 

showPrintCommands = 1 # 1=ON, 0=OFF

mu_friction = .45 #Friction coefficient / determines the reachable area



# The radius of the tether is modeled as an archimedean spiral: r = maxSpoolRadius - spoolGain * noTurns
spoolRadiusAtZeroTurns = .225 #.23 	# [m]
spoolRadiusAtMaxTurns = .195   	# [m]
maxSpoolTurns = 50		# [rad]

angularVelocity2motorCommand = 150/(14 * np.pi) 	# [omegaSpoolMax ~pi/2 => 5V ]
roverMassKG = 92.5 	# [kg]
linearSpeedMax = 0.135 	# [m/s]

inclineListLength = 20 	# number of inclination values to store
armGravityAngleListLength = 20	# number of armGravityAngle values to store
velocityListLength = 5 # number of velocity values to store

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
operationMode = 0	# 0: both, 	1: feedforward (ff) only, 	2: feedback (fb) only



# Adaptive Setpoints for Reference-Tension of the Feedback-Controller	
# ---------------------------------------------------------------------------------------------------------------------------------------------------------------
# ----- F_{V,ref} = max(polyA1 * F_{kg,ref} + polyA2 , tensionRefMin)    											
# -----		
# -----
# -----	Passive Mode (minimizes in-plane forces on trex):					     								
# ----- F_{kg,ref} = F_g*sin(inclination)*cos(armGravityAngle)    									
# -----
# ----
# -----	Active Mode  (maximizes in-plane forces in the direction of movement, without inducing slip):
# -----	F_{kg,ref} = F_g*sin(inclination)*cos(armGravityAngle) 
# -----		     + F_g * [ cos(armAngle) * directionOfMotion ] * [sqrt(mu_friction^2*cos(inclination)^2 - sin(inclination)^2*sin(armGravityAngle)^2)]
# -----			      \_________________ _______________/     \________________________________________ _______________________________________/
# -----									v							 							       v
# -----			      active supporting in dir. of motion up to 	  max. available force without violating slip-condition			
# -----
# ---------------------------------------------------------------------------------------------------------------------------------------------------------------


# Turn the setpointAdaptation OFF to set the setpoint (desiredTension) of the Feedback-Controller manually
setpointAdaptation = 1		# 1=ON, 0=OFF
#desiredTensionV = 2 		# [V] Set this variable, if setpointAdaptation is Off

# This is the 1st-order polynom that is used to transfrom a reference tension F_ref from [kg] to [V], see forceSensorCalibration.ods
polyA1 = 0.03  			# [V/kg] make sure 2V@10kg to maintain a minimal tension, 		
polyA2 = 1.7			# [V]	 make sure 2V@10kg to maintain a minimal tension, 		

minTensionV_base = 2  					# [V] 	 Minimum desiredTension which is fed into Feedback-Controller
minTensionV_extra = .15 # [V] Determines how much higher the minimal tension is when the arm points to the side. This is used to prevent dead-arm angles on flat ground. 


activeFeedbackControl = 1 	# ActiveMode: 1=ON, 0=OFF



# Adaptive gains Kp_in & Kp_out of Feedback-Controller
# ------------------------------------------------------------------------------------------
#
# 	The gains are a 2D-functions of inclination and armAngle. They decay with larger armAngles since the damping of the robot decreases.
#
#	see plot_2Dgains.m, 2D_GainScheduling.ods
#
# ------------------------------------------------------------------------------------------

# Turn the gainAdaptation OFF to set the gains Kp_in & Kp_out manually
gainAdaptation = 1	# 1=ON, 0=OFF
#Kp_in = 4 	# Set this variable, if gainAdaptation is OFF
#Kp_out = 2.5 	# Set this variable, if gainAdaptation is OFF

#loadingGainParameter
loadingA0 = 4.5 
loadingA90 = 2  
loadingC = -0.035 #was -0.046

#unloadingGainParameter
unloadingA0 = 3.5 
unloadingA90 = 1  
unloadingC = -0.035


####################### Initalize global variables  ################### 

currentInclinationRadAvg = 0					# [rad] Initalize inclination: this is required if IMU does not return measurements.
currentInclinationDegAvg = np.degrees(currentInclinationRadAvg)	# [deg] Initalize inclination in degrees, as well
inclineList = list()

currentArmGravityAngleRadAvg = np.pi 					# [rad] Initalize armGravityAngle: this is required if IMU does not return measurements.
currentArmGravityAngleDegAvg = np.degrees(currentArmGravityAngleRadAvg)	# [deg] Initalize armGravityAngle in degrees, as well
armGravityAngleList = list()

robotLinearVelocityAvg = 0	# [m/s] Initalize velocity
velocityList = list()

############### side functions #######################


def convertKG2V(desiredTensionKG): # Convert from KG2V to generate F_ref in V
        desiredTensionV = desiredTensionKG * polyA1 + polyA2
        return desiredTensionV

def convertV2KG(desiredTensionV): # Convert from V2KG to generate F_ref in kg
        desiredTensionKG = np.divide((desiredTensionV-polyA2), polyA1)
        return desiredTensionKG

def getTension(forceReading):  # get current tension in volts
	global tensionCurrentV, tensionCurrentKG
	tensionCurrentV = forceReading.Vout
	tensionCurrentKG = convertV2KG(tensionCurrentV)

def getArmAngle(armAngleReading):
	global armAngleDeg, armAngleRad
	armAngleDeg = armAngleReading.Deg
	armAngleRad = np.radians(armAngleDeg)

def getVelocityJoy(data): # get the joy command direction
	global joyLinearCommand
	joyLinearCommand = data.axes[4] # right joystick (vertical copmponent)

def getrobotLinearVelocity(cmdVelocity): # gets commanded vehicle velocity in m/s
	global robotLinearVelocity
	robotLinearVelocity = cmdVelocity.linear.x

def getSpoolTurns(lengthReading) :
	global numSpoolTurns
	numSpoolTurns = lengthReading.Turns

def getInclination(inclinationReading):
	global currentInclinationRad 
	currentInclinationRad = inclinationReading.inclinationRad	

# Explanation: The ArmGravityAngle is the angle between the robot arm and the projection of the gravity vector in the 
# xy-plane of the robot's body frame. Depending on this angle, the desiredTension is scaled. ->see publisherNode for more detailed explanation.
# For ArmGravityAngle = 180, robot drives straight down-hill
# For ArmGravityAngle = 90 or 270, robot drives parallel to the slope
# For ArmGravityAngle = 0, the robots drives straight up-hill
def getArmGravityAngle(armGravityAngleReading):
	global currentArmGravityAngleRad, currentArmGravityAngleDeg
	currentArmGravityAngleDeg = armGravityAngleReading.armGravityAngleDeg
	currentArmGravityAngleRad = np.radians(currentArmGravityAngleDeg)	


# A filter function, calculating the average over the past n values.
#
# Inclination is in [-90,90] with no jumps.
#
def updateInclination():
	global inclineList, currentInclinationRadAvg,currentInclinationDegAvg

	if 'currentInclinationRad' in globals() :
		if len(inclineList) > inclineListLength : 	# if more than max. no of elements in list
			inclineList.pop(0) 			# delete item at index zero
		inclineList.append(currentInclinationRad) 	# add latest measurement
		del globals()['currentInclinationRad'] 		# delete global variable, to capture when IMU freezes

		currentInclinationRadAvg = np.mean(inclineList)
		currentInclinationDegAvg = np.degrees(currentInclinationRadAvg)
	else : 
		#print('Inclination not available! Using default / last known inclination: \t\t[%f]\n' % currentInclinationDegAvg)
		rospy.logwarn('Inclination not available! Using default / last known inclination: \t\t\t[%f]\n' % currentInclinationDegAvg)

# A filter function, calculating the average over the past n values.
#
# ArmGravityAngle is in [0,360] with possible jumps between 0 and 360.
# This makes filtering more difficult
#
def updateArmGravityAngle():
	global armGravityAngleList, currentArmGravityAngleRadAvg, currentArmGravityAngleDegAvg

	if 'currentArmGravityAngleRad' in globals() :
		if len(armGravityAngleList) > armGravityAngleListLength : 		# if more than max. no of elements in list
			armGravityAngleList.pop(0) 					# delete item at index zero

		#currentArmGravityAngleRad = np.arcsin(np.sin(currentArmGravityAngleRad))  #TODO: check if this is correct.
		armGravityAngleList.append(((currentArmGravityAngleRad))) 			# add latest measurement

		del globals()['currentArmGravityAngleRad'] 				# delete global variable, to capture when IMU freezes
		
		currentArmGravityAngleRadAvg = np.mean(armGravityAngleList)
		currentArmGravityAngleDegAvg = np.degrees(currentArmGravityAngleRadAvg)
	else : 
		#print('ArmGravityAngle not available! Using default / last known armGravityAngle: \t\t[%f]\n' % currentArmGravityAngleRadAvg)
		rospy.logwarn('ArmGravityAngle not available! Using default / last known armGravityAngle: \t\t[%f]\n' % currentArmGravityAngleDegAvg)

# A filter function, calculating the average over the past n values.
def updateVelocity():
	global velocityList, robotLinearVelocityAvg

	if 'robotLinearVelocity' in globals() :
		if len(velocityList) > velocityListLength : 	# if more than max. no of elements in list
			velocityList.pop(0) 			# delete item at index zero
		velocityList.append(robotLinearVelocity) 	# add latest measurement
		del globals()['robotLinearVelocity'] 		# delete global variable, to capture when IMU freezes

		robotLinearVelocityAvg = np.mean(velocityList)
	else : 
		#print('Velocity not available! Using default / last known velocity: \t\t[%f]\n' % robotLinearVelocityAvg)
		rospy.logwarn('Velocity not available! Using default / last known velocity: \t\t\t[%f]\n' % robotLinearVelocityAvg)

# The tether supports TReX motion by reducing the effect of gravity on the slope. Depending on the state, the resulting
# total force, F_total, may be smaller or larger than the friction, F_F, of TReX. If it is larger, TReX may start to slip.
# If it is smaller, TReX can drive safely, and! we could have some margin left. If so, we can exploit this margin to further
# support TReX motion (activeFeedback mode).
#
#	see plot_referenceTension.m
#
#----------------------------------------------------------------
def evalSlipCondition(desiredTensionKG): 	
	
	#slipCondition1a = np.abs(np.sin(currentArmGravityAngleRadAvg))  # to keep lines short
	#slipCondition1b = np.abs(np.tan(currentInclinationRadAvg))  # to keep lines short
	#slipCondition1 = slipCondition1a*slipCondition1b

	slipCondition2a = np.square(np.tan(currentInclinationRadAvg)) 
	slipCondition2b = 2* np.cos(currentArmGravityAngleRadAvg) * np.abs(np.tan(currentInclinationRadAvg)) * np.divide(1,np.cos(currentInclinationRadAvg)) * np.divide(desiredTensionKG,roverMassKG)  
	slipCondition2c = np.square(np.divide(desiredTensionKG,roverMassKG)) * np.square(np.divide(1,np.cos(currentInclinationRadAvg))) 
	slipCondition2 = np.sqrt( slipCondition2a + slipCondition2b +  slipCondition2c )
	
	slipCondition = slipCondition2

	return slipCondition



# The minimum tension, F_min, scales with the armAngle. The robot is able to withstand a higher
# tension in equilibrium when the arm is pulling towards the side. This adaptation is done, to 
# prevent dead-zones when moving in small radius around a pole and the arm is at 90 deg while the 
# robot moves already towards the anchor.
def updateMinTension():
	global minTensionV, minTensionKG

	minTensionV = minTensionV_base + minTensionV_extra * np.abs(np.sin(armAngleRad))
	minTensionKG = convertV2KG(minTensionV)




############### main function #######################

def listener(emptyMessage):  # listens for empty joy message
	global Kp_in, Kp_out, desiredTensionV

	print('\n\n')	
	 
	################### Inclination available? / if yes: filter it ##################	

	updateInclination()

	################# ArmGravityAngle available? / if yes: filter it ################	
	
	updateArmGravityAngle()

	################# Velocity available? / if yes: filter it ################	
	
	updateVelocity()

	################# The minimum tension, F_min, is a function of the armAngle #############

	updateMinTension()

	########### Generating referenceTension that minimizes force, F_total, on TReX ###############

	if setpointAdaptation == 1:
		
		desiredTensionKG_base= np.max((-1) *roverMassKG * np.abs(np.sin(currentInclinationRadAvg)) *  np.cos(currentArmGravityAngleRadAvg)) 
		desiredTensionV_base = convertKG2V(desiredTensionKG_base)

		##################### slipCondition violated?  #############################	

		slipCondition_base = evalSlipCondition(desiredTensionKG_base)

		################### If some margin is left and activeMode is ON, we in/de-crease the reference tension in the direction of motion of TReX ########################
	
		if activeFeedbackControl == 1 and slipCondition_base <= mu_friction:

			directionOfMotion = - np.divide(robotLinearVelocityAvg,linearSpeedMax)  # in [-1,1] where -1: fullspeed backwards and +1: fullspeed forwards
			activeForceScaling = np.cos(armAngleRad) * directionOfMotion # Depending on wether we move up or downhill, back or forth, and in which direction, we scalte the available activeForce

			activeForceKG = roverMassKG * activeForceScaling * np.sqrt(np.square(mu_friction)*np.square(np.cos(currentInclinationRadAvg)) - np.square(np.sin(currentInclinationRadAvg))*np.square(np.sin(currentArmGravityAngleRadAvg)))
			activeForceV = convertKG2V(activeForceKG)-polyA2

			desiredTension_activeForceKG = desiredTensionKG_base + activeForceKG
			desiredTension_activeForceV = convertKG2V(desiredTension_activeForceKG)

		else:

			directionOfMotion = - np.divide(robotLinearVelocityAvg,linearSpeedMax)
			activeForceScaling = np.cos(armAngleRad) * directionOfMotion	

			activeForceKG = 0
			activeForceV = convertKG2V(activeForceKG) - polyA2

			desiredTension_activeForceKG = desiredTensionKG_base + activeForceKG
			desiredTension_activeForceV = convertKG2V(desiredTension_activeForceKG)


		#################### Re-Generating the referenceTension, if activeMode is ON ##################################
		
		desiredTensionKG = max(desiredTension_activeForceKG,minTensionKG)
		desiredTensionV = convertKG2V(desiredTensionKG)


		##################### slipCondition violated?  #############################	

		slipCondition = evalSlipCondition(desiredTensionKG)

		if slipCondition>mu_friction:

			print('SlipCondition violated: Save movement of TReX cannot be guaranteed!')
			rospy.loginfo('SlipCondition violated: Save movement of TReX cannot be guaranteed!')






	if setpointAdaptation == 0: 
		
		##################### slipCondition violated?  #############################	
		desiredTensionKG = convertV2KG(desiredTensionV)		

		slipCondition = evalSlipCondition(desiredTensionV)

		if slipCondition>mu_friction :
			print('SlipCondition violated: Save movement of TReX cannot be guaranteed!')
			rospy.loginfo('SlipCondition violated: Save movement of TReX cannot be guaranteed!')

		print('setpointAdaptation = OFF : Using Manual Reference Tension [V]: \t [%f]\n' % desiredTensionV)
		rospy.loginfo('setpointAdaptation = OFF : Using Manual Reference Tension [V]: \t [%f]\n' % desiredTensionV)



	############## Feedback: P-controller tracks reference tension #################

	tensionErrorV = desiredTensionV - tensionCurrentV 	# current error in V
	tensionErrorKG = desiredTensionKG - tensionCurrentKG	# current error in KG

	if tensionErrorV < 0 : 	# tension is too high -> reel out/unloading

		if gainAdaptation == 1:
			K_out0 = unloadingA0 * np.exp(unloadingC * np.abs(currentInclinationDegAvg))
			K_out90 = unloadingA90 * np.exp(unloadingC * np.abs(currentInclinationDegAvg))
			Kp_out =-np.abs( np.mod(armAngleDeg + 90,180)-90) * (K_out0 - K_out90)/90 + K_out0

		else:
			print('gainAdaptation = OFF :  Using manually defined Gain, Kp_out: \t [%f]\n' % Kp_out)
			rospy.loginfo('gainAdaptation = OFF :  Using manually defined Gain, Kp_out: \t [%f]\n' % Kp_out)
		motorVoltCommand_fb = Kp_out * tensionErrorV

	else:	# tension is too low -> reel in/loading

		if gainAdaptation == 1:
			K_in0 = loadingA0 * np.exp(loadingC * np.abs(currentInclinationDegAvg))
			K_in90 = loadingA90 * np.exp(loadingC * np.abs(currentInclinationDegAvg))
			Kp_in = -np.abs( np.mod(armAngleDeg + 90,180)-90) * (K_in0 - K_in90)/90 + K_in0

		else:
			print('gainAdaptation = OFF : Using manually defined Gain, Kp_in: \t [%f]\n' % Kp_in)
			rospy.loginfo('gainAdaptation = OFF : Using manually defined Gain, Kp_in: \t [%f]\n' % Kp_in)
		motorVoltCommand_fb = Kp_in * tensionErrorV
	
	


	################## Feedforward: control inputs to tether command ###############
	

	spoolRadius = spoolRadiusAtZeroTurns - np.divide(spoolRadiusAtZeroTurns-spoolRadiusAtMaxTurns, maxSpoolTurns) * numSpoolTurns

	angularVelocitySpool = - robotLinearVelocityAvg * np.cos(armAngleRad) / spoolRadius # tetherLength [m], joyLinearVelocity [m/s]
		
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



	################ Generate ControllerStateMsg #####################
	controlStateMessage = controlState() # messages to publish

	if operationMode == 1:
		controlStateMessage.operationMode =(' [Feedforward ONLY]')
	elif operationMode == 2:
		controlStateMessage.operationMode =(' [Feedback ONLY]')
	else:
		controlStateMessage.operationMode =(' [Feedforward + Feedback]')

	controlStateMessage.currentInclinationRadAvg = currentInclinationRadAvg
	controlStateMessage.currentInclinationDegAvg = currentInclinationDegAvg
	controlStateMessage.currentArmGravityAngleRadAvg = currentArmGravityAngleRadAvg
        controlStateMessage.currentArmGravityAngleDegAvg = currentArmGravityAngleDegAvg
	controlStateMessage.slipCondition = slipCondition
	controlStateMessage.measuredTensionV = tensionCurrentV
	controlStateMessage.measuredTensionKG = tensionCurrentKG
	controlStateMessage.desiredTensionV = desiredTensionV
	controlStateMessage.desiredTensionKG = desiredTensionKG
	if setpointAdaptation == 1:
		controlStateMessage.activeForceV = activeForceV
		controlStateMessage.activeForceKG = activeForceKG
	controlStateMessage.tensionErrorV = tensionErrorV
	controlStateMessage.tensionErrorKG = tensionErrorKG
	if tensionErrorV < 0 :
		controlStateMessage.Kp_out = Kp_out
	else :	
		controlStateMessage.Kp_in = Kp_in
	controlStateMessage.motorVoltCommand_fb = motorVoltCommand_fb
	
	controlStateMessage.robotLinearVelocity = robotLinearVelocityAvg
	controlStateMessage.numSpoolTurns = numSpoolTurns
	controlStateMessage.spoolRadius = spoolRadius
	controlStateMessage.armAngleRad = armAngleRad
	controlStateMessage.motorVoltCommand_ff = motorVoltCommand_ff
	
	controlStateMessage.motorVoltCommand = motorVoltCommand	

	controlStateMessage.header.stamp = rospy.Time.now()

	controlStatePub.publish(controlStateMessage)


	################ Print Statements #################
	if showPrintCommands == 1:
		print('\n\n')
		if operationMode == 1:
			print('Operation Mode: \t\t [Feedforward ONLY] \n')
		elif operationMode == 2:
			print('Operation Mode: \t\t [Feedback ONLY]\n')
		else:
			print('Operation Mode: \t\t [Feedforward + Feedback]\n')
		print(' ')
		
		print('Inclination [Deg]: \t\t[%f]\n' % currentInclinationDegAvg)
		print('ArmGravityAngle [Deg]: \t\t[%f]\n' % currentArmGravityAngleDegAvg)
		
		print('SlipCondition [ ]: \t\t[%f]' % slipCondition)
		slipConditionMargin = mu_friction - slipCondition
		print('SlipConditionMargin [ ]: \t[%f]\n' % slipConditionMargin)
		
		print('Measured Tension [V]: \t\t[%f]' % tensionCurrentV)
		print('Measured Tension [KG]: \t\t[%f]\n' % tensionCurrentKG)
		
		print('Minimum Tension [V]: \t\t[%f]' % minTensionV)
		print('Minimum Tension [KG]: \t\t[%f]\n' % minTensionKG)

		print('Base Tension [V]: \t\t[%f]' % desiredTensionV_base)
		print('Base Tension [KG]: \t\t[%f]\n' % desiredTensionKG_base)
		
		if setpointAdaptation == 1:

			print('Direction of Motion: \t\t[%f]' %directionOfMotion)
			print('Active Force Scaling: \t\t[%f]\n' %activeForceScaling)

			print('Active Force [V]: \t\t[%f]' %activeForceV)
			print('Active Force [KG]: \t\t[%f]\n' %activeForceKG)

		if desiredTensionKG == minTensionKG:
			print('Reference Tension: \t\t[Minimum Tension]\n')
		elif desiredTensionKG == desiredTensionKG_base:
			print('Reference Tension: \t\t[Base Tension]\n')
		elif desiredTensionKG == desiredTension_activeForceKG and activeForceKG != 0:
			print('Reference Tension:  \t\t[Base Tension + Active Force]\n')
		else:
			print('Reference Tension: WHAT IS WRONG?')

		print('Reference Tension [V]:\t \t[%f]' % desiredTensionV)
		print('Reference Tension [KG]:\t \t[%f]\n' % desiredTensionKG)

		print('Tension Error [V]: \t\t[%f]' % tensionErrorV)
		print('Tension Error [KG]: \t\t[%f]\n' % tensionErrorKG)

		if tensionErrorV < 0 :
			print('Feedback Gain Kp_out: \t\t[%f]\n' % Kp_out)
		else :
			print('Feedback Gain Kp_in: \t\t[%f]\n' % Kp_in)
		print('Motor Command FB [V]: \t\t[%f]\n' % motorVoltCommand_fb)
		print(' ')
		print('Velocity Input [m/s]: \t\t[%f]\n' % robotLinearVelocityAvg)
		#print('Spool Turns [rad]:  \t\t[%f]\n' % numSpoolTurns )
		#print('Spool Radius [m]:  \t\t[%f]\n' % spoolRadius)
		#print('Cos(ArmAngle):  \t\t[%f]\n' % np.cos(armAngleRad))
		#print('Angular Velocity [rad/s]: \t[%f]\n' % angularVelocitySpool)
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
	rospy.Subscriber("/tether_length", lengthEncoder  , getSpoolTurns)
	rospy.Subscriber("/joy_handoff", Empty, listener)
	rospy.Subscriber("/joy", Joy, getVelocityJoy)
	rospy.Subscriber("/tether_angle", angleEncoder, getArmAngle)
	rospy.Subscriber("/inclination", inclination, getInclination)
	rospy.Subscriber("/armGravityAngle", armGravityAngle, getArmGravityAngle)
	rospy.Subscriber("/clearpath/robots/vehicle/cmd_vel", Twist, getrobotLinearVelocity)
	rospy.spin()



#################### main loop ######################

if __name__ == '__main__':
	try:
		tensionControl()
	except rospy.ROSInterruptException:
		pass
