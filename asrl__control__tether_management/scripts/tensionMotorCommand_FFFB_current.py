#!/usr/bin/env python

##########################################################################################
######### Max's Feedforward Steering Controller with Tension Feedback         ############
######### For more information, please contact:  			      ############
#########		max.polzin@robotics.utias.utoronto.ca or 	      ############
#########		polzin.max@gmail.com  				      ############
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

# IMPORTANT NOTE: You have to understand the meaning of polyA1, polyA2, and minTensionV_base 
# If force sensor has changed, it is NOT sufficient to solely increase minTensionV_base
# For more information, see forceSensorCalibration.ods

# for arm gravity angle close to 0 and/or 360 the filtering fails due to discontinuities / only relevant when TReX moves above the last anchor point

###################################################################
####################### Calibration Parameters  ################### 
###################################################################

# mu_friction should be the only tweaking parameter
# ~0.2 slippery
# ~0.4 not slippery
init_mu_friction = .2 #initial friction coefficient is handed over to the parameter server.


# The radius of the tether is modeled as an archimedean spiral.
spoolRadiusAtZeroTurns = .225 	# [m]
spoolRadiusAtMaxTurns = .195   	# [m]
maxSpoolTurns = 50		# [rad]


# Force Transformation, V -> KG, KG -> V
# -----------------------------------------
# 	These parameters determine how a reference tension F_t' is translated from [kg] to [V] and vice versa.
# 	The mapping will change when the force sensor changes. These parameters have been obtained through calibration.
#
#	see forceSensorCalibration.ods
# -----------------------------------------

# For a conversion from KG to V, a linear mapping is used,
polyA2 = 1.7			# [V]	
polyA1 = 0.03  			# [V/kg]  			
# with a minimum tension being always maintained,
minTensionV_base = 2  	# [V] 
minTensionV_extra = .15 # [V]
# where the maintained minimum tension is varying depending on the armAngle, phi.


# Adaptive gains Kp_in & Kp_out of Feedback-Controller
# ------------------------------------------------------------------------------------------
# 	The gains are a 2D-functions of inclination, alpha, and armAngle, phi. They decay with varying phi and increasing alpha.
#
#	see plot_2Dgains.m, 2D_GainScheduling.ods
# ------------------------------------------------------------------------------------------
#loadingGainParameter
loadingA0 = 4.5 
loadingA90 = 2  
loadingC = -0.035

#unloadingGainParameter
unloadingA0 = 3.5 
unloadingA90 = 1  
unloadingC = -0.035

# Turn the gainAdaptation OFF to set the gains Kp_in & Kp_out manually
gainAdaptation = 1	# 1=ON, 0=OFF
#Kp_in = 4 	# Set this variable, if gainAdaptation is OFF
#Kp_out = 2.5 	# Set this variable, if gainAdaptation is OFF




###################################################################################################
########################################    Controller Mode     ###################################
###################################################################################################
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

# Turn the setpointAdaptation OFF to set F_t' for the Feedback-Controller manually
setpointAdaptation = 1		# 1=ON, 0=OFF
#referenceTensionV = 2 		# [V] Set this variable, if setpointAdaptation is Off



##############################################################
####################### System Parameters #################### 
##############################################################

showPrintCommands = 1 # 1=ON, 0=OFF

inclineListLength = 20 	# number of inclination values to store
armGravityAngleListLength = 20	# number of armGravityAngle values to store
velocityListLength = 5 # number of velocity values to store

angularVelocity2motorCommand = 150/(14 * np.pi) # [omegaSpoolMax ~pi/2 => 5V ]
roverMassKG = 92.5 	# [kg]
linearSpeedMax = 0.135 	# [m/s]

#########################################################################
####################### Initalize global variables  ##################### 
#########################################################################

currentInclinationRadAvg = 0					# [rad] Initalize inclination: this is required if IMU does not return measurements.
currentInclinationDegAvg = np.degrees(currentInclinationRadAvg)	# [deg] Initalize inclination in degrees, as well
inclineList = list()

currentArmGravityAngleRadAvg = np.pi 					# [rad] Initalize armGravityAngle: this is required if IMU does not return measurements.
currentArmGravityAngleDegAvg = np.degrees(currentArmGravityAngleRadAvg)	# [deg] Initalize armGravityAngle in degrees, as well
armGravityAngleList = list()

robotLinearVelocityAvg = 0	# [m/s] Initalize velocity
velocityList = list()

############### side functions #######################


def convertKG2V(referenceTensionKG): # Convert from KG2V to generate F_ref in V
        referenceTensionV = referenceTensionKG * polyA1 + polyA2
        return referenceTensionV

def convertV2KG(referenceTensionV): # Convert from V2KG to generate F_ref in kg
        referenceTensionKG = np.divide((referenceTensionV-polyA2), polyA1)
        return referenceTensionKG

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

# Explanation: The ArmGravityAngle, beta, is the angle between the robot arm and the projection of the gravity vector in the 
# xy-plane of the robot's body frame.
# IMPORTANT NOTE: beta is differently defined then in the paper
# For ArmGravityAngle = 180, robot drives straight down-hill
# For ArmGravityAngle = 90 or 270, robot drives parallel to the slope
# For ArmGravityAngle = 0, the robots drives straight up-hill
def getArmGravityAngle(armGravityAngleReading):
	global currentArmGravityAngleRad, currentArmGravityAngleDeg
	currentArmGravityAngleDeg = armGravityAngleReading.armGravityAngleDeg
	currentArmGravityAngleRad = np.radians(currentArmGravityAngleDeg)	


# A filter function, calculating the average over the past n values.
# Inclination is in [-90,90] with no jumps.
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
# ArmGravityAngle is in [0,360] with possible jumps between 0 and 360.
def updateArmGravityAngle():
	global armGravityAngleList, currentArmGravityAngleRadAvg, currentArmGravityAngleDegAvg
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


# If the inplane force, F_r, is larger than the traction, F_f, TReX is likely to slip.
# If F_r is smaller F_f, there is a margin in the tether tension, F_t, where TReX does not slip.
# This is equation (5) and (6) plugged into (7) from the paper, solved for mu.
# IMPORTANT NOTE: Abbreviating signs result from different definitions of beta in the paper.
#
#	see plot_referenceTension.m
#
#----------------------------------------------------------------
def evalSlipCondition(referenceTensionKG): 	
	
	slipConditionA = np.square(np.tan(currentInclinationRadAvg)) 
	slipConditionB = 2* np.cos(currentArmGravityAngleRadAvg) * np.abs(np.tan(currentInclinationRadAvg)) * np.divide(1,np.cos(currentInclinationRadAvg)) * np.divide(referenceTensionKG,roverMassKG)  
	slipConditionC = np.square(np.divide(referenceTensionKG,roverMassKG)) * np.square(np.divide(1,np.cos(currentInclinationRadAvg))) 
	slipCondition = np.sqrt( slipConditionA + slipConditionB +  slipConditionC )
	
	return slipCondition


# IMPORTANT NOTE: This is just a tweak which is not described in the paper. I am not even sure, if it has any benefits.
# It is definitely no longer required when the force sensor works better.
# Explanation: The minimum tension scales with the armAngle, phi. The robot is able to withstand a higher
# tension on flat ground without slipping when the arm is pulling towards the side. This adaptation is done, to 
# prevent dead-zones when moving in small radius around a pole and phi is stuck at +-90 deg.
def updateMinTension():
	global minTensionV, minTensionKG

	minTensionV = minTensionV_base + minTensionV_extra * np.abs(np.sin(armAngleRad))
	minTensionKG = convertV2KG(minTensionV)


# Obtains the friction parameter, mu_friction, from the parameter server. 
# If parameter is not available, the standard coefficient is used and a warning message returned.
# The friction coefficient determines the reachable area / influences the available margin for F_t' / is the main tuning parameter
def updateFrictionCoefficient():
	global mu_friction
	
	if rospy.has_param('mu_friction'):
    		mu_friction = rospy.get_param('mu_friction')
	else:
		mu_friction = init_mu_friction
		rospy.logwarn('Parameter, mu_friction, not available. Using standard value: [%f] ' %init_mu_friction )


######################################################
############### main function ########################
######################################################

def listener(emptyMessage):  # listens for empty joy message
	global Kp_in, Kp_out, referenceTensionV

	print('\n\n')	
	
	###### Obtain friction coefficient, mu #######
	updateFrictionCoefficient()
	 
	###### Inclination available? / if yes: filter it ######	
	updateInclination()

	###### ArmGravityAngle available? / if yes: filter it ########	
	updateArmGravityAngle()

	###### Velocity available? / if yes: filter it #######
	updateVelocity()

	###### The minimum tension is a function of the armAngle, phi #####
	updateMinTension()

	###### Computing reference tension, F_t', either automatically  as described in the paper (setpointAdaptation = 1) or manually (setpointAdaptation = 0)
	# -----						
	# -----		   tether tension, F_t, minimizing in-plane force, F_r
	# -----		      _____________________^_________________
	# -----		     /		   		     	     \
	# -----	      F_t' = F_g*sin(inclination)*cos(armGravityAngle) 
	# -----		     + [ cos(armAngle) * directionOfMotion ] * F_g * [sqrt(mu_friction^2*cos(inclination)^2 - sin(inclination)^2*sin(armGravityAngle)^2)]
	# -----		       \__________________ ________________/   \____________________________________________ ___________________________________________/
	# -----					  v							            v
	# -----			        scaling term, lambda_F  	  		max. available margin without violating slip-condition			
 	# -----											
	# ----------------------------------------------------------------------------------------------------------------------------------------------------------------------
	if setpointAdaptation == 1:
		
		##### Generating F_t' which minimizes F_r ########

		referenceTensionKG_base = (-1) *roverMassKG * np.abs(np.sin(currentInclinationRadAvg)) *  np.cos(currentArmGravityAngleRadAvg)

		##### Evaluate slip-condition for F_t' which minimizes F_r ######	

		slipCondition_base = evalSlipCondition(referenceTensionKG_base)

		if slipCondition_base <= mu_friction:
			margin_available = 1
		else:
			margin_available = 0

		##### Compute scaling term, lambda_F  #######

		directionOfMotion = - np.divide(robotLinearVelocityAvg,linearSpeedMax)  # in [-1,1] where -1: fullspeed backwards and +1: fullspeed forwards
		lambda_F = np.cos(armAngleRad) * directionOfMotion # Depending on motion w.r.t last intermediate anchor point, F_t' is scaled

		##### If margin available, we in-/decrease F_t' according to second term in Eqn. (9) in paper ######
	
		if margin_available:
			activeForceKG = roverMassKG * lambda_F * np.sqrt(np.square(mu_friction)*np.square(np.cos(currentInclinationRadAvg)) - np.square(np.sin(currentInclinationRadAvg))*np.square(np.sin(currentArmGravityAngleRadAvg)))
		else:
			activeForceKG = 0

		########### Generating F_t' including the exploited margin while maintaining a minimum tension (maintaining minimum tension is not described in the paper)          
		
		referenceTensionKG = max(referenceTensionKG_base + activeForceKG,minTensionKG)
		
		########### Evaluate slip-condition for new F_t' which is scaled and guarantees a minimium tension ############	

		slipCondition = evalSlipCondition(referenceTensionKG)
		slipConditionMargin = mu_friction - slipCondition #How much margin is still left?

		if slipCondition>mu_friction:
			print('SlipCondition violated: Save movement of TReX cannot be guaranteed!')
			rospy.loginfo('SlipCondition violated: Save movement of TReX cannot be guaranteed!')


	if setpointAdaptation == 0: 

		########### F_t' is manually set #########
		referenceTensionKG = convertV2KG(referenceTensionV)	
		
		########### Evaluate slip-condition for F_t' ##############		
		slipCondition = evalSlipCondition(referenceTensionKG)

		if slipCondition>mu_friction :
			print('SlipCondition violated: Save movement of TReX cannot be guaranteed!')
			rospy.loginfo('SlipCondition violated: Save movement of TReX cannot be guaranteed!')

		print('setpointAdaptation = OFF : Using Manual Reference Tension [V]: \t [%f]\n' % referenceTensionV)
		rospy.loginfo('setpointAdaptation = OFF : Using Manual Reference Tension [V]: \t [%f]\n' % referenceTensionV)



	######## Transform forces from KG to V ########
	
	if setpointAdaptation == 1:
		activeForceV = convertKG2V(activeForceKG) - polyA2 #subtract polyA2 to center it around 0 and not polyA2
		referenceTensionV_base = convertKG2V(referenceTensionKG_base)
	
	referenceTensionV = convertKG2V(referenceTensionKG)




	############## Feedback: P-controller tracks F_t' #################

	tensionErrorV = referenceTensionV - tensionCurrentV 	# current error in V
	tensionErrorKG = referenceTensionKG - tensionCurrentKG	# current error in KG

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

	angularVelocitySpool = (-1) * robotLinearVelocityAvg * np.cos(armAngleRad) / spoolRadius # tetherLength [m], joyLinearVelocity [m/s], the sign is changed due to sensor definition
		
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
	controlStateMessage = controlState_new() # messages to publish

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
	controlStateMessage.slipConditionMargin = slipConditionMargin
	controlStateMessage.measuredTensionV = tensionCurrentV
	controlStateMessage.measuredTensionKG = tensionCurrentKG
	controlStateMessage.referenceTensionV = referenceTensionV
	controlStateMessage.referenceTensionKG = referenceTensionKG
	if setpointAdaptation == 1:
		controlStateMessage.activeForceV = activeForceV
		controlStateMessage.activeForceKG = activeForceKG
		controlStateMessage.minTensionV = minTensionV
		controlStateMessage.minTensionKG = minTensionKG
		controlStateMessage.referenceTensionV_base = referenceTensionV_base
		controlStateMessage.referenceTensionV_base = referenceTensionV_base
		controlStateMessage.lambda_F = lambda_F
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
		
		print('Inclination, alpha, [Deg]: \t[%f]\n' % currentInclinationDegAvg)
		print('ArmGravityAngle, beta, [Deg]: \t[%f]\n' % currentArmGravityAngleDegAvg)
		
		# If slipConditionMargin < 0 -> no safe movement can be guaranteed
		print('SlipCondition: \t\t\t[%f]' % slipCondition)
		print('SlipConditionMargin: \t\t[%f]\n' % slipConditionMargin)
		
		print('Measured Tension [V]: \t\t[%f]' % tensionCurrentV)
		print('Measured Tension [KG]: \t\t[%f]\n' % tensionCurrentKG)
		
		if setpointAdaptation == 1:

			print('Minimum Tension [V]: \t\t[%f]' % minTensionV)
			print('Minimum Tension [KG]: \t\t[%f]\n' % minTensionKG)

			print('Base Tension [V]: \t\t[%f]' % referenceTensionV_base)
			print('Base Tension [KG]: \t\t[%f]\n' % referenceTensionKG_base)
		
			print('Direction of Motion: \t\t[%f]' %directionOfMotion)
			print('Scaling Term, lambda_F: \t[%f]\n' %lambda_F)

			print('Active Force [V]: \t\t[%f]' %activeForceV)
			print('Active Force [KG]: \t\t[%f]\n' %activeForceKG)

		if setpointAdaptation == 0:
			print('Reference Tension:  \t\t[Manually Set]\n')
		elif referenceTensionKG == minTensionKG:
			print('Reference Tension: \t\t[Minimum Tension]\n')
		elif referenceTensionKG == referenceTensionKG_base:
			print('Reference Tension: \t\t[Base Tension]\n')
		elif referenceTensionKG == referenceTensionKG_base + activeForceKG and activeForceKG != 0:
			print('Reference Tension:  \t\t[Base Tension + Active Force]\n')
		else:
			print('Reference Tension: WHAT IS WRONG?')

		print('Reference Tension [V]:\t \t[%f]' % referenceTensionV)
		print('Reference Tension [KG]:\t \t[%f]\n' % referenceTensionKG)

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
	controlStatePub = rospy.Publisher('/motorControlInterface/controlState', controlState_new, queue_size=10)
	rospy.init_node('tensionControl', anonymous=True)
	rospy.set_param('mu_friction', init_mu_friction)
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
