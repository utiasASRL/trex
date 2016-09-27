#!/usr/bin/env python

##########################################################################################

######### Max's Feedforward Steering Controller with PID Tension Feedback 2.0 ############

##########  THIS IS THE SECOND VERSION WITH PROPER FEEDBACK FORCE DEFINITION #############

##########################################################################################

############### import libraries and messages #########

import rospy
from std_msgs.msg import String, Empty
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from asrl__sensors__trex.msg import *
from asrl__control__tether_management.msg import *
from asrl__drivers__labjack.msg import *
from asrl__drivers__labjack.srv import *
import numpy as np


####################### System Parameters  ################### 

showPrintCommands = 1 # 1=ON, 0=OFF

mu_friction = .5 #Friction coefficient / determines the reachable area

spoolRadiusMax = .195 	# [m]
spoolRadiusMin = .04 	# [m]
tetherLengthMax = 42 	# [m]
angularVelocity2motorCommand = 150/(14 * np.pi) 	# [omegaSpoolMax ~pi/2 => 5V ]
roverMass = 92.5 	# [kg]
linearSpeedMax = 0.135 	# [m/s] %TODO its actually 0.135 but it actually seems that this is too high so this will now
linearSpeedThresh = 0.1 # [m/s] %TODO this was max now its scales the linear input 

currentInclinationRadAvg = 0					# [rad] Initalize inclination: this is required if IMU does not return measurements.
currentInclinationDegAvg = np.degrees(currentInclinationRadAvg)	# [deg] Initalize inclination in degrees, as well
inclineListLength = 20 	# number of inclination values to store
inclineList = list()

# Explanation: The ArmGravityAngle is the angle between the robot arm and the projection of the gravity vector in the 
# xy-plane of the robot's body frame. Depending on this angle, the desiredTension is scaled. ->see publisherNode for more detailed explanation.
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
# -------------------------------------------------------------------------------------------------------------------------------------------
# ----- F_{V,ref} = max(polyA1 * F_{kg,ref} + polyA2 , tensionRefMin)    							-------------
# -----		
# -----
# -----	Passive Mode (minimizes in-plane forces on trex):					     				-------------
# ----- F_{kg,ref} = F_g*sin(inclination)*cos(armGravityAngle)    								-------------	(eventually swap sign of the cosine)
# -----
# ----
# -----	Active Mode  (maximizes in-plane forces in the direction of movement, without inducing slip):
# -----	F_{kg,ref} = F_g*sin(inclination)*cos(armGravityAngle) 
# -----		     + F_g * cos(armAngle)*sqrt(mu_friction^2*cos(inclination)^2 - sin(inclination)^2*sin(armGravityAngle)^2)
# -----
# ----- Hint: Variable names differ slightly in the code, e.g. inclination -> currentInclinationRadAvg
# ------------------------------------------------------------------------------------------------------------------------------------------

activeFeedbackControl = 1 	# ActiveMode: 1=ON, 0=OFF

tensionRefMin = 2.1  		# [V] 	 Minimum desiredTension which is fed into Feedback-Controller

# This is the 1st-order polynom that is used to transfrom a reference tension F_ref from [kg] to [V]
polyA1 = 0.03  			# [V/kg] make sure 2V@10kg to maintain a minimal tension, 		based on InclinationTest01.ods,
polyA2 = 1.7			# [V]	 make sure 2V@10kg to maintain a minimal tension, 		based on InclinationTest01.ods,


# Turn the setpointAdaptation OFF to set the setpoint (desiredTension) of the Feedback-Controller manually
setpointAdaptation = 1		# 1=ON, 0=OFF
#desiredTension = 2 		# [V] Set this variable, if setpointAdaptation is Off


#TODO: test if feedbackControl works
#TODO: test if activeFeedbackControl works



# Adaptive gains Kp_in & Kp_out for Tension Feedback-Controller
# --------------------------------------------------------------------
#
# TODO:The gains are now 2D-functions of inclination and armGravityAngle
#
# --------------------------------------------------------------------

#loadingGainParameter
#unloadingGainParameter

# Turn the gainAdaptation OFF to set the gains Kp_in & Kp_out manually
gainAdaptation = 0	# 1=ON, 0=OFF
Kp_in = 4 	# Set this variable, if gainAdaptation is OFF
Kp_out = 2.5 	# Set this variable, if gainAdaptation is OFF







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

#TODO PM... added new callback
def getrobotLinearVelocity(cmdVelocity): # gets commanded vehicle velocity in m/s
	global robotLinearVelocity
	robotLinearVelocity = cmdVelocity.linear.x

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

# Mapping INPUT to desired OUTPUT range
def scale(val, src, dst):
	return ((val - src[0]) / (src[1]-src[0])) * (dst[1]-dst[0]) + dst[0]


############### main function #######################

def listener(emptyMessage):  # listens for empty joy message
	global Kp_in, Kp_out, desiredTension, inclineList, currentInclinationRadAvg, currentInclinationDegAvg, armGravityAngleList, currentArmGravityAngleRadAvg, currentArmGravityAngleDegAvg







#	print('\n\n\n')
#       print('desiredTension [V]: \t\t\t[%f]\n' % desiredTension)
#       print('armAngle [deg]: \t\t\t[%f]\n' % armAngleDeg)
#       print('Reeling-in gain: \t\t\t[%f]\n' % Kp_in)
#       print('Reeling-out gain: \t\t\t[%f]\n' % Kp_out)
#	print('\n\n\n')


	 
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


	##################### Check if slipCondition is violated?  #############################	

	slipCondition = np.abs(np.sin(currentArmGravityAngleRadAvg))*np.tan(currentInclinationRadAvg) - mu_friction # If >0 slip might occur / if <0 no slip

	if slipCondition>0 :
		print('SlipCondition violated: Save movement of TReX cannot be guaranteed!')
		rospy.loginfo('SlipCondition violated: Save movement of TReX cannot be guaranteed!')

	#TODO: The slipCondition does not consider when we move with minimum tension...

	#TODO: at this configureation, the force is subtraction F_active -> direction of motion 1 = forward moving.. 0 hold .. -1 backwards moving


	##################### Generating reference tension #############################

	if setpointAdaptation == 1:

		#TODO: Check if armGravityAngle has to be signFlipped
		#TODO: Check if armAngle has to be signFlipped
		
		desiredTensionKG_base= roverMass * np.sin(currentInclinationRadAvg) * (-1) * np.cos(currentArmGravityAngleRadAvg) # F_ref that minimzed the resulting foces acting on the robot.
		
		if activeFeedbackControl == 1 and slipCondition < 0: # We can only actively support the forces on the robot, if we have some margin left. -> slipCondition < 0
			directionOfMotion = 1
			activeForceKG = roverMass * np.cos(armAngleRad) * directionOfMotion * np.sqrt(np.square(mu_friction)*np.square(np.cos(currentInclinationRadAvg)) - np.square(np.sin(currentInclinationRadAvg))*np.square(np.sin(currentArmGravityAngleRadAvg)))
			#TODO: check formula
			#TODO: slipcondition for uphill movement from matlab 
		else:
			activeForceKG = 0

		desiredTensionKG = desiredTensionKG_base + activeForceKG
		desiredTensionV = convertKG2V(desiredTensionKG, polyA1, polyA2)
		desiredTension = max(desiredTensionV,tensionRefMin)
		
		print('\n\n\n')
		print('activeForceKG [kg]: \t\t[%f]\n' % activeForceKG)
		print('desiredTensionKG_base [kg]: \t\t[%f]\n' % desiredTensionKG_base)
                print('desiredTensionKG_total [kg]: \t\t[%f]\n' % desiredTensionKG)
		print('slipCondition: \t\t\t[%f]\n' % slipCondition)
		print('\n\n\n')

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
	
	
	spoolRadius = (spoolRadiusMax - (spoolRadiusMax-spoolRadiusMin)/tetherLengthMax*tetherLength)

  #TODO PM I changed this... Use the direct commanded robot velocity message
	robotLinearVelocityScaled = scale(robotLinearVelocity, (-linearSpeedMax, linearSpeedMax), (-linearSpeedThresh, linearSpeedThresh))
	angularVelocitySpool = - robotLinearVelocityScaled * np.cos(armAngleRad) / spoolRadius # tetherLength [m], joyLinearVelocity [m/s]
	
  #TODO PM I changed this... Use the joy command
	joyLinearVelocity = linearSpeedMax * joyLinearCommand
  #angularVelocitySpool = - joyLinearVelocity * np.cos(armAngleRad) / spoolRadius # tetherLength [m], joyLinearVelocity [m/s]
	
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
    #TODO PM... Changed this too
		print('Velocity Input (rover raw) [m/s]: \t[%f]\n' % robotLinearVelocity)
		print('Velocity Input (rover scaled) [m/s]: \t[%f]\n' % robotLinearVelocityScaled)
		print('Velocity Input (joy) [m/s]: \t[%f]\n' % joyLinearVelocity)
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
	rospy.Subscriber("/clearpath/robots/vehicle/cmd_vel", Twist, getrobotLinearVelocity)
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
