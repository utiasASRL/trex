#!/usr/bin/env python

###########################################################################
########### Max's First Simple Fuzzy-Based Tether Controller ##############
###########################################################################

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
import skfuzzy as fuzz
from skfuzzy import control as ctrl


############### Tunable Parameters ####################

# TODO make a flat and steep max

motorVoltsMax = 5 	# actuator saturation
setVolts = 2.3		# desired tether force
sensedVoltsMin = 0	


minTension = 2
maxTension = 3.2

minVel = -.1
maxVel = .1

minTorque = -5
maxTorque = 5


stepTension = (maxTension-minTension)/2
stepVel = (maxVel-minVel)/2
stepTorque = 2.5



################ initialize variables #################
voltsErrorLast = 0
setVoltsLast = 1
setScale = 0
lastDirection = 1 # reelou (slow speed)




############ Initializing Fuzzy Controller #############
#
# This section only establishes logical linkages between variables.
# TENSION, VELOCITY, TORQUE are variables based on sensor mappings.

##### Antecedent/Consequent objects hold universe variables and membership 
tension = ctrl.Antecedent(np.arange(minTension, maxTension + 0.01, stepTension), 'tension')
velocity = ctrl.Antecedent(np.arange(minVel, maxVel+.01, stepVel), 'velocity')

torque = ctrl.Consequent(np.arange(minTorque, maxTorque+.01, stepTorque), 'torque')

##### Generating membership functions

tension['toolow'] = fuzz.trimf(tension.universe, [minTension, minTension, minTension + stepTension])
tension['right'] = fuzz.trimf(tension.universe, [minTension, minTension + stepTension, maxTension])
tension['toohigh'] = fuzz.trimf(tension.universe, [minTension + stepTension, maxTension, maxTension])

velocity['drivehome'] = fuzz.trimf(velocity.universe, [minVel, minVel, minVel + stepVel])
velocity['standstill'] = fuzz.trimf(velocity.universe, [minVel, minVel + stepVel, maxVel])
velocity['driveaway'] = fuzz.trimf(velocity.universe, [minVel + stepVel, maxVel, maxVel])

torque['reelinquick'] = fuzz.trimf(torque.universe, [minTorque, minTorque, minTorque+stepTorque])
torque['reelin'] = fuzz.trimf(torque.universe, [minTorque, minTorque+stepTorque, minTorque+2*stepTorque])
torque['remain'] = fuzz.trimf(torque.universe, [minTorque+stepTorque, minTorque+2*stepTorque, minTorque+3*stepTorque])
torque['reelout'] = fuzz.trimf(torque.universe, [minTorque+2*stepTorque, minTorque+3*stepTorque, maxTorque])
torque['reeloutquick'] = fuzz.trimf(torque.universe, [minTorque+3*stepTorque, maxTorque, maxTorque])

#tension.view()
#velocity.view()
#torque.view()
#raw_input()


##### Creating tether-reeling rules
rule1 = ctrl.Rule(tension['toolow'] & velocity['drivehome'], torque['reelinquick'])
rule2 = ctrl.Rule(tension['right'] & velocity['drivehome'], torque['reelin'])
rule3 = ctrl.Rule(tension['toohigh'] & velocity['drivehome'], torque['remain'])

rule4 = ctrl.Rule(tension['toolow'] & velocity['standstill'], torque['reelin'])
rule5 = ctrl.Rule(tension['right'] & velocity['standstill'], torque['remain'])
rule6 = ctrl.Rule(tension['toohigh'] & velocity['standstill'], torque['reelout'])

rule7 = ctrl.Rule(tension['toolow'] & velocity['driveaway'], torque['remain'])
rule8 = ctrl.Rule(tension['right'] & velocity['driveaway'], torque['reelout'])
rule9 = ctrl.Rule(tension['toohigh'] & velocity['driveaway'], torque['reeloutquick'])


# Generate simulated control system
torque_ctrl = ctrl.ControlSystem([rule1, rule2, rule3,rule4,rule5,rule6,rule7,rule8,rule9])
torque_ctrl_sim = ctrl.ControlSystemSimulation(torque_ctrl)





############### main function #######################

def listener(emptyMessage): # listens for empty joy message
	global currentVolts, controlPub, tensionPub, linearVelocity, angularVelocity, armAngleRad, torque_ctrl_sim, setVolts, torque_ctrl
	
	# messages to publish
	controlMessage = motorCommand()
	tensionMessage = tensionCommand()

	####################################################
	########  FUZZY CONTROLLER ############################
	####################################################
	print('Control Setting:\t FUZZY CONTROL\n')


	if currentVolts < sensedVoltsMin :
		setVolts = sensedVoltsMin

	# tether tension: negative toolow , positive toohigh
	tensionError = currentVolts
	
	# input velocity: negative drivinghome , positive drivingaway
	# scaled with direction of movement
	actualTetherVelocity = linearVelocity * np.cos(armAngleRad)

	torque_ctrl_sim.input['tension'] = tensionError
	torque_ctrl_sim.input['velocity'] = actualTetherVelocity

	torque_ctrl_sim.compute()
	
	voltsScaler = torque_ctrl_sim.output['torque']

	print('TReX Velocity:\t\t[%f]\n' % actualTetherVelocity)
	print('Rel. Tension Error: \t[%f]\n' % currentVolts)
	
	############### send motor commands ##############

	if voltsScaler < 0 :
		controlMessage.motorDir = 0 #CW In
		controlMessage.desiredVoltage = abs(voltsScaler)
		print('Reeling-In: \t\t[%f]\n' % abs(voltsScaler))
	elif voltsScaler > 0 :
		controlMessage.motorDir = 1 # CCW Out
		controlMessage.desiredVoltage = abs(voltsScaler)
		print('Reeling-Out: \t\t[%f]\n' % abs(voltsScaler))		
	else:
		controlMessage.desiredVoltage = 0


	############## check for output limiatations ###########
	
	if controlMessage.desiredVoltage > 5 :
		print('Warning: Voltage Setpoint too high')
		controlMessage.desiredVoltage = 5


	############ generate message content #############

	controlMessage.header.stamp = rospy.Time.now()
	tensionMessage.header.stamp = rospy.Time.now()
	controlPub.publish(controlMessage)
	tensionPub.publish(tensionMessage)

	################ print statements #################
	#print('Direction (0=in,1=out): [%i]\n' % controlMessage.motorDir)
	#print('Desired Tension:\t[%f]\n'% tensionMessage.setVolts)
	#print('Current Tension:\t[%f]\n'% currentVolts)	
	#print('Tension Error:\t\t[%f]\n'% voltsError)
	#print('PID Scaler:\t\t[%f]\n' % voltsScaler)
	print('Desired Motor Volts\t[%f]\n\n' % (controlMessage.desiredVoltage))



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

