#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from labjack import ljm
from asrl__sensors__trex.msg import *
from asrl__control__tether_management.msg import *
from asrl__drivers__labjack.msg import *
from asrl__drivers__labjack.srv import *

# kcu, new
import errno
import os
import time
import datetime
from datetime import datetime, timedelta
start_date_time = datetime.now()
numMsgCt = 0

class motorControlInterface:
	# STATE
	divisor = 5 #sets ramp to voltage, e.g., difference/divisor
	motorAddresses = [1002,2004,2005] # [voltageInput, directionInput, brakeState]
	motorFrames = 3
	motorDataTypes = [ljm.constants.FLOAT32, ljm.constants.UINT16, ljm.constants.UINT16]
	ledNames = ['EIO4','EIO5','EIO6']
	ledFrames = 3
	voltageTolerance = 0.001
	def __init__(self):
		rospy.init_node('motorControlInterface')
		#SUBSCRIBE to desired motorCommands		
		self.desiredMotorCommandSubscribe = rospy.Subscriber('~read_motorCommand', motorCommand, self.desiredCommandCallback) #('name of request', MSG dependancy, point to function in class)
		self.motorConPub = rospy.Publisher('~motorControl', motorControl, queue_size=10)
		self.motorWritePub = rospy.Publisher('labjack_interface_node/write_addresses', writeAddresses, queue_size=10)
		self.ledWritePub = rospy.Publisher('labjack_interface_node/write_names', writeNames, queue_size=10)
		#provide Service to request current motor states
		self.m_currentVoltage = 0 #member variable
		self.m_currentDirection = 0
		self.m_currentbrakeState = 0
		self.timeSinceLastCommand = rospy.get_rostime();
		self.currentCommand = motorCommand()

	def readMotorValuesfromLabjack(self):
		global start_date_time, numMsgCt
		rospy.wait_for_service('labjack_interface_node/read_addresses')
		try:
			readAddressesFunction = rospy.ServiceProxy('labjack_interface_node/read_addresses', readAddresses)
			request = readAddressesRequest() 
			request.addresses = [1002] #motorControlInterface.motorAddresses
			request.numFrames = 1 #motorControlInterface.motorFrames
			request.dataTypes = [ljm.constants.FLOAT32] #motorControlInterface.motorDataTypes
			response = readAddressesFunction(request)
			#print(response.data)
			self.m_currentVoltage = response.data[0]
			#self.m_currentDirection = response.data[1]
			#self.m_currentbrakeState = response.data[2]

			current_date = datetime.now()
			check = current_date - start_date_time  # gives you a timedelta
			numMsgCt += 1
			if check >= timedelta(seconds=1):
				print str("Time : "), str(start_date_time), str(", Num of Msgs : "), str(
					numMsgCt), ", response.data : ", response.data
				start_date_time = datetime.now()
				numMsgCt = 0

		except rospy.ServiceException, e:
			print('Service Call Failed: [%s]' % e)

	def publishMotorValues(self):
		motorConMsg = motorControl()
		motorConMsg.Vout = self.m_currentVoltage
		motorConMsg.motorDirection = self.m_currentDirection
		motorConMsg.brakeState = self.m_currentbrakeState
		motorConMsg.header.stamp = rospy.Time.now() # get current time (for float use rospy.get_time())
		self.motorConPub.publish(motorConMsg)		

	def generateDesiredControlMessage(self,desiredVolts,desiredDirection):
		motorWriteMsg = writeAddresses()
		motorWriteMsg.addresses = motorControlInterface.motorAddresses
		motorWriteMsg.numFrames = motorControlInterface.motorFrames
		motorWriteMsg.dataTypes = motorControlInterface.motorDataTypes
		motorWriteMsg.values = [0,0,0]
		motorWriteMsg.values[1] = desiredDirection
		motorWriteMsg.values[0] = self.incrementMotorSpeed(desiredVolts)
		if abs(motorWriteMsg.values[0]) < motorControlInterface.voltageTolerance:
			motorWriteMsg.values[2] = 0 #BrakeSetting.BRAKE_ON
		else:
			motorWriteMsg.values[2] = 1 #BrakeSetting.BRAKE_OFF
		self.m_currentDirection = motorWriteMsg.values[1]
		self.m_currentbrakeState = motorWriteMsg.values[2]
		return motorWriteMsg

	def desiredCommandCallback(self,command):
		print('received command')
		self.currentCommand = command
		self.timeSinceLastCommand = command.header.stamp

	def getLEDSetting(self,direction,brakeSetting):
		led = [0,0,0]
		if self.m_currentbrakeState == 0:
			led[0] = 1 # IDLE
		elif self.m_currentDirection == 0:
			led[1] = 1 # CW (direction = 0)
		else:
			led[2] = 1 # CCW (direction = 1)
		return led

	def setLED(self,led):
		ledWriteMsg = writeNames()
		ledWriteMsg.names = motorControlInterface.ledNames
		ledWriteMsg.numFrames = motorControlInterface.ledFrames
		ledWriteMsg.values = led
		self.ledWritePub.publish(ledWriteMsg)

	def controlMotor(self): 		
		desiredVolts = 0
		diff = rospy.Time.now() - self.timeSinceLastCommand
		#print('It has been %f seconds since last command \n\n' % diff.to_sec())
		if diff.to_sec() < 0.2:
			desiredVolts = self.currentCommand.desiredVoltage
		desiredDirection = self.currentCommand.motorDir
		requestMessage = self.generateDesiredControlMessage(desiredVolts,desiredDirection)
		self.motorWritePub.publish(requestMessage)
		led = self.getLEDSetting(requestMessage.values[1],requestMessage.values[2])
		self.setLED(led)

	def incrementMotorSpeed(self, motorVolts):
		#difference = motorVolts - self.m_currentVoltage
		#print(difference)
		#interval = difference / motorControlInterface.divisor
		#print(interval)
		#print(motorVolts)
		setVolts = motorVolts # self.m_currentVoltage + interval
		return setVolts

	def Run(self):
		r = rospy.Rate(100) # 10hz
		while not rospy.is_shutdown():
			try:
				self.controlMotor()
				rospy.sleep(50/1000)
				self.readMotorValuesfromLabjack()
				self.publishMotorValues()
				r.sleep()
			except rospy.ServiceException as exc:
				print("Service did not process request: " + str(exc))
			except rospy.exceptions.ROSException as exc:
				print("ROSException : " + str(exc))
			except rospy.ROSInterruptException:
				pass
#THE MAIN LOOP
if __name__ == "__main__":
	interface = motorControlInterface()
	interface.Run()
