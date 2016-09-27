#!/usr/bin/env python

# QUAD ENCODER (MOTOR)

import rospy
from std_msgs.msg import String, Empty
from labjack import ljm
from asrl__sensors__trex.msg import *
from asrl__sensors__trex.srv import *
from asrl__control__tether_management.msg import *
from asrl__drivers__labjack.msg import *
from asrl__drivers__labjack.srv import *
import numpy as np

### NEW CODE : kcu
import sys
import os.path
sys.path.append(
    os.path.abspath(os.path.join(os.path.dirname(__file__), os.path.pardir)))
from utils.lj_messageFreq import *
from utils.lj_sensor_buffer import *
import utils.lj_logger as lj_logger
from utils.lj_logger import *
import utils.globalConfigData as globalConfigData

# MOTOR ENCODER
SpoolTurns = 0
secsLast = 0
SpoolTurnsLast = 0
motorRes = 128.0 # 32 pulses Quadrature Encoder //  128
gearRatio = 307.54
#resetSpool = bool(0)
magVoltsThresh = 3 # volts to trip encoder
magVoltsLast = 0
magVolts = 0
publishHold = 0
tickOffset = 0
resetTicks = False

### NEW CODE : kcu
logging = TRexLabJackLogger("/tmp/trexLJLogs/lj_motorEncoder/lj_motorEncoder_msgFreq_log.txt")
start_date_time = datetime.now()
numMsgCt = 0

def encoderSettingsToLabJack():
	global SettingsPub
	try:
		### NEW CODE : kcu
		logging.info("TODO : service : labjack_interface_node/write_names, not being called?")

		#rospy.wait_for_service('labjack_interface_node/write_names')
		#try:
		#print("encoderSettingsToLabJack() Call")
		#writeNamesFunction = rospy.ServiceProxy('labjack_interface_node/write_names', writeNames)
		request = writeNames()
		request.names = ['DIO6_EF_ENABLE','DIO7_EF_ENABLE','DIO6_EF_INDEX','DIO7_EF_INDEX','DIO6_EF_ENABLE','DIO7_EF_ENABLE']
		request.numFrames = 6
		request.values = [0,0,10,10,1,1] # disable A&B Ch (0), set A&B to Quadrature (10), enable A&B Ch (1)
		#response = writeNamesFunction(request)
		settingsPub.publish(request)
		#except rospy.ServiceException, e:
		#print('Service Call Failed: [%s]' % e)
	except rospy.ServiceException, e:
		print('Service Call Failed: [%s]' % e)
		
#def listener(emptyMessage): # listen for empty message from joyMotorCommand to reset spool to zero postion manually
#	global resetSpool
#	resetSpool = 1

def getMagEncoder(voltsReading): # get current tension in volts
	global magVolts, magVoltsThresh, magVoltsLast, resetTicks 
	magVolts = voltsReading.Vout


	if magVolts >= magVoltsThresh and magVoltsLast < magVoltsThresh  :
		resetTicks = True
	
	magVoltsLast = magVolts

# TODO: remove this callback: not used
def requestMotorSpeedFromLabJack():
	# , resetSpool
	global SpoolTurnsLast, secsLast, magVolts, magVoltsLast, publishHold, logging, start_date_time, numMsgCt
	rospy.wait_for_service('labjack_interface_node/read_names')
	try:
		t = rospy.Time.now()
		secs = t.to_sec()
		readNamesFunction = rospy.ServiceProxy('labjack_interface_node/read_names', readNames)
		request = readNamesRequest()

		#if resetSpool == 1:
		#	request.names = ['DIO6_EF_READ_A_F_AND_RESET']
		#resetSpool = 0

		if magVolts >= magVoltsThresh and magVoltsLast < magVoltsThresh  :
			request.names = ['DIO6_EF_READ_A_F_AND_RESET']
		else:
			request.names = ['DIO6_EF_READ_A_F']

		# will set to publish message when encoder is triggered for the first time
		if magVolts < 1:
			publishHold = 1

		magVoltsLast = magVolts
		request.numFrames = 1

		response = readNamesFunction(request)

		### NEW CODE : kcu
		current_date = datetime.now()
		check = current_date - start_date_time  # gives you a timedelta
		numMsgCt += 1
		if check >= timedelta(seconds=1):
			if globalConfigData.print_time:
				print str("Time : "), str(start_date_time), str(", Num of Msgs : "), str(
					numMsgCt), ", response.data : ", response.data
			start_date_time = datetime.now()
			numMsgCt = 0

		ticks = response.data[0]
		SpoolTurns = (ticks / motorRes) / (gearRatio) # turns of spool
		RPS = ((SpoolTurns - SpoolTurnsLast) / (secs - secsLast))
		RPM = (RPS * 60)
		angle = (abs((SpoolTurns - np.floor(SpoolTurns)))*2*np.pi)
		#print('Current Spool Rotation [%f]' % angle)
		secsLast = secs
		SpoolTurnsLast = SpoolTurns
		return (ticks, SpoolTurns, RPS, RPM, angle)
	except rospy.ServiceException, e:
		print('Service Call Failed: [%s]' % e)
	except TypeError as exc:
		print "Exception caught a Type Error : ", str(exc)
		print "A likely reason is that : ticks = response.data[0], failed. The lj interface or lj interface callers might be down."


### NEW CODE : kcu
def intermediateNodeResponse(response):
	global SpoolTurnsLast, secsLast, magVolts, magVoltsLast, publishHold, logging, start_date_time, numMsgCt, tickOffset, resetTicks
	try:
		t = rospy.Time.now()
		secs = t.to_sec()

		current_date = datetime.now()
		check = current_date - start_date_time  # gives you a timedelta
		numMsgCt += 1
		if check >= timedelta(seconds=1):
			if globalConfigData.print_time:
				print str("Time : "), str(start_date_time), str(", Num of Msgs : "), str(numMsgCt), ", response.data : ", response.data
			strResMsgFreq = str("Time : ") + str(start_date_time) + str(", Num of Msgs : ") + str(
				numMsgCt) + ", response.data : " + str(response.data)
			logging.info(strResMsgFreq)
			start_date_time = datetime.now()
			numMsgCt = 0

		ticks = response.data[0]
		# New Stuff here
		if resetTicks == True:
			rospy.logwarn("Resetting ticks!")
			tickOffset = ticks
			resetTicks = False
		
		ticks = ticks - tickOffset
		# new stuff stop here

		SpoolTurns = (ticks / motorRes) / (gearRatio) # turns of spool
		RPS = ((SpoolTurns - SpoolTurnsLast) / (secs - secsLast))
		RPM = (RPS * 60)

		angle = (abs((SpoolTurns - np.floor(SpoolTurns)))*2*np.pi)
		
		
		#print('Current Spool Rotation [%f]' % angle)
		secsLast = secs
		SpoolTurnsLast = SpoolTurns

		motorEncMsg = motorEncoder()
		motorEncMsg.Ticks = ticks
		motorEncMsg.Turns = SpoolTurns
		motorEncMsg.RPS = RPS
		motorEncMsg.RPM = RPM
		motorEncMsg.Angle = angle
		motorEncMsg.header.stamp = rospy.Time.now()
		if publishHold == 1:
			motorEncPub.publish(motorEncMsg)
	except TypeError as exc:
		print "Exception caught a Type Error : ", str(exc)
		print "A likely reason is that : ticks = response.data[0], failed. The lj interface or lj interface callers might be down."
	except rospy.ROSException as exc:
		print "Exception caught a ROSException : ", str(exc)

if __name__ == '__main__':

	logging.info("lj_motorEncoder started")  ### NEW CODE : kcu

	#ROS PARAMS
	settingsPub = rospy.Publisher('/labjack_interface_node/write_names',writeNames,queue_size=10);
	motorEncPub = rospy.Publisher('/motor_encoder', motorEncoder, queue_size=10)
	#rospy.Subscriber("/reset_spool", Empty, listener)
	rospy.Subscriber("/mag_encoder", magEncoder, getMagEncoder)
	rospy.init_node('motorEncoderPub', anonymous=True)
	encoderSettingsToLabJack() #set ports to quadrature

	########### NEW CODE CHANGES
	if globalConfigData.use_new_code_version:
		try:
			print "NEw code Version"
			# Registar this subscriber w/ intermediate
			rospy.wait_for_service(globalConfigData.to_register_topic_name_split_1)
			registerFunction = rospy.ServiceProxy(
				globalConfigData.to_register_topic_name_split_1, registerLJSensor)
			request = registerLJSensorRequest()
			#if magVolts >= magVoltsThresh and magVoltsLast < magVoltsThresh:
			#	request.sensor_id = 'DIO6_EF_READ_A_F_AND_RESET'
			#else:
			request.sensor_id = 'DIO6_EF_READ_A_F'
			# will set to publish message when encoder is triggered for the first time
			if magVolts < 1:
				publishHold = 1

			magVoltsLast = magVolts
			request.num_of_frames = 1
			request.topic_name = globalConfigData.motor_lji_caller_sub
			request.first_registration = True
			response = registerFunction(request)
			# Setup subscriber for parsing data
			intermediateRes = rospy.Subscriber(
				globalConfigData.motor_lji_caller_sub, readResponse, intermediateNodeResponse)
			rospy.spin()
		except rospy.ServiceException as exc:
			print("Service did not process request: " + str(exc))
		except rospy.exceptions.ROSException as exc:
			print("ROSException : " + str(exc))
		except rospy.ROSInterruptException: pass
	else:
		print "Old code Version"
		r = rospy.Rate(globalConfigData.loop_hz)  # 10hz

		while not rospy.is_shutdown():
			try:
				data = requestMotorSpeedFromLabJack()
				motorEncMsg = motorEncoder()
				motorEncMsg.Ticks = data[0] #ticks
				motorEncMsg.Turns = data[1] #SpoolTurns
				motorEncMsg.RPS = data[2] #RPS
				motorEncMsg.RPM = data[3] #RPM
				motorEncMsg.Angle = data[4] #Angle
				motorEncMsg.header.stamp = rospy.Time.now()
				if publishHold == 1:
					motorEncPub.publish(motorEncMsg)
				r.sleep()
				#print('length: [%0.2f]m' % (data[2]))
			except rospy.ROSInterruptException: pass
			except TypeError as exc:
				print "Exception caught a Type Error : ", str(exc)
				print "A likely reason is that : ticks = response.data[0], failed. The lj interface or lj interface callers might be down."
			except KeyboardInterrupt:
				print "Keyboard Interrupt"
