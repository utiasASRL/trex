#!/usr/bin/env python

# QUAD ENCODER (LENGTH)

import rospy
from std_msgs.msg import String
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

# LENGTH ENCODER
PulleyRad = 0.0263 # 0.03135 # 0.02955 #0.028185 # as measured in m rope diameter (taut) 8.7mm, pulley inner diamter = 45.37mm
lengthRes = 400.0 # should be 100 x 4 pulses Quadrature Encoder = 2000
turns = 0
secsLast = 0
ticksNow = 0
turnsLast = 0

### NEW CODE : kcu
logging = TRexLabJackLogger("/tmp/trexLJLogs/lj_lengthEncoder/lj_lengthEncoder_msgFreq_log.txt")
start_date_time = datetime.now()
numMsgCt = 0

def encoderSettingsToLabJack():
	global settingsPub
	try:
		### NEW CODE : kcu
		logging.info("TODO : service : labjack_interface_node/write_names, not being called?")
		#print('waiting for service')
		#rospy.wait_for_service('labjack_interface_node/write_names')
		#try:
		#print('setting up proxy')
		#writeNamesFunction = rospy.ServiceProxy('labjack_interface_node/write_names', writeNames)
		#print('writing names')
		request = writeNames()
		#print('names read')
		request.names = ['DIO2_EF_ENABLE','DIO3_EF_ENABLE','DIO2_EF_INDEX','DIO3_EF_INDEX','DIO2_EF_ENABLE','DIO3_EF_ENABLE']
		request.numFrames = 6
		request.values = [0,0,10,10,1,1] # disable A&B Ch (0), set A&B to Quadrature (10), enable A&B Ch (1)
		settingsPub.publish(request)
		#response = writeNamesFunction(request)
		#except rospy.ServiceException, e:
		#	print('Service Call Failed: [%s]' % e)
	except rospy.ServiceException, e:
		print('Service Call Failed: [%s]' % e)

def requestLengthFromLabJack():
	global secsLast, turnsLast, logging, start_date_time, numMsgCt
	rospy.wait_for_service('labjack_interface_node/read_names')
	try:
		t = rospy.Time.now()
		secs = t.to_sec()
		readNamesFunction = rospy.ServiceProxy('labjack_interface_node/read_names', readNames)
		request = readNamesRequest()
		request.names = ['DIO2_EF_READ_A_F'] #TODO Use 'DIO2_EF_READ_A_AND_RESET'
		request.numFrames = 1

		response = readNamesFunction(request)
		ticks = response.data[0]
		#print("TICKS : " + str(ticks))

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

		turns = ticks/lengthRes #divide ticks by pulses for quadrature
		tetherRPS = ((turns - turnsLast) / (secs - secsLast))
		tetherV  = tetherRPS*2*np.pi*PulleyRad
		dist = turns*2*np.pi*PulleyRad # 2pir*rotations
		angle = abs((turns - np.floor(turns)))*2*np.pi
		turnsLast = turns
		secsLast = secs
		return (ticks, turns, dist, angle, tetherV)
	except rospy.ServiceException, e:
		print('Service Call Failed: [%s]' % e)
	except TypeError as exc:
		print "Exception caught a Type Error : ", str(exc)
		print "A likely reason is that : ticks = response.data[0], failed. The lj interface or lj interface callers might be down."

### NEW CODE : kcu
def intermediateNodeResponse(response):
	global secsLast, turnsLast, logging, start_date_time, numMsgCt
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
		#print "TICK ARE : ", ticks
		turns = ticks / lengthRes  # divide ticks by pulses for quadrature
		tetherRPS = ((turns - turnsLast) / (secs - secsLast))
		tetherV = tetherRPS * 2 * np.pi * PulleyRad
		dist = turns * 2 * np.pi * PulleyRad  # 2pir*rotations
		angle = abs((turns - np.floor(turns))) * 2 * np.pi
		turnsLast = turns
		secsLast = secs

		lengthMsg = lengthEncoder()
		lengthMsg.Ticks = ticks
		lengthMsg.Turns = turns
		lengthMsg.Meters = dist
		lengthMsg.Angle = angle
		lengthMsg.Velocity = tetherV
		lengthMsg.header.stamp = rospy.Time.now()
		lengthPub.publish(lengthMsg)
	except TypeError as exc:
		print "Exception caught a Type Error : ", str(exc)
		print "A likely reason is that : ticks = response.data[0], failed. The lj interface or lj interface callers might be down."

if __name__ == '__main__':

	logging.info("lj_lengthEncoder started")	### NEW CODE : kcu
	#ROS PARAMS
	settingsPub = rospy.Publisher('/labjack_interface_node/write_names',writeNames,queue_size=10);
	logging.info("lj_lengthEncoder publisher /labjack_interface_node/write_names started")	### NEW CODE : kcu
	lengthPub = rospy.Publisher('/tether_length', lengthEncoder, queue_size=10)
	logging.info("lj_lengthEncoder publisher /tether_length")	### NEW CODE : kcu
	rospy.init_node('lengthEncoderPub', anonymous=True)
	logging.info("lj_lengthEncoder ros node : lengthEncoderPub, initialized")  ### NEW CODE : kcu

	if globalConfigData.use_new_code_version:
		try:
			print "NEw code Version"
			# Registar this subscriber w/ intermediate
			encoderSettingsToLabJack() #set ports to quadrature
			rospy.wait_for_service(globalConfigData.to_register_topic_name_split_1)
			registerFunction = rospy.ServiceProxy(
				globalConfigData.to_register_topic_name_split_1, registerLJSensor)
			request = registerLJSensorRequest()
			request.sensor_id = 'DIO2_EF_READ_A_F'
			request.num_of_frames = 1
			request.topic_name = globalConfigData.length_lji_caller_sub
			request.first_registration = True
			response = registerFunction(request)
			# Setup subscriber for parsing data
			intermediateRes = rospy.Subscriber(
				globalConfigData.length_lji_caller_sub, readResponse, intermediateNodeResponse)
			rospy.spin()
			exit()	#Exiting
		except rospy.ServiceException as exc:
			print("Service did not process request: " + str(exc))
		except rospy.exceptions.ROSException as exc:
			print("ROSException : " + str(exc))
		except rospy.ROSInterruptException: pass
	else:
		print "Old code Version"
		r = rospy.Rate(globalConfigData.loop_hz) # 10hz
	#	print('sending settings to lab jack!\n')
		encoderSettingsToLabJack() #set ports to quadrature
	#	print('settings sent\n')
		while not rospy.is_shutdown():
			try:
		#		print('Requesting length from lab jack!\n')
				data = requestLengthFromLabJack()
				lengthMsg = lengthEncoder()
				lengthMsg.Ticks = data[0] #ticks
				lengthMsg.Turns = data[1] #turns
				lengthMsg.Meters = data[2] #dist
				lengthMsg.Angle = data[3] #angle
				lengthMsg.Velocity = data[4] #tetherVelocity
				lengthMsg.header.stamp = rospy.Time.now()
				lengthPub.publish(lengthMsg)
				r.sleep()
				#print('length: [%0.2f]m' % (data[2]))
			except rospy.ROSInterruptException: pass
			except TypeError as exc:
				print "Exception caught a Type Error : ", str(exc)
				print "A likely reason is that : ticks = response.data[0], failed. The lj interface or lj interface callers might be down."
