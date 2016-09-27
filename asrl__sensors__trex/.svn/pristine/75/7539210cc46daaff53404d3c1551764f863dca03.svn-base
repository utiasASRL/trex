#!/usr/bin/env python

# PITCH POTENIOMETER

import rospy
from std_msgs.msg import String
from labjack import ljm
from asrl__sensors__trex.msg import *
from asrl__sensors__trex.srv import *
from asrl__control__tether_management.msg import *
from asrl__drivers__labjack.msg import *
from asrl__drivers__labjack.srv import *

### NEW CODE : kcu
from utils.lj_messageFreq import *
from utils.lj_sensor_buffer import *
import utils.lj_logger as lj_logger
from utils.lj_logger import *
import utils.globalConfigData as globalConfigData

### NEW CODE : kcu
logging = TRexLabJackLogger("/tmp/trexLJLogs/lj_pitchPot/lj_pitchPot_msgFreq_log.txt")
start_date_time = datetime.now()
numMsgCt = 0

pitchOffset = 75

# Mapping INPUT to desired OUTPUT range
def scale(val, src, dst):
	return ((val - src[0]) / (src[1]-src[0])) * (dst[1]-dst[0]) + dst[0]

def requestPitchFromLabJack():
	global logging, start_date_time, numMsgCt
	rospy.wait_for_service('labjack_interface_node/read_names')
	try:
		readNamesFunction = rospy.ServiceProxy('labjack_interface_node/read_names', readNames)
		request = readNamesRequest()
		request.names = ['AIN1'] # Force Sensor
		request.numFrames = 1 # Force Sensor

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

		degrees = scale(response.data[0], (0.0, 5.0), (0.0, 359.0))
		pitchCalibrated = degrees - pitchOffset
		return (response.data[0], pitchCalibrated)
	except rospy.ServiceException, e:
		print('Service Call Failed: [%s]' % e)

### NEW CODE : kcu
def intermediateNodeResponse(response):
	global secsLast, turnsLast, send_buffer, logging, start_date_time, numMsgCt
	try:
		t = rospy.Time.now()
		secs = t.to_sec()

		current_date = datetime.now()
		check = current_date - start_date_time  # gives you a timedelta
		numMsgCt += 1
		if check >= timedelta(seconds=1):
			strResMsgFreq = str("Time : ") + str(start_date_time) + str(", Num of Msgs : ") + str(numMsgCt) + ", response.data : " + str(response.data)
			logging.info(strResMsgFreq)
			if globalConfigData.print_time:
				print str("Time : "), str(start_date_time), str(", Num of Msgs : "), str(numMsgCt), ", response.data : ", response.data
			start_date_time = datetime.now()
			numMsgCt = 0

		degrees = scale(response.data[0], (0.0, 5.0), (0.0, 359.0))
		pitchCalibrated = degrees - pitchOffset

		pitchMsg = pitchPot()
		pitchMsg.Vout = response.data[0]  # Vout
		pitchMsg.Deg = pitchCalibrated  # pitchCalibrated
		pitchMsg.header.stamp = rospy.Time.now()
		pitchPub.publish(pitchMsg)
	except TypeError as exc:
		print "Exception caught a Type Error : ", str(exc)
		print "A likely reason is that : ticks = response.data[0], failed. The lj interface or lj interface callers might be down."
	except rospy.ROSException as exc:
		print "Exception caught a ROSException : ", str(exc)

if __name__ == '__main__':
	logging.info("lj_pitchPot started")  ### NEW CODE : kcu

	#ROS PARAMS
	pitchPub = rospy.Publisher('/tether_pitch', pitchPot, queue_size=10)
	rospy.init_node('pitchPotPub', anonymous=True)

	########### NEW CODE CHANGES
	if globalConfigData.use_new_code_version:
		try:
			print "NEw code Version"
			# Registar this subscriber w/ intermediate
			rospy.wait_for_service(globalConfigData.to_register_topic_name_split_2)
			registerFunction = rospy.ServiceProxy(globalConfigData.to_register_topic_name_split_2, registerLJSensor)
			request = registerLJSensorRequest()
			request.sensor_id = 'AIN1'
			request.num_of_frames = 1
			request.topic_name = globalConfigData.pitchPot_lji_caller_sub
			request.first_registration = True
			response = registerFunction(request)
			# Setup subscriber for parsing data
			intermediateRes = rospy.Subscriber(
				globalConfigData.pitchPot_lji_caller_sub, readResponse, intermediateNodeResponse)
			rospy.spin()
		except rospy.ServiceException as exc:
			print("Service did not process request: " + str(exc))
		except rospy.exceptions.ROSException as exc:
			print("ROSException : " + str(exc))
		except rospy.ROSInterruptException: pass
	else:
		print "Old code Version"
		r = rospy.Rate(globalConfigData.loop_hz) # 10hz
		while not rospy.is_shutdown():
			try:
				data = requestPitchFromLabJack()
				pitchMsg = pitchPot()
				pitchMsg.Vout = data[0] #Vout
				pitchMsg.Deg = data[1] #pitchCalibrated
				pitchMsg.header.stamp = rospy.Time.now()
				pitchPub.publish(pitchMsg)
				r.sleep()
				#print('tension: [%0.2f]KgF' % (data[2]))
			except rospy.ROSInterruptException: pass
			except TypeError as exc:
				print "Exception caught a Type Error : ", str(exc)
				print "A likely reason is that : ticks = response.data[0], failed. The lj interface or lj interface callers might be down."
			except KeyboardInterrupt:
				print "Keyboard Interrupt"
