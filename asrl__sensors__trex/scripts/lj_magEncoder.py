#!/usr/bin/env python

# Magnetic Encoder

import rospy
from std_msgs.msg import String
from labjack import ljm
from asrl__sensors__trex.msg import *
from asrl__sensors__trex.srv import *
from asrl__control__tether_management.msg import *
from asrl__drivers__labjack.msg import *
from asrl__drivers__labjack.srv import *

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

### NEW CODE : kcu
logging = TRexLabJackLogger("/tmp/trexLJLogs/lj_magEncoder/lj_magEncoder_msgFreq_log.txt")
start_date_time = datetime.now()
numMsgCt = 0

def requestMagVoltsFromLabJack():
	global logging, start_date_time, numMsgCt
	rospy.wait_for_service('labjack_interface_node/read_names')
	try:
		readNamesFunction = rospy.ServiceProxy('labjack_interface_node/read_names', readNames)
		request = readNamesRequest()
		request.names = ['AIN2']
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

		return (response.data[0])
	except rospy.ServiceException, e:
		print('Service Call Failed: [%s]' % e)

### NEW CODE : kcu
def intermediateNodeResponse(response):
	global secsLast, turnsLast, logging, start_date_time, numMsgCt
	try:
		current_date = datetime.now()
		check = current_date - start_date_time  # gives you a timedelta
		numMsgCt += 1
		if check >= timedelta(seconds=1):
			if globalConfigData.print_time:
				print str("Time : "), str(start_date_time), str(", Num of Msgs : "), str(numMsgCt), ", response.data : ", response.data
			strResMsgFreq = str("Time : ") + str(start_date_time) + str(", Num of Msgs : ") + str(numMsgCt) + ", response.data : " + str(response.data)
			logging.info(strResMsgFreq)
			start_date_time = datetime.now()
			numMsgCt = 0

		magMsg = magEncoder()
		magMsg.Vout = response.data[0]
		magMsg.header.stamp = rospy.Time.now()
		magEncPub.publish(magMsg)
	except TypeError as exc:
		print "Exception caught a Type Error : ", str(exc)
		print "A likely reason is that : ticks = response.data[0], failed. The lj interface or lj interface callers might be down."
	except rospy.ROSException as exc:
		print "Exception caught a ROSException : ", str(exc)

if __name__ == '__main__':
	logging.info("lj_forceCell started")  ### NEW CODE : kcu

	#ROS PARAMS
	magEncPub = rospy.Publisher('/mag_encoder', magEncoder, queue_size=100)
	rospy.init_node('magEncoderPub', anonymous=True)

	########### NEW CODE CHANGES : kcu
	if globalConfigData.use_new_code_version:
		try:
			print "New code Version"
			# Register this subscriber w/ intermediate
			rospy.wait_for_service(globalConfigData.to_register_topic_name_split_2)
			registerFunction = rospy.ServiceProxy(
				globalConfigData.to_register_topic_name_split_2, registerLJSensor)
			request = registerLJSensorRequest()
			request.sensor_id = 'AIN2'
			request.num_of_frames = 1
			request.topic_name = globalConfigData.mag_lji_caller_sub
			request.first_registration = True
			response = registerFunction(request)
			# Setup subscriber for parsing data
			intermediateRes = rospy.Subscriber(
				globalConfigData.mag_lji_caller_sub, readResponse, intermediateNodeResponse)
			rospy.spin()
		except rospy.ServiceException as exc:
			print("Service did not process request: " + str(exc))
		except rospy.exceptions.ROSException as exc:
			print("ROSException : " + str(exc))
		except rospy.ROSInterruptException: pass
	else:
		print "Old code Version"
		r = rospy.Rate(globalConfigData.loop_hz) # 100hz
		while not rospy.is_shutdown():
			try:
				data = requestMagVoltsFromLabJack()
				magMsg = magEncoder()
				magMsg.Vout = data
				magMsg.header.stamp = rospy.Time.now()
				magEncPub.publish(magMsg)
				r.sleep()
			except rospy.ROSInterruptException: pass
			except TypeError as exc:
				print "Exception caught a Type Error : ", str(exc)
				print "A likely reason is that : ticks = response.data[0], failed. The lj interface or lj interface callers might be down."
			except KeyboardInterrupt:
				print "Keyboard Interrupt"