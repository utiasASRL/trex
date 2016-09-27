#!/usr/bin/env python

# OMEGA LOAD CELL

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

KgFMax = 227.0 # Sensor Max Force in KgF (Imperial = 500 lbs)
Vin = 12.0 # Voltage applied to Sensor
mVOut = Vin*2.167 # Sensor Output is 2mV/V
mVIn = 30.0 # Amplifier mV In
VOut = 10.0 # Amplifier V Out
VpKg = (mVOut/KgFMax)*(VOut/mVIn) # volts/KgF
lbsCon = 2.2046 # convert kg to lbs factor
angleFactor = 1.414 # trig relation of angled rope over pulley in radian

# 2nd degree poly y=Ax^2+Bx+C
polyA = 9.798E-5
polyB = 0.021
polyC = 1.602

### NEW CODE : kcu
logging = TRexLabJackLogger("/tmp/trexLJLogs/lj_forceCell/lj_forceCell_msgFreq_log.txt")
start_date_time = datetime.now()
numMsgCt = 0

def requestForceFromLabJack():
	global logging, start_date_time, numMsgCt
	rospy.wait_for_service('labjack_interface_node/read_names')
	try:
		readNamesFunction = rospy.ServiceProxy('labjack_interface_node/read_names', readNames)
		request = readNamesRequest()
		request.names = ['AIN0'] # Force Sensor
		request.numFrames = 1 # Force Sensor

		response = (readNamesFunction(request))

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

		Vout = response.data[0]
		#forceFactoryCalibrated = ((Vout/VpKg)/angleFactor)
		kgFcalculated = abs(np.divide(np.subtract(np.sqrt(-4*polyA*polyC+4*polyA*Vout+np.square(polyB)),polyB),(2*polyA)) ) # KgF
		return (response.data[0], kgFcalculated)
	except rospy.ServiceException, e:
		print('Service Call Failed: [%s]' % e)
	except TypeError as exc:
		print "Exception caught a Type Error : ", str(exc)
		print "A likely reason is that : ticks = response.data[0], failed. The lj interface or lj interface callers might be down."


### NEW CODE : kcu
def intermediateNodeResponse(response):
	global logging, start_date_time, numMsgCt
	try:
		t = rospy.Time.now()
		secs = t.to_sec()

		current_date = datetime.now()
		check = current_date - start_date_time  # gives you a timedelta
		numMsgCt += 1
		if check >= timedelta(seconds=1):
			strResMsgFreq = str("Time : ") + str(start_date_time) + str(", Num of Msgs : ") + str(
				numMsgCt) + ", response.data : " + str(response.data)
			logging.info(strResMsgFreq)
			if globalConfigData.print_time:
				print str("Time : "), str(start_date_time), str(", Num of Msgs : "), str(numMsgCt), ", response.data : ", response.data
			start_date_time = datetime.now()
			numMsgCt = 0

		ticks = response.data[0]
		Vout = response.data[0]
		# forceFactoryCalibrated = ((Vout/VpKg)/angleFactor)
		kgFcalculated = abs(np.divide(np.subtract(np.sqrt(-4 * polyA * polyC + 4 * polyA * Vout + np.square(polyB)), polyB),
									  (2 * polyA)))  # KgF

		forceMsg = forceCell()
		forceMsg.Vout = response.data[0]
		forceMsg.KgForce = kgFcalculated
		forceMsg.header.stamp = rospy.Time.now()
		forcePub.publish(forceMsg)
	except TypeError as exc:
		print "Exception caught a Type Error : ", str(exc)
		print "A likely reason is that : ticks = response.data[0], failed. The lj interface or lj interface callers might be down."
	except rospy.ROSException as exc:
		print "Exception caught a ROSException : ", str(exc)
		
if __name__ == '__main__':
	logging.info("lj_forceCell started")  ### NEW CODE : kcu

	#ROS PARAMS
	forcePub = rospy.Publisher('/tether_tension', forceCell, queue_size=10)
	rospy.init_node('forceCellPub', anonymous=True)

	########### NEW CODE CHANGES
	if globalConfigData.use_new_code_version:
		try:
			print "NEw code Version"
			# Registar this subscriber w/ intermediate
			rospy.wait_for_service(globalConfigData.to_register_topic_name_split_1)
			registerFunction = rospy.ServiceProxy(
				globalConfigData.to_register_topic_name_split_1, registerLJSensor)
			request = registerLJSensorRequest()
			request.sensor_id = 'AIN0'
			request.num_of_frames = 1
			request.topic_name = globalConfigData.force_lji_caller_sub
			request.first_registration = True
			response = registerFunction(request)
			# Setup subscriber for parsing data
			intermediateRes = rospy.Subscriber(
				globalConfigData.force_lji_caller_sub, readResponse, intermediateNodeResponse)
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
				data = requestForceFromLabJack()
				forceMsg = forceCell()
				forceMsg.Vout = data[0]
				forceMsg.KgForce = data[1]
				forceMsg.header.stamp = rospy.Time.now()
				forcePub.publish(forceMsg)
				r.sleep()
				#print('tension: [%0.2f]KgF' % (data[2]))
			except rospy.ROSInterruptException: pass
			except TypeError as exc:
				print "Exception caught a Type Error : ", str(exc)
				print "A likely reason is that : ticks = response.data[0], failed. The lj interface or lj interface callers might be down."
			except KeyboardInterrupt:
				print "Keyboard Interrupt"