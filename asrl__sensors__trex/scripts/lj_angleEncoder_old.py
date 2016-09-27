#!/usr/bin/env python

#Zettlex Angle Encoder (uses SPI communication protocal)

import rospy
from std_msgs.msg import String
from labjack import ljm
from asrl__sensors__trex.msg import *
from asrl__sensors__trex.srv import *
from asrl__control__tether_management.msg import *
from asrl__drivers__labjack.msg import *
from asrl__drivers__labjack.srv import *
import numpy

# SET VARIABLES
angleScale = 0.0054932 # 0.0054932 deg/bit //  65,536 counts/rev
angleOffset = 225 # offset of arm from vehicle zero
bytePosition = 31
numBits = 1
syncMask = 0xFFFFFFFFFFFF >> 48-numBits
zeroPointDefaultFlagMask = 0x000100000000
postionMask = 0x000080000000
degCalibrated = 0
sensorValue = 0
syncFlag = 0
positionValid = 0
zeroPointDefaultFlag = 0
pitchAngle = 0
pitchCalibrated = 0

def requestAngleFromLabJack():
	rospy.wait_for_service('labjack_interface_node/read_SPI')
	try:
		readSPIFunction = rospy.ServiceProxy('labjack_interface_node/read_SPI', readSPI)
		request = readSPIRequest() #try ReadSPIRequest() also
		request.MISO_PORT = 0 # Data A Output (SPI)
		request.MOSI_PORT = 21 # Data B Not Connected
		request.CLKA_PORT = 1 # Clock A	
		request.CLKB_PORT = 1000 # Clock B > 2.5V (DAC)	
		request.CHIP_PORT = 22 # Chipset Not Connected
		request.LOGIC_LEVEL = 2.5 # 2.5V Logic Level (1/2 the operating voltage)
		request.SPI_FREQ = 65497 # Frequency = 1*10^9 / (175*(65536-SpeedThrottle) + 1020) / valid value 1:65536
		request.SPI_MODE = 1 #Bit 1 = CPOL, Bit 0 = CPHA. 0 = 0/0 = b00, 1 = 0/1 = b01, 2 = 1/0 = b10, 3 = 1/1 = b11
		request.SPI_OPT = 0 # Option for CS [DOES NOT CHANGE]
		request.SPI_GO = 1 # Starts the SPI communication [DOES NOT CHANGE]
		request.NUM_BYTES = 6 # Number of Bytes in Message
		request.TX_MSG = [0, 0, 0, 0, 0, 0] # initialize TX with dummy "zero" message
		response = readSPIFunction(request)
		dataRead = map(int, response.byteRead)
		dataRead = ''.join('{:02x}'.format(x) for x in dataRead)
		syncFlag = int(dataRead,16) >> bytePosition-1;
		syncFlag = syncFlag & syncMask;
		positionValid = int(dataRead,16) & postionMask
		positionValid = positionValid >> 31
		zeroPointDefaultFlag = int(dataRead,16) & zeroPointDefaultFlagMask
		zeroPointDefaultFlag = zeroPointDefaultFlag >> 32
		sensorValue = int(dataRead,16) & 0x00001FFFFFFF
		sensorValue = sensorValue >> 9
		degree = -1*((sensorValue * angleScale) - angleOffset) #multipled by -1 in order to correct direction in urdf
		if degree < 0.0:    
			degCalibrated = 360.0 + degree
		else:
			degCalibrated = degree
		#print(degCalibrated)
		return (degCalibrated, sensorValue, syncFlag, positionValid, zeroPointDefaultFlag) 
	except rospy.ServiceException, e:
		print('Service Call Failed: [%s]' % e)

if __name__ == '__main__':
		
	#ROS PARAMS
	anglePub = rospy.Publisher('/tether_angle', angleEncoder, queue_size=10)
	rospy.init_node('angleEncoderPub', anonymous=True)
	r = rospy.Rate(10) # 10hz

	while not rospy.is_shutdown():
		try:
			data = requestAngleFromLabJack()
			angleMsg = angleEncoder()
			angleMsg.Deg = data[0] # angle
			angleMsg.Raw = data[1] # sensorValue
			angleMsg.Sync = data[2] # syncFlag
			angleMsg.Valid = data[3] # positionValid
			angleMsg.Zero = data[4] # zeroPointDefaultFlag
			angleMsg.header.stamp = rospy.Time.now()
			anglePub.publish(angleMsg)
			r.sleep()
			#print('angle: [%0.2f]' % (data[0]))
		except rospy.ROSInterruptException: pass



