#!/usr/bin/env python

# LabJack T7 Interface

import rospy
from labjack import ljm
from asrl__sensors__trex.msg import *
from asrl__control__tether_management.msg import *
from asrl__drivers__labjack.msg import *
from asrl__drivers__labjack.srv import *

class LabjackInterfaceNode:
	#handle = ljm.open(ljm.constants.dtT7, ljm.constants.ctETHERNET, "192.168.0.49") # use for ETH connection
	handle = ljm.open(ljm.constants.dtANY, ljm.constants.ctANY, "ANY") # use for USB connection

	def __init__(self):
		rospy.init_node('labjack_interface_node')
		rospy.on_shutdown(self.shutdownHook)
		#self.handle = ljm.open(ljm.constants.dtT7, ljm.constants.ctETHERNET, "192.168.0.49") # use for ETH connection
		self.handle = ljm.open(ljm.constants.dtANY, ljm.constants.ctANY, "ANY") # use for USB connection
		#SUBSCRIPTIONS
		writeNamesSubscribe = rospy.Subscriber('~write_names', writeNames, self.dataWriteNames)		
		writeAddressesSubscribe = rospy.Subscriber('~write_addresses', writeAddresses, self.dataWriteAddresses)		
		#SERVICES
		readNamesService = rospy.Service('~read_names', readNames, self.dataReadNames)#('name of request', MSG dependancy, point to function in class)
		readAddressesService = rospy.Service('~read_addresses', readAddresses, self.dataReadAddresses)
		readSPIService = rospy.Service('~read_SPI', readSPI, self.dataReadSPI)

	def Run(self):
		rospy.spin()

	#SUBSCRIPTIONS
	def dataWriteNames(self,writeRequest):
		if writeRequest.numFrames == 1:
			ljm.eWriteName(self.handle, writeRequest.names[0], writeRequest.values[0])
			#print(writeRequest.numFrames)
			#print(writeRequest.names)
			#print(writeRequest.values)
		else:
			ljm.eWriteNames(self.handle, writeRequest.numFrames, writeRequest.names, writeRequest.values)
			#print(writeRequest.numFrames)
			#print(writeRequest.names)
			#print(writeRequest.values)

	def dataWriteAddresses(self,writeRequest):
		if writeRequest.numFrames == 1:
			#print('writeAddress')			
			ljm.eWriteAddress(self.handle, writeRequest.addresses[0], writeRequest.dataTypes[0], writeRequest.values[0])
		else:
			#print('writeAddress')	
			#print(writeRequest.values)		
			ljm.eWriteAddresses(self.handle, writeRequest.numFrames, writeRequest.addresses, writeRequest.dataTypes, writeRequest.values)
        
	#SERVICES
	def dataReadNames(self,readRequest):
		response = readNamesResponse()
		#print('numFrames: %i' % readRequest.numFrames)
		#print(readRequest.names)
		if readRequest.numFrames == 1:
			dataRead = ljm.eReadName(self.handle, readRequest.names[0])
			#response.data.append(dataRead)
			response.data = [0]
			response.data[0] = dataRead
		else:
			response.data = ljm.eReadNames(self.handle, readRequest.numFrames, readRequest.names)
			#response.data = dataRead
		return response

	def dataReadAddresses(self,readRequest):
		#print('reading addresses frames %i' % readRequest.numFrames)
		response = readAddressesResponse()
		if readRequest.numFrames == 1:	
			dataRead = ljm.eReadAddress(self.handle, readRequest.addresses[0], readRequest.dataTypes[0])
			response.data = [0]
			response.data[0] = dataRead			
		else:
			response.data = ljm.eReadAddresses(self.handle, readRequest.numFrames, readRequest.addresses, readRequest.dataTypes)
		return response

	def dataReadSPI(self,readRequest):
		
		#print('processing request...')
		#print('LOGIC_LEVEL %f ' % (readRequest.LOGIC_LEVEL))
		#print('CLKB_PORT %i ' % (readRequest.CLKB_PORT))
		#print('CHIP_PORT %i ' % (readRequest.CHIP_PORT))
		#print('CLKA_PORT %i ' % (readRequest.CLKA_PORT))
		#print('MISO_PORT %i ' % (readRequest.MISO_PORT))
		#print('MOSI_PORT %i ' % (readRequest.MOSI_PORT))
		#print('SPI_MODE %i ' % (readRequest.SPI_MODE))
		#print('SPI_FREQ %i ' % (readRequest.SPI_FREQ))
		#print('SPI_OPT %i ' % (readRequest.SPI_OPT))
		#print('NUM_BYTES %i ' % (readRequest.NUM_BYTES))
		#print(readRequest.TX_MSG)

		response = readSPIResponse()

		ljm.eWriteAddress(self.handle, readRequest.CLKB_PORT, ljm.constants.FLOAT32, readRequest.LOGIC_LEVEL)
		ljm.eWriteName(self.handle, "SPI_CS_DIONUM", readRequest.CHIP_PORT) 
		ljm.eWriteName(self.handle, "SPI_CLK_DIONUM", readRequest.CLKA_PORT)
		ljm.eWriteName(self.handle, "SPI_MISO_DIONUM", readRequest.MISO_PORT)
		ljm.eWriteName(self.handle, "SPI_MOSI_DIONUM", readRequest.MOSI_PORT)
		ljm.eWriteName(self.handle, "SPI_MODE", readRequest.SPI_MODE)
		ljm.eWriteName(self.handle, "SPI_SPEED_THROTTLE", readRequest.SPI_FREQ)
		ljm.eWriteName(self.handle, "SPI_OPTIONS", readRequest.SPI_OPT)
		ljm.eWriteName(self.handle, "SPI_NUM_BYTES", readRequest.NUM_BYTES)
		ljm.eWriteNameArray(self.handle, "SPI_DATA_TX",readRequest.NUM_BYTES , readRequest.TX_MSG)
		ljm.eWriteName(self.handle, "SPI_GO", readRequest.SPI_GO)
		response.byteRead = ljm.eReadNameArray(self.handle, "SPI_DATA_RX", readRequest.NUM_BYTES)
		return response

	#SHUT DOWN PROCEDURE
	def shutdownHook(self):
		print('shutting down')
		ljm.close(self.handle) # Close Labjack

#THE MAIN LOOP
if __name__ == "__main__":
	interface = LabjackInterfaceNode()
	interface.Run()

