#!/usr/bin/env python

# LabJack T7 Interface Buffer
import rospy
from labjack import ljm
from asrl__sensors__trex.msg import *
from asrl__control__tether_management.msg import *
from asrl__drivers__labjack.msg import *
from asrl__drivers__labjack.srv import *

import time

# READ BUFFER LABJACK
# I.e. eWriteAddress, eReadAddress
#   These functions return data (can be seen in form of ticks) which get processed by each sensor.
#
#                       <----       sensor_1 request call (readRequest.name)
#                           -> return (response)
# labjack_interface     <----       sensor_2 request call (readRequest.name)
#                           -> return (response)
#                       <----       sensor_3 request call (readRequest.name)
#                           -> return (response)
#                       ....
#
#   Issue : some of the return responses might be missed....
#
#   sensor_1.py
#       request_SENSORTYPE_FromLabJack
#          response = readNamesFunction(request)
#           This response might not be the next consecutive response?
#
#   sensor_2.py ...

##########
# Code Change :

# SENSOR
# Global Variables :
#       send_buffer = [ MSG_1 = {SEQ_ID, readRequest.name}, MSG_2 = {SEQ_ID, readRequest.name} ]
# Operations :
#   | <---- sensor_1 request call
#   |           A = {REQ_SEQ_ID, readRequest.name}
#   | ----> return (SEQ_ID, ACK, response)
#            if response.seq_id != send_buffer[0].seq_id
#               WARN : SKIPPED MESSAGE
#               ACTION : ?

# labjack_interface
# Global Variables :
#       recv_buffer = { MSG_ID_1 : SEQ_NUM, MSG_ID_2 : SEQ_NUM, ...}
#  ->   |    dataReadNames(readMsg = {SEQ_ID, readRequest}) or  dataReadAddresses(...) or dataReadSPI(...)
#       |       msg_id_recv = readMsg.readRequest.names[0]
#       |       if recv_buffer[msg_id_recv] == (readMsg.SEQ_ID + 1)
#       |           OKAY
#       |       else
#       |           WARNING : SEQ_ID DO NOT MATCH, SOME MISSED MESSAGE!
#       |           return response = {SEQ_NUM+1, ACK = FALSE, data)

#############
# Testing
#
# labjack_interface.py ...read() functions to randomly drop messages
# sensor_[n].py        ...request() functions to randomly drop messages
#
########################################################################################################


class LabjackBuffer:

    def __init__(self):
        print("LabjackBuffer() INIT!")
        self.recv_buffer = {}
        self.tabConterMap = {};
        self.tabCounter = 0

    def check(self, name, SEQ_NUM):
        if name not in self.recv_buffer:
            self.recv_buffer[name] = 0
            self.tabConterMap[name] = self.tabCounter
            self.tabCounter += 8
            if (SEQ_NUM != 0):
                print ("INITIAL SEQ NUM IS NOT 0!")
                return False

        #print ("NAME IS : " + name)
        #print ("SEQ NUM IN MAP IS : " + str(self.recv_buffer[name]))
        if ((self.recv_buffer[name] + 1) == SEQ_NUM):
            self.recv_buffer[name] = SEQ_NUM
            return True
        elif ((self.recv_buffer[name] + 1) > SEQ_NUM):
            print ("REQUESTED SEQ NUM IS TOO LOW")
            time.sleep(10)
        else:
            print ("REQUESTED SEQ NUM IS TOO LOW")
            time.sleep(10)
        return False

