# LabJack T7 Interface Sensor Buffer
import rospy
from labjack import ljm
from asrl__sensors__trex.msg import *
from asrl__control__tether_management.msg import *
from asrl__drivers__labjack.msg import *
from asrl__drivers__labjack.srv import *

class LabjackSensorBuffer:

    def __init__(self):
        print("LabjackSensorBuffer() INIT!")
        self.send_buffer = {}

    def getAndIncSeqNum(self, name):
        self.incrementSeqNum(name)
        return self.send_buffer[name]

    # Helper for getSeqNum
    def incrementSeqNum(self, name):
        if name not in self.send_buffer:
            self.send_buffer[name] = 0
        else:
            self.send_buffer[name] += 1

    def check(self, name, SEQ_NUM):
        if name not in self.send_buffer:
            self.send_buffer[name] = 0
            if (SEQ_NUM != 0):
                print ("INITIAL SEQ NUM IS NOT 0!")
                return False

        # print ("NAME IS : " + name)
        # print ("SEQ NUM IN MAP IS : " + str(self.send_buffer[name]))
        if ((self.send_buffer[name] + 1) == SEQ_NUM):
            self.send_buffer[name] = SEQ_NUM
            return True
        elif ((self.send_buffer[name] + 1) > SEQ_NUM):
            print ("REQUESTED SEQ NUM IS TOO LOW")
        else:
            print ("REQUESTED SEQ NUM IS TOO LOW")
        return False