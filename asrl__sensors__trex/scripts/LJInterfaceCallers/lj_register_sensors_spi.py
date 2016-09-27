#!/usr/bin/env python

import rospy
from labjack import ljm
from asrl__sensors__trex.msg import *
from asrl__sensors__trex.srv import *
from asrl__control__tether_management.msg import *
from asrl__drivers__labjack.msg import *
from asrl__drivers__labjack.srv import *

import sys
#sys.path.append("..")
import sys
import os.path
sys.path.append(
    os.path.abspath(os.path.join(os.path.dirname(__file__), os.path.pardir)))
from utils.lj_messageFreq import *
from utils.lj_sensor_buffer import *
import utils.lj_logger as lj_logger
from utils.lj_logger import *
import utils.globalConfigData as globalConfigData

logging = TRexLabJackLogger("/tmp/trexLJLogs/lj_registar/lj_registar_log.txt")

class ReadNamesContainer:
    def __init__(self, id , num_of_frames):
        self.sensorID = id
        self.numOfFrames = num_of_frames

class ReadSPIContainer:
    def __init__(self, id, num_of_frames, MISO_PORT, MOSI_PORT, CLKA_PORT, CLKB_PORT,
                 CHIP_PORT, LOGIC_LEVEL, SPI_FREQ, SPI_MODE, SPI_OPT, SPI_GO, NUM_BYTES,
                 TX_MSG):
        self.sensorID = id
        self.numOfFrames = num_of_frames
        self.MISO_PORT = MISO_PORT
        self.MOSI_PORT = MOSI_PORT
        self.CLKA_PORT = CLKA_PORT
        self.CLKB_PORT = CLKB_PORT
        self.CHIP_PORT = CHIP_PORT
        self.LOGIC_LEVEL = LOGIC_LEVEL
        self.SPI_FREQ = SPI_FREQ
        self.SPI_MODE = SPI_MODE
        self.SPI_OPT = SPI_OPT
        self.SPI_GO = SPI_GO
        self.NUM_BYTES = NUM_BYTES
        self.TX_MSG = TX_MSG


class LJSensorRegistrar:
    def __init__(self):

        # Register for SPI...
        registerSPIService = rospy.Service('/register_lj_spi_sensor', registerLJSPISensor,
            self.registerSPISensor)
        # SPI Sensors
        self.registeredSPISensors = {}
        self.sensorSPIPublishers = {}
        logging.info("Register_lj_sensor subscriber is listening")
        self.unregister_time_interval = globalConfigData.loop_hz * globalConfigData.lji_callers_unregister_mag
        self.unregister_time_interval_count = 0

    # Register SPI Sensors
    def registerSPISensor(self, registrationRequest):
            print "WE ARE IN registerSPISensor()"
            sensorID = registrationRequest.sensor_id
            numOfFrames = registrationRequest.num_of_frames
            topic = registrationRequest.topic_name
            print "RECV TN: ", topic
            # if first_registration, setup service

            self.registeredSPISensors[sensorID] = [
                ReadSPIContainer(
                    sensorID, numOfFrames,
                    registrationRequest.MISO_PORT,
                    registrationRequest.MOSI_PORT,
                    registrationRequest.CLKA_PORT,
                    registrationRequest.CLKB_PORT,
                    registrationRequest.CHIP_PORT,
                    registrationRequest.LOGIC_LEVEL,
                    registrationRequest.SPI_FREQ,
                    registrationRequest.SPI_MODE,
                    registrationRequest.SPI_OPT,
                    registrationRequest.SPI_GO,
                    registrationRequest.NUM_BYTES,
                    registrationRequest.TX_MSG),
                topic]
            if registrationRequest.first_registration:
                sensorPub = rospy.Publisher(topic, spiResponse, queue_size=10);
                self.sensorSPIPublishers[sensorID] = sensorPub
            response = registerLJSPISensorResponse()
            return response

    # Unregister Sensors
    def unregister(self, sensorID):
         del self.registeredSensors[sensorID]
         del self.sensorPublishers[sensorID]

    def callLJSPI(self):
        print "WE ARE IN CALLLJ"

        framesCountSPI = 0
        namesSPI = []
        namesPrevLen = 0

        r = rospy.Rate(globalConfigData.loop_hz)
        while not rospy.is_shutdown():
            try:
                # CALL LJ_INTERFACE w/ all sensor names
                if len(self.registeredSPISensors) != namesPrevLen:
                    # Regular Sensors
                    namesPrevLen = len(self.registeredSPISensors)
                    framesCountSPI = 0
                    namesSPI = []
                    print "Recheck map, pub to send..., new len is : ", namesPrevLen
                    # SPI Sensors
                    for key, value in self.registeredSPISensors.iteritems():
                        #print "LOOP?"
                        namesSPI.append(key)
                        mapV = value
                        readSPICont = mapV[0]
                        topic = mapV[1]
                        numOfF = readSPICont.numOfFrames
                        framesCountSPI += numOfF
                        keySPI = key
                        # print "Key is : ", key
                        # print "framesCountSPI : ", framesCountSPI

                # SPI Sensors, request from LJ and publish back to subscriber listening.
                if len(self.sensorSPIPublishers) > 0:
                    rospy.wait_for_service('labjack_interface_node/read_SPI')
                    readNamesFunctionSPI = rospy.ServiceProxy('labjack_interface_node/read_SPI', readSPI)
                    loopCount = 0
                    for key, value in self.registeredSPISensors.iteritems():
                        #print "CALLING SPI?"
                        loopCount+=1
                        request = readSPIRequest()
                        #request.names = namesSPI
                        #request.numFrames = framesCountSPI
                        keySPI = key
                        mapQ = value
                        value = self.registeredSPISensors[keySPI]
                        readSPICont = mapQ[0]

                        request.MISO_PORT = readSPICont.MISO_PORT
                        request.MOSI_PORT = readSPICont.MOSI_PORT
                        request.CLKA_PORT = readSPICont.CLKA_PORT
                        request.CLKB_PORT = readSPICont.CLKB_PORT
                        request.CHIP_PORT = readSPICont.CHIP_PORT
                        request.LOGIC_LEVEL = readSPICont.LOGIC_LEVEL
                        request.SPI_FREQ = readSPICont.SPI_FREQ
                        request.SPI_MODE = readSPICont.SPI_MODE
                        request.SPI_OPT = readSPICont.SPI_OPT
                        request.SPI_GO = readSPICont.SPI_GO
                        request.NUM_BYTES = readSPICont.NUM_BYTES
                        request.TX_MSG = readSPICont.TX_MSG

                        response = readNamesFunctionSPI(request)
                        # TODO
                        print "response.data : ", response.byteRead
                        pub1 = self.sensorSPIPublishers[keySPI]
                        if pub1.get_num_connections() == 0:
                            self.unregister_time_interval_count+=1
                            if self.unregister_time_interval_count == self.unregister_time_interval:
                                self.unregister(keySPI)
                                self.unregister_time_interval_count = 0
                        else:
                            writeMsg = spiResponse()
                            writeMsg.byteRead = response.byteRead
                            pub1.publish(writeMsg)

                r.sleep()
            except rospy.ServiceException as exc:
                print("registerLJSensors, Service did not process request: " + str(exc))
            except rospy.exceptions.ROSException as exc:
                print("registerLJSensors, ROSException : " + str(exc))
                exit()  # OPTIONAL : this exception does not exit, so we force an exit if wanted behaviour
            except rospy.ROSInterruptException as exc:
                print("registerLJSensors, ROS Interrupt Exception: " + str(exc))
            except rospy.ROSInterruptException as exc:
                print("registerLJSensors, ROS Runtime Interrupt Exception: " + str(exc))

if __name__ == '__main__':
    try:
        rospy.init_node('registerLJSensors', anonymous=True)
        registration = LJSensorRegistrar()
        registration.callLJSPI();
    except rospy.ROSInterruptException: pass
    except rospy.ROSInitException: pass
    except TypeError as exc:
        print "Exception caught a Type Error : ", str(exc)
        print "A likely reason is that : ticks = response.data[0], failed. The lj interface or lj interface callers might be down."
    except KeyboardInterrupt:
        print "Keyboard Interrupt"
