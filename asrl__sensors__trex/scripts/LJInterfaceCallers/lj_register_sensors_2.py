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

logging = TRexLabJackLogger("/tmp/trexLJLogs/lj_registar/" + globalConfigData.to_register_log_name_split_2)  #lj_registar_log.txt")

class ReadNamesContainer:
    def __init__(self, id , num_of_frames):
        self.sensorID = id
        self.numOfFrames = num_of_frames

class LJSensorRegistrar:
    def __init__(self):
        # Register for read
        registarService = rospy.Service(globalConfigData.to_register_topic_name_split_2, registerLJSensor, #'/register_lj_sensor', registerLJSensor,
            self.registerSensor)

        # Regular lj read call sensors
        self.registeredSensors = {}
        self.sensorPublishers = {}

        logging.info("Register_lj_sensor subscriber is listening")
        self.unregister_time_interval = globalConfigData.loop_hz * globalConfigData.lji_callers_unregister_mag
        self.unregister_time_interval_count = 0

    def registerSensor(self, registrationRequest):
        try:
            print "WE ARE IN registerSensor()"
            sensorID = registrationRequest.sensor_id
            numOfFrames = registrationRequest.num_of_frames
            topic = registrationRequest.topic_name
            print "RECV TN: ", topic
            print "sensorID TN: ", sensorID
            # if first_registration, setup service
            self.registeredSensors[sensorID] = [ReadNamesContainer(sensorID, numOfFrames), topic]
            if registrationRequest.first_registration :
                sensorPub = rospy.Publisher(topic, readResponse, queue_size=10);
                self.sensorPublishers[sensorID] = sensorPub
            response = registerLJSensorResponse()
            return response
        except rospy.ROSInterruptException: pass

    # Unregister Sensors
    def unregister(self, sensorID):
         del self.registeredSensors[sensorID]
         del self.sensorPublishers[sensorID]

    def callLJSensors(self):
        print "WE ARE IN CALLLJ"
        print "Size of Pub map : ", len(self.sensorPublishers)

        framesCount = 0
        names = []
        namesPrevLen = 0

        r = rospy.Rate(globalConfigData.loop_hz)
        while not rospy.is_shutdown():
            try:
                # CALL LJ_INTERFACE w/ all sensor names
                if len(self.registeredSensors) != namesPrevLen:
                    # Regular Sensors
                    namesPrevLen = len(self.registeredSensors)
                    framesCount = 0
                    names = []
                    print "Recheck map, pub to send..., new len is : ", namesPrevLen
                    for key, value in self.registeredSensors.iteritems():
                        # print "LOOP?"
                        # print "key is :", key
                        names.append(key)
                        mapV = value
                        readNms = mapV[0]
                        topic = mapV[1]
                        numOfF = readNms.numOfFrames
                        framesCount += numOfF
                        # print "Key is : ", key
                        # print "framesCount : ", framesCount
                    print "names : ", names
                    print "framesCount : ", framesCount

                # Regular Sensors
                if len(self.sensorPublishers) > 0:
                    rospy.wait_for_service('labjack_interface_node/read_names', timeout=10) # timeout in seconds
                    readNamesFunction = rospy.ServiceProxy('labjack_interface_node/read_names', readNames)
                    request = readNamesRequest()
                    request.names = names   # list of sensor names, sending to read as a batch
                    request.numFrames = framesCount
                    response = readNamesFunction(request)
                    #print "response.data : ", response.data

                responseIndexCounter =0
                #print len(self.sensorPublishers)
                #print self.sensorPublishers

                for key in self.sensorPublishers.keys():
                    #print "*PUB to : ", key
                    value = self.sensorPublishers[key]
                    #print "value : ", value
                        # We could have added to the registered sensors before we call the service
                    if responseIndexCounter < len(response.data):
                        pub1 = value
                        if pub1.get_num_connections() == 0:
                            self.unregister_time_interval_count+=1
                            if self.unregister_time_interval_count == self.unregister_time_interval:
                                self.unregister(key)
                                self.unregister_time_interval_count = 0
                        else:
                            self.unregister_time_interval_count = 0
                            writeMsg = readResponse()
                            writeMsg.data.append(response.data[responseIndexCounter])
                            pub1.publish(writeMsg)
                            responseIndexCounter+=1


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
        registration.callLJSensors();
    except rospy.ROSInterruptException: pass
    except rospy.ROSInitException: pass
    except TypeError as exc:
        print "Exception caught a Type Error : ", str(exc)
        print "A likely reason is that : ticks = response.data[0], failed. The lj interface or lj interface callers might be down."
    except KeyboardInterrupt:
        print "Keyboard Interrupt"
