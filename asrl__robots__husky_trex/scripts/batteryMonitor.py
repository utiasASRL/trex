#!/usr/bin/env python

#import roslib; roslib.load_manifest('batteryMonitor')
import rospy
import sys
from subprocess import call

from std_msgs.msg import String
from clearpath_base.msg import SystemStatus

class monitor():

  def __init__(self):
    rospy.init_node('batteryMonitor', anonymous=True)

    rospy.Subscriber("/clearpath/robots/vehicle/data/system_status", SystemStatus, self.callback)

    self.warn_limit = rospy.get_param('~warn_limit', 30) # in Volts
    self.error_limit = rospy.get_param('~error_limit', 20) # in Volts

    rospy.spin()

  def callback(self, data):
    if data.voltages[0] < self.warn_limit:
      rospy.logwarn("Volatage is low: %s V", data.voltages[0])
      #print('\a')
      if data.voltages[0] > self.error_limit:
        call(["espeak", "-v", "en", "battery warning"])
        rospy.sleep(2)
      else:
        call(["espeak", "-v", "en", "battery dying"])
        rospy.sleep(0.5)

  
if __name__ == '__main__':
  n = monitor()
