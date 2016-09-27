#!/usr/bin/env python

import errno
import os
import time
import datetime
from datetime import datetime

class TRexLabJackLogger:

    def __init__(self, path):
        print("WE ARE IN TRexLabJackLogger CONSTRUCTOR!")

        directory = os.path.dirname(path)
        if not os.path.exists(directory):
            print "DIR IS : ", directory
            os.makedirs(directory)

        self.logfile = open(path, 'w')
        self.clock_start = time.time()

    def __del__(self):
        self.logfile.close()

    def info(self, message):
        self.logfile.write("INFO " + str(datetime.now()) + " : " + message + "\n")
        self.logfile.flush()

    def debug(self, message):
        self.logfile.write("DEBUG " + str(datetime.now()) + " : " + message + "\n")
        self.logfile.flush()

    def error(self, message):
        self.logfile.write("ERROR " + str(datetime.now()) + " : " + message + "\n")
        self.logfile.flush()

    def getCurrentTimeStr(self):
        return str(datetime.datetime.fromtimestamp(self.clock_start))
