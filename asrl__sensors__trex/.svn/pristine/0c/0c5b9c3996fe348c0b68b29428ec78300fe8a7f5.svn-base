import rospy
from labjack import ljm
from asrl__sensors__trex.msg import *
from asrl__control__tether_management.msg import *
from asrl__drivers__labjack.msg import *
from asrl__drivers__labjack.srv import *
import numpy as np
from datetime import datetime, timedelta

# Ignore this Class
#     It causes a memory leak. Keeping it here for reference
class LabjackMessageFreqMonitor:

    def __init__(self):
        self.freqContainer = []
        self.step = timedelta(seconds=1)
        self.prevLength = 0

    def setStartTime(self):
        self.start_date =  datetime.now()

    def setEndTime(self):
        self.end_date = datetime.now()


    def addTime(self, time):
        now = datetime.now()
        d = now.date()
        t = now.time()
        a = {}
        a['time_step'] = (datetime(d.year, d.month, d.day, t.hour, t.minute, t.second))
        self.freqContainer.append(a)

    def dateRange(self, start, end, step):
        d = start
        while d < end: # right-open interval
            yield d
            d += step

    def printFreqContainer(self):
        for i in self.freqContainer:
            print i

    def getResults(self):
        times = np.fromiter((d['time_step'] for d in self.freqContainer),
                     dtype='datetime64[us]', count=len(self.freqContainer))

        bins = np.fromiter(self.dateRange(self.start_date, self.end_date +
                                          self.step, self.step),
                            dtype='datetime64[us]')

        a, bins = np.histogram(times, bins)
        print "RESULTS : "
        R = (dict(zip(bins[a.nonzero()].tolist(), a[a.nonzero()])))
        for r in sorted(R):
            print str("Time : "), r, str(", Num of Msgs : "), R[r]

    def getLastResultStr(self):
        times = np.fromiter((d['time_step'] for d in self.freqContainer),
                            dtype='datetime64[us]', count=len(self.freqContainer))

        bins = np.fromiter(self.dateRange(self.start_date, self.end_date +
                                      self.step, self.step),
                       dtype='datetime64[us]')
        a, bins = np.histogram(times, bins)

        R = (dict(zip(bins[a.nonzero()].tolist(), a[a.nonzero()])))
        #sortedR = sorted(R)
        #r = sortedR[-1]
        #strRes = str("Time : ") + str(r) +  str(", Num of Msgs : ") + str(R[r])

        #r = sortedR[len(R)-1]
        #print str("Time : "), r, str(", Num of Msgs : "), R[r]


        if (len(R) == (self.prevLength)) :
            return

        #print ("//////////////////")
        #print "Lnegth : ", len(R)
        self.prevLength = len(R)
        count = 0

        key = sorted(R)[len(R) - 2]
        value = R[key]
        #print "CHECK HERE ", str("Time : "), key, str(", Num of Msgs : "), value


        return str("Time : ") + str(key) + str(", Num of Msgs : ") + str(value)

        #for r in sorted(R):
        #    #print "Count " , count
        #    if (count == (len(R) - 2)): #Last one is small
        #        #print str("Time : "), r, str(", Num of Msgs : "), R[r]
        #        return str("Time : ") + str(r) + str(", Num of Msgs : ") + str(R[r])
        #    count +=1
        return ""
