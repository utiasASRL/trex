#include <iostream>
#include <boost/shared_ptr.hpp>
#include "std_msgs/String.h"
#include "asrl__drivers__labjack/readNames.h"
#include "asrl__drivers__labjack/writeNames.h"
#include <chrono>
#include <memory>
#include <math.h>

class LengthEncoders
{
public :

    LengthEncoders();
    ~LengthEncoders();

    void encoderSettingsToLabJack();

    /*void requestLengthFromLabJack();*/

    void doCall();

private :
    ros::Publisher settingsPub;
    ros::Publisher lengthPub;

    ros::ServiceClient ljClient;

    ros::NodeHandle nh;

    int rosLoopRate;
    // Start time

    typedef std::chrono::time_point<std::chrono::high_resolution_clock> loggerClock;
    loggerClock start_time ;

    // Length Encoder
    int PulleyRad = 0.0263;  // 0.03135 # 0.02955 #0.028185 # as measured in m rope diameter (taut) 8.7mm, pulley inner diamter = 45.37mm
    int lengthRes = 400.0;   // should be 100 x 4 pulses Quadrature Encoder = 2000
    int turns = 0;
    std::time_t secsLast = 0;
    int ticksNow = 0;
    int turnsLast = 0;

};