#include <iostream>
#include <boost/shared_ptr.hpp>
#include "std_msgs/String.h"
#include "asrl__drivers__labjack/readNames.h"
#include "asrl__drivers__labjack/readNames.h"
#include "asrl__drivers__labjack/readAddresses.h"
#include "asrl__drivers__labjack/readSPI.h"
#include "asrl__drivers__labjack/writeNames.h"
#include "asrl__drivers__labjack/writeAddresses.h"
#include <chrono>
#include <memory>
#include <math.h>

// For the LabJackM Library
#include "LabJackM.h"

class LJInterface
{
public :
    LJInterface();
    ~LJInterface();


    void dataReadNames();
    void dataReadAddresses();
    void dataReadSPI();

    //
    void run();
    void shutdownHook();

private :
    // Member Functions
    void dataWriteNames(const asrl__drivers__labjack::writeNames::ConstPtr& writeRequest);
    void dataWriteAddresses(const asrl__drivers__labjack::writeAddresses::ConstPtr& writeRequest);
    bool dataReadNames(
        asrl__drivers__labjack::readNames::Request& readRequest,
        asrl__drivers__labjack::readNames::Response& readResponse);
    bool dataReadAddresses(
        asrl__drivers__labjack::readAddresses::Request& readRequest,
        asrl__drivers__labjack::readAddresses::Response& readResponse);
    bool dataReadSPI(
        asrl__drivers__labjack::readSPI::Request& readRequest,
        asrl__drivers__labjack::readSPI::Response& readResponse);

    ros::NodeHandle nh;

    ros::Subscriber writeNamesSubscribe;        // TOPIC : ~write_names
    ros::Subscriber writeAddressesSubscribe;    // TOPIC : ~write_addresses

    ros::ServiceServer readNamesService;        // TOPIC : ~read_names
    ros::ServiceServer readAddressesService;    // TOPIC : ~read_addresses
    ros::ServiceServer readSPIService;          // TOPIC : ~read_SPI


    int rosLoopRate;
    // Start time

    typedef std::chrono::time_point<std::chrono::high_resolution_clock> loggerClock;
    loggerClock start_time ;

    // LJ
    int handle;
    int err;
};