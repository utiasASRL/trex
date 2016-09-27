#pragma GCC diagnostic ignored "-pedantic"
#include <ros/ros.h>
#pragma GCC diagnostic pop
#include <iostream>
#include <boost/shared_ptr.hpp>
#include <chrono>
#include "LJInterface.hpp"

LJInterface::LJInterface()
{
    // Subscribers
    writeNamesSubscribe = nh.subscribe("~write_names", 1000, &LJInterface::dataWriteNames, this);
    writeAddressesSubscribe = nh.subscribe("~writeAddresses", 1000, &LJInterface::dataWriteAddresses, this);

    // Services
    readNamesService = nh.advertiseService
        <asrl__drivers__labjack::readNames::Request, asrl__drivers__labjack::readNames::Response>
        ("~read_names", boost::bind(&LJInterface::dataReadNames, this, _1, _2));


    readAddressesService = nh.advertiseService
        <asrl__drivers__labjack::readAddresses::Request, asrl__drivers__labjack::readAddresses::Response>
        ("~read_addresses", boost::bind(&LJInterface::dataReadAddresses, this, _1, _2));

    readSPIService = nh.advertiseService
        <asrl__drivers__labjack::readSPI::Request, asrl__drivers__labjack::readSPI::Response>
        ("~read_names", boost::bind(&LJInterface::dataReadSPI, this, _1, _2));

    // LJ
    err = LJM_Open(LJM_dtANY, LJM_ctANY, "LJM_idANY", &handle);
}
LJInterface::~LJInterface(){}

///////////////////////////////////
// SUBSCRIBER CALLBACKS
///////////////////////////////////
void LJInterface::dataWriteNames(
    const asrl__drivers__labjack::writeNames::ConstPtr& writeRequest)
{

}

void LJInterface::dataWriteAddresses(
    const asrl__drivers__labjack::writeAddresses::ConstPtr& writeRequest)
{

}

///////////////////////////////////
// SERVICE CALLBACKS
///////////////////////////////////
bool LJInterface::dataReadNames(
    asrl__drivers__labjack::readNames::Request& readRequest,
    asrl__drivers__labjack::readNames::Response& readResponse)
{

    if (readRequest.numFrames == 1)
    {
        double value = 0;
        LJM_eReadName(handle, readRequest.names[0].c_str(), &value);
        readResponse.data.resize(1);
        readResponse.data[0] = value;
    }
    else
    {
        // TODO
        int errorAddress;
        double values[readRequest.numFrames];
        double ainValues[3];
        const char * chPtr[3];
        LJM_eReadNames(handle, 3, chPtr, ainValues, &errorAddress);
        //LJM_eReadNames(handle, readRequest.numFrames, {"AIN5", "AIN6", "AIN10"}, &values, &errorAddress);
        readResponse.data.resize(readRequest.numFrames);
        //readResponse.data =  values;

        //readResponse.data.insert(readResponse.data.end(), &values[0], &values[readRequest.numFrames]);

        //response.data = ljm.eReadNames(self.handle, readRequest.numFrames, readRequest.names)
    }

    return true;
}

bool LJInterface::dataReadAddresses(
    asrl__drivers__labjack::readAddresses::Request& readRequest,
    asrl__drivers__labjack::readAddresses::Response& readResponse)
{
    return true;
}

bool LJInterface::dataReadSPI(
    asrl__drivers__labjack::readSPI::Request& readRequest,
    asrl__drivers__labjack::readSPI::Response& readResponse)
{
    return true;
}
/**/

int main(int argc, char ** argv) {

	std::string userInput;
    std::cout << "WE ARE IN LJInterface!" << std::endl;
	//ros::init(argc, argv, "force_cell_cpp_test");

    std::shared_ptr<LJInterface> lj_interface(new LJInterface());

    return 1;
}
