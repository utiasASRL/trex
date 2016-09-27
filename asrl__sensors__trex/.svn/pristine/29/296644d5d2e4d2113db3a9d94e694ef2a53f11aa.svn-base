#pragma GCC diagnostic ignored "-pedantic"
#include <ros/ros.h>
#pragma GCC diagnostic pop
#include <iostream>
#include <boost/shared_ptr.hpp>
#include "std_msgs/String.h"
#include "asrl__drivers__labjack/readSPI.h"
#include "asrl__drivers__labjack/writeNames.h"
#include "asrl__sensors__trex/angleEncoder.h"
#include <chrono>
#include "sensors/angleEncoder.hpp"

int countMsgsRecv = 0;

AngleEncoder::AngleEncoder()
{
    //nh = ros::NodeHandle("~");
    lengthPub = nh.advertise< asrl__sensors__trex::angleEncoder >("/tether_angle", 10);
    ljClient = nh.serviceClient<asrl__drivers__labjack::readSPI>("/labjack_interface_node/read_SPI");
    rosLoopRate = 100;
    start_time = std::chrono::high_resolution_clock::now();
}

AngleEncoder::~AngleEncoder() {}

void AngleEncoder::encoderSettingsToLabJack() {}

void AngleEncoder::doCall()
{
    int countMsg = 0;
    ros::Rate rate(300);
    while (ros::ok())
    {
       asrl__drivers__labjack::readSPI srv;
		srv.request.MISO_PORT = 0; // Data A Output (SPI)
		srv.request.MOSI_PORT = 21; // Data B Not Connected
		srv.request.CLKA_PORT = 1; // Clock A
		srv.request.CLKB_PORT = 1000; // Clock B > 2.5V (DAC)
		srv.request.CHIP_PORT = 22; // Chipset Not Connected
		srv.request.LOGIC_LEVEL = 2.5; // 2.5V Logic Level (1/2 the operating voltage)
		srv.request.SPI_FREQ = 65497; // Frequency = 1*10^9 / (175*(65536-SpeedThrottle) + 1020) / valid value 1:65536
		srv.request.SPI_MODE = 1; //Bit 1 = CPOL, Bit 0 = CPHA. 0 = 0/0 = b00, 1 = 0/1 = b01, 2 = 1/0 = b10, 3 = 1/1 = b11
		srv.request.SPI_OPT = 0; // Option for CS [DOES NOT CHANGE]
		srv.request.SPI_GO = 1; // Starts the SPI communication [DOES NOT CHANGE]
		srv.request.NUM_BYTES = 6; // Number of Bytes in Message
		srv.request.TX_MSG = {0, 0, 0, 0, 0, 0}; // # initialize TX with dummy "zero" message

        if (ljClient.call(srv))
        {

        }
        auto current_time = std::chrono::high_resolution_clock::now();
        int secondsPassed = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time).count();
        //std::cout << "SECONDS PASSED : " << secondsPassed << std::endl;

        std::time_t end_time = std::chrono::system_clock::to_time_t(start_time);
        if (secondsPassed == 1) {
            start_time = std::chrono::high_resolution_clock::now();
            std::cout << "TIME? : " << std::ctime(&end_time) << " NUM OF MSG IS : " << countMsg << std::endl;
            countMsg = 0;
        }
        countMsg+=1;

        asrl__sensors__trex::angleEncoder angleMsg;
        angleMsg.Deg = 1;
        angleMsg.Raw = 2;
        lengthPub.publish(angleMsg);

        rate.sleep();
    }
    return;

    // THIS IS NOT WORKING!


    ros::Rate loop_rate(rosLoopRate);

    while (ros::ok())
    {
        asrl__drivers__labjack::readSPI srv;
		srv.request.MISO_PORT = 0; // Data A Output (SPI)
		srv.request.MOSI_PORT = 21; // Data B Not Connected
		srv.request.CLKA_PORT = 1; // Clock A
		srv.request.CLKB_PORT = 1000; // Clock B > 2.5V (DAC)
		srv.request.CHIP_PORT = 22; // Chipset Not Connected
		srv.request.LOGIC_LEVEL = 2.5; // 2.5V Logic Level (1/2 the operating voltage)
		srv.request.SPI_FREQ = 65497; // Frequency = 1*10^9 / (175*(65536-SpeedThrottle) + 1020) / valid value 1:65536
		srv.request.SPI_MODE = 1; //Bit 1 = CPOL, Bit 0 = CPHA. 0 = 0/0 = b00, 1 = 0/1 = b01, 2 = 1/0 = b10, 3 = 1/1 = b11
		srv.request.SPI_OPT = 0; // Option for CS [DOES NOT CHANGE]
		srv.request.SPI_GO = 1; // Starts the SPI communication [DOES NOT CHANGE]
		srv.request.NUM_BYTES = 6; // Number of Bytes in Message
		srv.request.TX_MSG = {0, 0, 0, 0, 0, 0}; // # initialize TX with dummy "zero" message

        if (ljClient.call(srv))
        {
            auto current_time = std::chrono::high_resolution_clock::now();
            int secondsPassed = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time).count();
            if (secondsPassed >= 1)
            {
                std::time_t end_time = std::chrono::system_clock::to_time_t(start_time);
                //auto secs = std::chrono::high_resolution_clock::now();
                std::cout << "TIME : " << std::ctime(&end_time) << " NUM OF MSG IS : " << countMsgsRecv << std::endl;
                start_time = std::chrono::high_resolution_clock::now();
                countMsgsRecv = 0;
            }
            else
            {
                countMsgsRecv+=1;
            }
        }
        //std::cout << "WE ARE THRE?" << std::endl;
    }

	ros::spin();
}

int main(int argc, char ** argv) {

    std::cout << "WE ARE IN Angle!" << std::endl;
	std::string userInput;

	ros::init(argc, argv, "angle_encoder_cpp_test");

    std::shared_ptr<AngleEncoder> lengthEncoder(new AngleEncoder());
    lengthEncoder->encoderSettingsToLabJack();
    lengthEncoder->doCall();

    return 1;
}
