#pragma GCC diagnostic ignored "-pedantic"
#include <ros/ros.h>
#pragma GCC diagnostic pop
#include <iostream>
#include <boost/shared_ptr.hpp>
#include "std_msgs/String.h"
#include "asrl__drivers__labjack/readNames.h"
#include "asrl__drivers__labjack/writeNames.h"
#include "asrl__sensors__trex/pitchPot.h"
#include <chrono>
#include "sensors/pitchPot.hpp"

int countMsgsRecv = 0;

PitchPot::PitchPot()
{
    //nh = ros::NodeHandle("~");
    settingsPub = nh.advertise< asrl__drivers__labjack::writeNames >("/labjack_interface_node/write_names", 10);
    ljClient = nh.serviceClient<asrl__drivers__labjack::readNames>("/labjack_interface_node/read_names");
    lengthPub = nh.advertise< asrl__sensors__trex::pitchPot >("/tether_pitch", 10);

    rosLoopRate = 100;
    start_time = std::chrono::high_resolution_clock::now();
}

PitchPot::~PitchPot() {}

void PitchPot::encoderSettingsToLabJack() {}

void PitchPot::doCall()
{
     int countMsg = 0;
    ros::Rate rate(200);
    while (ros::ok())
    {
        asrl__drivers__labjack::readNames srv;
        srv.request.names.resize(1);
        srv.request.names[0] = {"AIN1"}; //TODO Use 'DIO2_EF_READ_A_AND_RESET'
        srv.request.numFrames = 1;
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
        asrl__sensors__trex::pitchPot pitchPotMsg;
        pitchPotMsg.Vout = 1;
        lengthPub.publish(pitchPotMsg);

        rate.sleep();
    }
    return;

    // THIS IS NOT WORKING!

    ros::Rate loop_rate(rosLoopRate);

    while (ros::ok())
    {
        asrl__drivers__labjack::readNames srv;
        srv.request.names.resize(1);
        srv.request.names[0] = {"AIN1"};
        srv.request.numFrames = 1;

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
                auto ticks = srv.response.data[0];
                std::cout << "TICK IS : " << ticks << std::endl;

                // Parse return

                //turns = ticks/lengthRes #divide ticks by pulses for quadrature
                auto turns = ticks/lengthRes; //divide ticks by pulses for quadrature

                //auto tetherRPS = ((turns - turnsLast) / (secs - secsLast));

                //auto tetherV  = tetherRPS*2*np.pi*PulleyRad;

                auto dist = turns*2*M_PI*PulleyRad; // 2pir*rotations
                auto angle = abs((turns - floor(turns)))*2*M_PI;
                auto turnsLast = turns;
                //secsLast = secs



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

	std::string userInput;
    std::cout << "WE ARE IN MOTORS!" << std::endl;
	ros::init(argc, argv, "pitch_pot_encoder_cpp_test");

    std::shared_ptr<PitchPot> motorEncoder(new PitchPot());
    motorEncoder->encoderSettingsToLabJack();
    motorEncoder->doCall();
    //lengthEncoder->initialize();

    // SETTINGS Publishers
    //auto settingsPub = nh.advertise< asrl__drivers__labjack::writeNames >("/labjack_interface_node/write_names", 10);
    // Add a logger? --> easylogging
    //auto lengthPub = nh.advertise< asrl__drivers__labjack::writeNames >("/tether_length", 10);
    // Log
    // LJ Service
    //ros::ServiceClient client = nh.serviceClient<asrl__drivers__labjack::readNames>("/labjack_interface_node/read_names");
    //ros::Rate loop_rate(100);

    /*while (ros::ok())
    {
        asrl__drivers__labjack::readNames srv;
        srv.request.names.resize(1);
        srv.request.names[0] = "DIO6_EF_READ_A_F";
        srv.request.numFrames = 1;

        //srv.request.b = atoll(argv[2]);
        //std::cout << "WE ARE HERE?" << std::endl;
        if (client.call(srv))
        {
            auto current_time = std::chrono::high_resolution_clock::now();
            int secondsPassed = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time).count();
            if (secondsPassed >= 1)
            {
                std::time_t end_time = std::chrono::system_clock::to_time_t(start_time);

                std::cout << "TIME : " << std::ctime(&end_time) << " NUM OF MSG IS : " << countMsgsRecv << std::endl;
                start_time = std::chrono::high_resolution_clock::now();
                countMsgsRecv = 0;
                auto ticks = srv.response.data[0];
                std::cout << "TICK IS : " << ticks << std::endl;
            }
            else
            {
                countMsgsRecv+=1;
            }
        }
        //std::cout << "WE ARE THRE?" << std::endl;
    }

	ros::spin();*/

    return 1;
}
