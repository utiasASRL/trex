#pragma GCC diagnostic ignored "-pedantic"
#include <ros/ros.h>
#pragma GCC diagnostic pop
#include <iostream>
#include <boost/shared_ptr.hpp>
#include "std_msgs/String.h"
#include "asrl__drivers__labjack/readNames.h"
#include <chrono>

auto start_time = std::chrono::high_resolution_clock::now();
int countMsgsRecv = 0;
/*void ljInterfaceCallback(
    const std_msgs::String::ConstPtr& msg) {
	//std::cout << "----------------------------------------------------------" << std::endl;
	auto current_time = std::chrono::high_resolution_clock::now();

    int secondsPassed = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time).count();
    //std::cout << "Program has been running for " << std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time).count()  << " seconds" << std::endl;

    if (secondsPassed >= 1)
    {
        std::time_t end_time = std::chrono::system_clock::to_time_t(start_time);

        std::cout << "TIME : " << std::ctime(&end_time) << " NUM OF MSG IS : " << countMsgsRecv << std::endl;
        start_time = std::chrono::high_resolution_clock::now();
        countMsgsRecv = 0;
    }
    else
    {
        countMsgsRecv+=1;
    }

    return;
}*/

void responseCallback(const asrl__drivers__labjack::readNamesResponse::ConstPtr& rosMsg) {
	std::cout << "We recved a response in: responseCallback" << std::endl;
	auto current_time = std::chrono::high_resolution_clock::now();
	int secondsPassed = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time).count();
	if (secondsPassed >= 1)
    {
        std::time_t end_time = std::chrono::system_clock::to_time_t(start_time);

        std::cout << "TIME : " << std::ctime(&end_time) << " NUM OF MSG IS : " << countMsgsRecv << std::endl;
        start_time = std::chrono::high_resolution_clock::now();
        countMsgsRecv = 0;
    }
    else
    {
        countMsgsRecv+=1;
    }
}

int main(int argc, char ** argv) {

	std::string userInput;

	// Setup ros subscriber on port: 22424
	// Topic: /robochunk_translator/proto_to_ros/dc1394Frame_info

	ros::init(argc, argv, "lj_interface_cpp_test");
	ros::NodeHandle nh("~");
	//ros::Subscriber dc1394FrameSub = nh.subscribe(
	//		"/forest", 1,
	//		ljInterfaceCallback);

    ros::ServiceClient client = nh.serviceClient<asrl__drivers__labjack::readNames>("/labjack_interface_node/read_names");
    ros::Rate loop_rate(100);
    while (ros::ok())
    {
        asrl__drivers__labjack::readNames srv;
        srv.request.names.resize(1);
        srv.request.names[0] = "DIO2_EF_READ_A_F";
        srv.request.numFrames = 1;

        //srv.request.b = atoll(argv[2]);
        //std::cout << "WE ARE HERE?" << std::endl;
        if (client.call(srv))
        {
            //ROS_INFO("Sum: %ld", (long int)srv.response.sum);
            //std::cout << "WE ARE THRE?" << std::endl;
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

	ros::spin();

    return 1;
}