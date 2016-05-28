#include "error_handling.h"
#include "rover.h"
// #include "robot_controller_c.h"

#include <chrono>
#include <future>
#include <thread>

#include <iostream>
#include <fstream>

#include <cstring>

#include <boost/asio.hpp>
#include <boost/range/algorithm/for_each.hpp>

int main(int nArgs, char* aczArgs[]) {
	// TODO: Setup boost::asio TCP server to send map bitmaps?

	if(nArgs<3) {
		std::cout << "Syntax: robot <ttyDevice> <logfile>\n";
		return 1;
	}
	
	boost::asio::io_service io_service;
	std::cout << "Opening " << aczArgs[1] << "\n";
	boost::asio::serial_port serial(io_service, aczArgs[1]);
	serial.set_option(boost::asio::serial_port::baud_rate(230400));
	
	auto SendCommand = [&](SRobotCommand const& cmd) {
		VERIFYEQUAL(boost::asio::write(serial, boost::asio::buffer(&cmd, sizeof(decltype(cmd)))), sizeof(decltype(cmd)));
	};
	
	std::cout << "Opening " << aczArgs[2] << "\n";
	std::basic_ofstream<char> ofsLog(aczArgs[2], std::ios_base::binary | std::ios_base::out | std::ios_base::trunc);
	VERIFY(ofsLog.good());
	
	std::cout << "Resetting Controller\n";
	SendCommand(SRobotCommand::reset()); // throws boost::system:::system_error
    std::this_thread::sleep_for(1s);
	std::cout << "Connecting to Controller\n";
	SerialWrite(SRobotCommand::connect()); // throws boost::system:::system_error
	
	std::atomic<bool> bRunning{true};
	
	// Command loop
	auto f = std::async([&](){
		while(true) {
			switch(std::cin.get()) {
				case 'w': SerialWrite(SRobotCommand::forward()); break;
				case 'a': SerialWrite(SRobotCommand::left_turn()); break;
				case 's': SerialWrite(SRobotCommand::backward()); break;
				case 'd': SerialWrite(SRobotCommand::right_turn()); break;
				case 'x': SerialWrite(SRobotCommand::stop()); bRunning = false; return;
			}	
		}		
	});
	
	// Sensor loop
	auto start = std::chrono::system_clock::now();
	while(bRunning) {
		SSensorData data;
		VERIFYEQUAL(boost::asio::read(serial, boost::asio::buffer(&data, sizeof(SSensorData))), sizeof(SSensorData)); // throws boost::system::system_error
		
		auto end = std::chrono::system_clock::now();
		std::chrono::duration<double> diff = end-start;

		ofsLog << diff.count() << ": " 
			<< data.m_nYaw << " "
			<< data.m_nAngle << " "
			<< data.m_nDistance;
			
		boost::for_each(data.m_anEncoderTicks, [&](short nEncoderTick) {
			ofsLog << " " << nEncoderTick; 
		});
		ofsLog << '\n';
	}
	return 0;
}