#include "error_handling.h"
#include "rover.h"
// #include "robot_controller_c.h"

#include <chrono>
#include <thread>

#include <iostream>
#include <fstream>

#include <cstring>

#include <boost/asio.hpp>
#include <boost/range/algorithm/for_each.hpp>

boost::asio::io_service g_io_service;
void run_io_service() {
	while(true) {
		try {
			boost::asio::io_service::work work(g_io_service);
			g_io_service.run();
		} catch(...) {
			ASSERT(false);
		}
	}
}

int main(int nArgs, char* aczArgs[]) {
	// TODO: Setup boost::asio TCP server to send map bitmaps?

	if(nArgs<3) {
		std::cout << "Syntax: robot <ttyDevice> <logfile>\n";
		return 1;
	}
	
	std::thread threadIO(run_io_service);

	std::cout << "Opening " << aczArgs[1] << "\n";
	boost::asio::serial_port serial(g_io_service, aczArgs[1]);
	serial.set_option(boost::asio::serial_port::baud_rate(230400));
	
	auto SerialWrite = [&](auto cmd) {
		VERIFYEQUAL(boost::asio::write(serial, boost::asio::buffer(&cmd, sizeof(decltype(cmd)))), sizeof(decltype(cmd)));
	};
	
	std::basic_ofstream<char> ofsLog(aczArgs[2], std::ios_base::binary | std::ios_base::out | std::ios_base::trunc);
	VERIFY(ofsLog.good());
	
	std::cout << "Waiting for Arduino\n";

	bool bSentReset = false;
	while(true) {
		char chHandshake;
		VERIFYEQUAL(boost::asio::read(serial, boost::asio::buffer(&chHandshake, sizeof(chHandshake))), sizeof(chHandshake)); // throws boost::system::system_error
				
		if(chHandshake==g_chHandshake) {
			std::cout << "Received Handshake\n";
			break;
		} else if(!bSentReset) {
			std::cout << "Send Reset\n";
			bSentReset = true;
			
			SerialWrite(SRobotCommand::reset()); // throws boost::system:::system_error
		}
	}
	
	std::cout << "\n Send Handshake\n";
	SerialWrite(g_chHandshake); // throws boost::system:::system_error
	
	auto start = std::chrono::system_clock::now();
	while(true) {
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
		
		switch(std::cin.get()) {
			case 'w': SerialWrite(SRobotCommand::forward()); break;
			case 'a': SerialWrite(SRobotCommand::left_turn()); break;
			case 's': SerialWrite(SRobotCommand::backward()); break;
			case 'd': SerialWrite(SRobotCommand::right_turn()); break;
			case 'x': SerialWrite(SRobotCommand::stop()); goto done;
		}
	}
	
done:
	g_io_service.stop();
	threadIO.join();
	return 0;
}