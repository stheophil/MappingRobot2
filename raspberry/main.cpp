#include "error_handling.h"
#include "rover.h"
// #include "robot_controller_c.h"

#include <chrono>
#include <future>
#include <thread>
using namespace std::chrono_literals;

#include <iostream>
#include <fstream>

#include <cstring>

#include <boost/asio.hpp>
#include <boost/range/algorithm/for_each.hpp>

struct SConfigureStdin {
	termios m_termOld;
	
	SConfigureStdin() {
		tcgetattr( STDIN_FILENO, &m_termOld);
    	termios termNew = m_termOld;
		
		// http://stackoverflow.com/questions/1798511/how-to-avoid-press-enter-with-any-getchar?lq=1
    	// ICANON normally takes care that one line at a time will be processed
    	// that means it will return if it sees a "\n" or an EOF or an EOL
    	termNew.c_lflag &= ~(ICANON | ECHO);
    	tcsetattr( STDIN_FILENO, TCSANOW, &termNew);
	}
    
	~SConfigureStdin() {
    	tcsetattr( STDIN_FILENO, TCSANOW, &m_termOld);	
	}
};

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
	SendCommand(SRobotCommand::connect()); // throws boost::system:::system_error
	
	std::atomic<bool> bRunning{true};

	enum ECalibration {
		ecalibrationUNKNOWN,
		ecalibrationINPROGRESS,
		ecalibrationWAITFORUSER,
		ecalibrationDONE
	};
	std::atomic<ECalibration> ecalibration{ecalibrationUNKNOWN};

	// Command loop
	auto f = std::async(std::launch::async, [&] {
		std::cout << "Waiting for commands\n";
		SConfigureStdin s;
		while(true) {
			switch(static_cast<char>(std::getchar())) {
				case 'w': SendCommand(SRobotCommand::forward()); break;
				case 'a': SendCommand(SRobotCommand::left_turn()); break;
				case 's': SendCommand(SRobotCommand::backward()); break;
				case 'd': SendCommand(SRobotCommand::right_turn()); break;
				case 'x': SendCommand(SRobotCommand::reset()); bRunning = false; return;
				case 'c': std::cout << "C!" << std::endl; if(ecalibrationWAITFORUSER==ecalibration) ecalibration=ecalibrationDONE; break;
			}
		}
	});
	
	// Sensor loop
	auto tpStart = std::chrono::system_clock::now();
	std::chrono::time_point<std::chrono::system_clock> tpLastMessage;
	while(bRunning) {
		// TODO: This needs a time out!
		SSensorData data;
		VERIFYEQUAL(boost::asio::read(serial, boost::asio::buffer(&data, sizeof(SSensorData))), sizeof(SSensorData)); // throws boost::system::system_error
		
		switch(ecalibration) {
			case ecalibrationUNKNOWN:
				if(data.m_nYaw!=USHRT_MAX) {
					std::cout << "To calibrate accelerometer, move robot in an 8." << std::endl;
					ecalibration = ecalibrationINPROGRESS;
				} else {
					ecalibration = ecalibrationDONE; // No accelerometer, nothing to calibrate
				}
				break;
			case ecalibrationINPROGRESS: {
				auto tpNow = std::chrono::system_clock::now();
				std::chrono::duration<double> durDiff = tpNow-tpLastMessage;
				if(5.0 < durDiff.count()) {					
					std::cout << "Calibration values: " << static_cast<int>(data.m_nCalibSystem) << ", " 
						<< static_cast<int>(data.m_nCalibGyro) << ", " 
						<< static_cast<int>(data.m_nCalibAccel) << ", "
						<< static_cast<int>(data.m_nCalibMag) << std::endl;
					tpLastMessage = tpNow;
				}
				if(3==data.m_nCalibSystem) {
					std::cout << "Calibration succeeded. Put robot on the floor and press c." << std::endl;
					ecalibration = ecalibrationWAITFORUSER;
				}
				break;
			}
			case ecalibrationWAITFORUSER: {
				break; // do nothing until user presses button
			}
			case ecalibrationDONE:
			{
				auto tpEnd = std::chrono::system_clock::now();
				std::chrono::duration<double> durDiff = tpEnd-tpStart;

				ofsLog << durDiff.count() << ";" 
					<< data.m_nYaw << ";"
					<< data.m_nAngle << ";"
					<< data.m_nDistance;
					
				boost::for_each(data.m_anEncoderTicks, [&](short nEncoderTick) {
					ofsLog << ";" << nEncoderTick; 
				});
				ofsLog << '\n';
			}
		}
	}
	return 0;
}
