#include "error_handling.h"
#include "rover.h"
#include "robot_configuration.h"

#include "fast_particle_slam.h"

#include <chrono>
#include <future>
#include <thread>
#include <functional>

using namespace std::chrono_literals;

#include <iostream>
#include <fstream>

#include <boost/asio.hpp>
#include <boost/algorithm/cxx11/all_of.hpp>
#include <boost/range/algorithm/for_each.hpp>

#include <boost/optional.hpp>
#include <boost/pool/object_pool.hpp>

#include "opencv2/opencv.hpp"

/*  Configures stdin, so robot can be controlled with 
	WASD keys from command line. 
	Disables automatic echoing of input characters and 
	disables "canonical" mode, which processes stdin
	input line by line. 
*/
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

enum ECalibration {
	ecalibrationUNKNOWN,
	ecalibrationINPROGRESS,
	ecalibrationWAITFORUSER,
	ecalibrationDONE
};

using FOnSensorData = std::function< boost::optional<SRobotCommand>(SSensorData const&) >;

/*
	Connection to robot using boost::asio 
	Resets and syncs with connected robot microcontroller.
	Receives sensor packets from robot microcontroller on serial port strPort, passes it on to func.
	When the robot microcontroller sends yaw values from an attached IMU, it must be calibrated first.  
	When manual robot control is enabled, processes WASD keyboard controls and sends them to microcontroller.
*/
struct SRobotConnection : SConfigureStdin {
	SRobotConnection(boost::asio::io_service& io_service, std::string const& strPort, bool bManual, FOnSensorData func)
		: m_io_service(io_service)
		, m_stdin(io_service, ::dup(STDIN_FILENO))
		, m_serial(io_service, strPort) // throws boost::system::system_error
		, m_timer(io_service, boost::posix_time::seconds(60))
		, m_funcOnSensorData(func)
		, m_bManual(bManual)
	{
		wait_for_command();
		wait_for_sensor_data();

		// Send synchronous connection request
		auto SendCommand = [&](SRobotCommand cmd) {
			VERIFYEQUAL(boost::asio::write(m_serial, boost::asio::buffer(&cmd, sizeof(decltype(cmd)))), sizeof(decltype(cmd)));
		};

		std::cout << "Resetting Controller\n";
		SendCommand(SRobotCommand::reset()); // throws boost::system:::system_error
		std::this_thread::sleep_for(1s);

		std::cout << "Connecting to Controller\n";
		SendCommand(SRobotCommand::connect()); // throws boost::system:::system_error
	}

	~SRobotConnection() {
		// The robot connection must only be destroyed once the io_service has
		// completed all requests.
		ASSERT(m_bShutdown);
	}

	void wait_for_command() {
		// Read a character of input
		boost::asio::async_read(
			m_stdin, 
			boost::asio::buffer(&m_ch, sizeof(char)),
			[&](boost::system::error_code const& ec, std::size_t length) {
				ASSERT(!ec);
				ASSERT(!m_bShutdown);

				switch(m_ch) {
					case 'q': 
						if(ecalibrationWAITFORUSER==m_ecalibration) {
							m_ecalibration=ecalibrationDONE; 
						}
						break;
					case 'x': 
						std::cout << "Shutting down.\n";
						Shutdown();
						return; // Don't wait for further commands
					default: 
						if(m_bManual) {
							switch(m_ch) {
								case 'e': send_command(SRobotCommand::forward_left()); break;
								case 'r': send_command(SRobotCommand::forward()); break;
								case 't': send_command(SRobotCommand::forward_right()); break;

								case 'd': send_command(SRobotCommand::left_turn()); break;
								case 'g': send_command(SRobotCommand::right_turn()); break;

								case 'c': send_command(SRobotCommand::backward_left()); break;
								case 'v': send_command(SRobotCommand::backward()); break;
								case 'b': send_command(SRobotCommand::backward_right()); break;
							}
						}					
				}

				wait_for_command();
			});
	}
 
	void wait_for_sensor_data() {
		m_timer.expires_from_now(boost::posix_time::seconds(60));
		m_timer.async_wait([this](boost::system::error_code const& ec) {
			if(!ec) {
				std::cout << "No command for 60s. Shutting down.\n";
				Shutdown(); // No data in 60s -> shutdown
			}
		});

		boost::asio::async_read(
			m_serial, 
			boost::asio::buffer(&m_sensordata, sizeof(SSensorData)),
			[&](boost::system::error_code const& ec, std::size_t length) {
				ASSERT(!ec);
				ASSERT(length==sizeof(SSensorData));

				// Ignore further data if we're waiting for the last reset command to be delivered
				if(m_bShutdown) return;
				m_timer.cancel();

				switch(m_ecalibration) {
					case ecalibrationUNKNOWN:
						if(m_sensordata.m_nYaw!=USHRT_MAX) {
							std::cout << "To calibrate accelerometer, move robot in an 8." << std::endl;
							m_ecalibration = ecalibrationINPROGRESS;
						} else {
							m_ecalibration = ecalibrationDONE; // No accelerometer, nothing to calibrate
						}
						break;
					case ecalibrationINPROGRESS: {
						auto tpNow = std::chrono::system_clock::now();
						std::chrono::duration<double> durDiff = tpNow-m_tpLastMessage;
						if(5.0 < durDiff.count()) {					
							std::cout << "Calibration values: " << static_cast<int>(m_sensordata.m_nCalibSystem) << ", " 
								<< static_cast<int>(m_sensordata.m_nCalibGyro) << ", " 
								<< static_cast<int>(m_sensordata.m_nCalibAccel) << ", "
								<< static_cast<int>(m_sensordata.m_nCalibMag) << std::endl;
							m_tpLastMessage = tpNow;
						}
						if(3==m_sensordata.m_nCalibSystem) {
							std::cout << "Calibration succeeded. Put robot on the floor and press q." << std::endl;
							m_ecalibration = ecalibrationWAITFORUSER;
						}
						break;
					}
					case ecalibrationWAITFORUSER: {
						break; // do nothing until user presses button
					}
					case ecalibrationDONE: {
						m_funcOnSensorData(m_sensordata);
						break;
					}
				}

				wait_for_sensor_data();
			}); 
	}

	void send_command(SRobotCommand rcmd) {
		// send_command *may* be called from another than m_io_service's thread
		// dispatch to correct thread, copying rcmd

		m_io_service.dispatch([this, rcmd] {
			auto prcmd = m_poolrcmd.construct(rcmd);
			boost::asio::async_write(
				m_serial,
				boost::asio::buffer(prcmd, sizeof(SRobotCommand)),
				[this, prcmd](boost::system::error_code const& ec, std::size_t length) {
					ASSERT(!ec);
					ASSERT(length==sizeof(SRobotCommand));
					m_poolrcmd.free(prcmd);
				});
		});
	}

private:
	void Shutdown() {
		ASSERT(!m_bShutdown);
		m_bShutdown = true;
		send_command(SRobotCommand::reset());
	}

	bool m_bShutdown = false;
	boost::asio::io_service& m_io_service;

	boost::asio::posix::stream_descriptor m_stdin;
	char m_ch;

	boost::asio::serial_port m_serial;
	boost::asio::deadline_timer m_timer;
	std::chrono::system_clock::time_point m_tpLastMessage;
	
	SSensorData m_sensordata;
	std::function< boost::optional<SRobotCommand>(SSensorData const&) > m_funcOnSensorData;

	boost::object_pool<SRobotCommand> m_poolrcmd;
	
	bool const m_bManual;
	ECalibration m_ecalibration = ecalibrationUNKNOWN;
};

int ConnectToRobot(std::string const& strPort, std::ofstream& ofsLog, bool bManual, boost::optional<std::string> const& strOutput) {
	// Establish robot connection via serial port
	try {
		// State shared with parallel thread
		std::mutex m;
		CFastParticleSlamBase pfslam;
		std::condition_variable cv;
		std::deque<SScanLine> deqscanline;
		
		std::thread t([&pfslam, &m, &cv, &deqscanline, &strOutput] {
			while(true) {	
				SScanLine scanline;
				{
					std::unique_lock<std::mutex> lk(m);
					cv.wait(lk, [&]{ return !deqscanline.empty(); });
					
					if(1<deqscanline.size()) {
						std::cout << "Warning: Scan line queue size is " << deqscanline.size() << std::endl;
					}
					
					scanline = std::move(deqscanline.front());
					deqscanline.pop_front();
				}
				
				pfslam.receivedSensorData(scanline);
				if(strOutput) {
					try {					
						cv::imwrite(strOutput.get(), pfslam.getMapWithPose());	
					} catch(std::exception const& e) {
						std::cerr << "Error writing to " << strOutput.get() << ": " << e.what() << std::endl;
					} catch(...) {
						std::cerr << "Unknown error writing to " << strOutput.get() << std::endl;
						std::abort();
					}
				}	
			}
		});
		
		boost::asio::io_service io_service;
		
		SScanLine scanline;
		auto tpStart = std::chrono::system_clock::now();
		SRobotConnection rc(io_service, strPort, bManual,
			 [&](SSensorData const& data) {
				if(ofsLog.is_open()) {
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

				if(!scanline.add(data)) {
					{
						std::unique_lock<std::mutex> lk(m);
						deqscanline.emplace_back(scanline);
						cv.notify_one();
					}
					
					scanline.clear();
					scanline.add(data);
				}
				
				// TODO: if(!bManual) return controller's command to robot
				 
				return boost::none;
			 }); // throws boost::system:::system_error
			 
		t.detach();
		io_service.run();
		
		return 0;
	} catch(boost::system::system_error const& s) {
		std::cerr << s.what();
		return 1;
	}
}
