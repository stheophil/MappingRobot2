#include "error_handling.h"
#include "rover.h"
#include "deadreckoning.h"
#include "robot_configuration.h"

#include "particle_slam.h"
#include "fast_particle_slam.h"
#include "scanmatching.h"

#include <chrono>
#include <future>
#include <thread>
#include <functional>

using namespace std::chrono_literals;

#include <iostream>
#include <fstream>

#include <cstring>

#include <boost/asio.hpp>
#include <boost/algorithm/cxx11/all_of.hpp>
#include <boost/range/algorithm/for_each.hpp>
#include <boost/program_options.hpp>
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
		auto SendCommand = [&](SRobotCommand const& cmd) {
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
	    			case 'w': 
	    				if(m_bManual) {
	    					send_command(SRobotCommand::forward()); 
	    				}
	    				break;
					case 'a': 
						if(m_bManual) {
							send_command(SRobotCommand::left_turn()); 
						}
						break;
					case 's': 
						if(m_bManual) {
							send_command(SRobotCommand::backward()); 
						}
						break;
					case 'd':
						if(m_bManual) {
							send_command(SRobotCommand::right_turn()); 
						}
						break;
					case 'c': 
						if(ecalibrationWAITFORUSER==m_ecalibration) {
							m_ecalibration=ecalibrationDONE; 
						}
						break;
					case 'x': 
						Shutdown();
						return; // Don't wait for further commands
				}

			    wait_for_command();
    		});
    }

    void wait_for_sensor_data() {
    	m_timer.async_wait([this](boost::system::error_code const& ec) {
            ASSERT(!ec);
    		Shutdown(); // No data in 60s -> shutdown
    	});

    	boost::asio::async_read(
    		m_serial, 
    		boost::asio::buffer(&m_sensordata, sizeof(SSensorData)),
    		[&](boost::system::error_code const& ec, std::size_t length) {
    			ASSERT(!ec);
    			ASSERT(length==sizeof(SSensorData));

    			// Ignore further data if we're waiting for the last reset command to be delivered
    			if(m_bShutdown) return;

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
							std::cout << "Calibration succeeded. Put robot on the floor and press c." << std::endl;
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

    void send_command(SRobotCommand const& rcmd) {
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

constexpr char c_szHELP[] = "help";
constexpr char c_szPORT[] = "port";
constexpr char c_szLOG[] = "log";
constexpr char c_szMANUAL[] = "manual";

constexpr char c_szINPUT[] = "input-file";
constexpr char c_szVIDEO[] = "video";
constexpr char c_szOUTPUT[] = "out";

int ParseLogFile(std::FILE* fp, bool bVideo, boost::optional<std::string> const& ostrOutput);

int main(int nArgs, char* aczArgs[]) {
	namespace po = boost::program_options;

	// We currently support two input modes
	// - when 'input-file' is specified, we read the saved log data and can create
	//	 a map from that log data without powering up the robot. Good for testing
	//	 algorithms.
	// - otherwise connect to microcontroller via serial port 'port'
	//	 Currently, the robot can be controlled manually using the WASD keys and 
	//	 the robot controller will send the sensor data which can be saved for
	//	 later analysis
	po::options_description optdescGeneric("Allowed options");
	optdescGeneric.add_options()
	    (c_szHELP, "Print help message")
	    (c_szPORT, po::value<std::string>()->value_name("p"), "Connect to robot on port <p>")
	    (c_szINPUT, po::value<std::string>()->value_name("file"), "Read sensor data from input file <file>");

	po::options_description optdescRobot("Robot options");
	optdescRobot.add_options()
	    (c_szLOG, po::value<std::string>()->value_name("file"), "Log all sensor data to <file>")
	    (c_szMANUAL, "Control robot manually via AWSD keys");
    
    po::options_description optdescInputFile("Input File Options");
	optdescInputFile.add_options()
	    (c_szVIDEO, "If specified, a video of path will be written instead of map image")
        (c_szOUTPUT, po::value<std::string>()->value_name("file"), "Write output to <file>");
    
    po::options_description optdesc;
    optdesc.add(optdescGeneric).add(optdescRobot).add(optdescInputFile);
    
	po::variables_map vm;
	po::store(po::parse_command_line(nArgs, aczArgs, optdesc), vm);
	po::notify(vm);    
	
	if(vm.count(c_szHELP)) {
		std::cout << optdesc << std::endl;
		return 0;
	} else if(vm.count(c_szINPUT)) {
		// Read saved sensor data from log file 
		auto const strLogFile = vm[c_szINPUT].as<std::string>();
		std::FILE* fp = std::fopen(strLogFile.c_str(), "r");
		if(!fp) {
			std::cerr << "Couldn't open " << vm[c_szINPUT].as<std::string>() << std::endl;
			return 1;
		}

        bool const bVideo = vm.count(c_szVIDEO);
        boost::optional<std::string> ostrOutput = vm.count(c_szOUTPUT)
            ? boost::make_optional(vm[c_szOUTPUT].as<std::string>())
            : boost::none;
        
        return ParseLogFile(fp, bVideo, ostrOutput);
	} else if(vm.count(c_szPORT)) {
		// Read serial port, log file name etc
		auto const strPort = vm[c_szPORT].as<std::string>();
		std::string strLog;
		if(vm.count(c_szLOG)) {
			strLog = vm[c_szLOG].as<std::string>();
		}
		bool const bManual = vm.count(c_szMANUAL);

		// TODO: Setup boost::asio TCP server to send map bitmaps?
		// Should that be an boost::asio::io_service running on a separate thread?

		// Establish robot connection via serial port
		try {
			boost::asio::io_service io_service;	

			std::basic_ofstream<char> ofsLog;
			if(!strLog.empty()) {
				ofsLog.open(strLog, std::ios_base::binary | std::ios_base::out | std::ios_base::trunc);
				VERIFY(ofsLog.good());
			}

			auto tpStart = std::chrono::system_clock::now();
			SRobotConnection rc(io_service, strPort, bManual,
				 [&](SSensorData const& data) {
				 	if(!strLog.empty()) {
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

				 	// TODO: Pass sensordata to robot controller, let robot controller queue sensor data
				 	// if necessary
				 	// TODO: if(!bManual) return controller's command to robot
                     
                    return boost::none;
				 }); // throws boost::system:::system_error
			io_service.run();
		} catch(boost::system::system_error const& s) {
			std::cerr << s.what();
			return 1;
		} 
		return 0;	
	} else {
		std::cout << "You must specify either the port to read from or an input file to parse" << std::endl;
		std::cout << optdesc << std::endl;
		return 1;
		return 1;
	}
}
