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
#include <boost/lexical_cast.hpp>

#include <boost/optional.hpp>
#include <boost/pool/object_pool.hpp>

#include "opencv2/opencv.hpp"
#include <microhttpd.h>

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

using FOnOdometryData = std::function< void (SOdometryData const&) >;
using FOnLidarData = std::function< boost::optional<SRobotCommand>(SLidarData const&) >;

/*
	Connection to robot using boost::asio 
	Resets and syncs with connected robot microcontroller.
	Receives sensor packets from robot microcontroller on serial port strPort, passes it on to funcOdometry.
	When the robot microcontroller sends yaw values from an attached IMU, it must be calibrated first.  
	When manual robot control is enabled, processes WASD keyboard controls and sends them to microcontroller.
*/
struct SRobotConnection : SConfigureStdin {
	SRobotConnection(boost::asio::io_service& io_service, 
		std::string const& strPort, std::string const& strLidar, 
		bool bManual, 
		FOnOdometryData funcOdometry,
		FOnLidarData funcLidar
	)
		: m_io_service(io_service)
		, m_stdin(io_service, ::dup(STDIN_FILENO))
		, m_serialOdo(io_service, strPort) // throws boost::system::system_error
		, m_serialLidar(io_service, strLidar) // throws boost::system::system_error
		, m_timer(io_service, boost::posix_time::seconds(60))
		, m_funcOnOdometryData(funcOdometry)
		, m_funcOnLidarData(funcLidar)
		, m_bManual(bManual)
	{
		m_serialLidar.set_option(boost::asio::serial_port_base::baud_rate(115200));
		m_serialLidar.set_option(boost::asio::serial_port_base::character_size(8));
		m_serialLidar.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
		m_serialLidar.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
		m_serialLidar.set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));

		wait_for_command();
		wait_for_sensor_data();
		wait_for_lidar_data();

		// Send synchronous connection request
		auto SendCommand = [&](SRobotCommand cmd) {
			VERIFYEQUAL(boost::asio::write(m_serialOdo, boost::asio::buffer(&cmd, sizeof(decltype(cmd)))), sizeof(decltype(cmd)));
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
			m_serialOdo, 
			boost::asio::buffer(&m_odometry, sizeof(SOdometryData)),
			[&](boost::system::error_code const& ec, std::size_t length) {
				ASSERT(!ec);
				ASSERT(length==sizeof(SOdometryData));

				// Ignore further data if we're waiting for the last reset command to be delivered
				if(m_bShutdown) return;
				m_timer.cancel();

				m_funcOnOdometryData(m_odometry);

				wait_for_sensor_data();
			}); 
	}
	
	void wait_for_lidar_data() {
		// TODO: Separate time-out timer for lidar data?
		boost::asio::async_read(
			m_serialLidar, 
			boost::asio::buffer(&m_lidardata, sizeof(SLidarData)),
			[&](boost::system::error_code const& ec, std::size_t length) {
				ASSERT(!ec);
				ASSERT(length==sizeof(SLidarData));

				// Ignore further data if we're waiting for the last reset command to be delivered
				if(m_bShutdown) return;
				// m_timer.cancel();

				m_funcOnLidarData(m_lidardata);

				wait_for_lidar_data();
			}); 
	}

	void send_command(SRobotCommand rcmd) {
		// send_command *may* be called from another than m_io_service's thread
		// dispatch to correct thread, copying rcmd

		m_io_service.dispatch([this, rcmd] {
			auto prcmd = m_poolrcmd.construct(rcmd);
			boost::asio::async_write(
				m_serialOdo,
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

	boost::asio::serial_port m_serialOdo;
	boost::asio::serial_port m_serialLidar;
	boost::asio::deadline_timer m_timer;
	
	SOdometryData m_odometry;
	FOnOdometryData m_funcOnOdometryData;

	SLidarData m_lidardata;
	FOnLidarData m_funcOnLidarData;

	boost::object_pool<SRobotCommand> m_poolrcmd;
	
	bool const m_bManual;
};

int ConnectToRobot(std::string const& strPort, std::string const& strLidar, std::ofstream& ofsLog, bool bManual, boost::optional<std::string> const& strOutput) {
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
		
		SRobotConnection rc(io_service, strPort, strLidar, bManual,
			 [&](SOdometryData const& odom) {
				if(ofsLog.is_open()) {
					auto tpEnd = std::chrono::system_clock::now();
					std::chrono::duration<double> durDiff = tpEnd-tpStart;

					ofsLog << "o;" << durDiff.count() << ';'
						<< odom.m_nFrontLeft << ';'
						<< odom.m_nFrontRight << ';'
						<< odom.m_nBackLeft << ';'
						<< odom.m_nBackRight;
					ofsLog << '\n';
				}
				scanline.add(odom);
			 },
			 [&](SLidarData const& lidar) {
				if(ofsLog.is_open()) {
					auto tpEnd = std::chrono::system_clock::now();
					std::chrono::duration<double> durDiff = tpEnd-tpStart;

					ofsLog << "l;" << durDiff.count() << ';';
					ForEachAngleDistance(lidar, [&](int nAngle, int nDistance) {
						ofsLog << ';' << nAngle << ';' << nDistance;
						return true;
					});
					ofsLog << '\n';
				}

				if(!scanline.add(lidar)) {
					{
						std::unique_lock<std::mutex> lk(m);
						deqscanline.emplace_back(scanline);
						cv.notify_one();
					}
					
					scanline.clear();
					scanline.add(lidar);
				}

				return boost::none;
			 }			 
		); // throws boost::system:::system_error

		MHD_Daemon* pdaemon = nullptr;
		if(bManual) {
			std::cout << "Starting server on port 8088" << std::endl;
			pdaemon = MHD_start_daemon(
				MHD_USE_THREAD_PER_CONNECTION, 
				/*port*/ 8088, 
				/*accept all connections*/ nullptr, nullptr, 
				[](void* pvData, struct MHD_Connection* pconn, 
					const char* szUrl, const char* szMethod, const char* szVersion, const char* szUploadData, size_t* stUploadDataSize, 
					void** ppvConnectionData) {

					if(0 != strcmp(szMethod, MHD_HTTP_METHOD_GET)) { 
    					return MHD_NO; // unexpected method
					}

					static int s_nDummy;
  					if(&s_nDummy != *ppvConnectionData) { // do never respond on first call
      					*ppvConnectionData = &s_nDummy;
      					return MHD_YES;
    				}
  					*ppvConnectionData = nullptr;

					// Set Access-Control-Allow-Origin header for any client IP
					sockaddr* psockaddr = MHD_get_connection_info(pconn, MHD_CONNECTION_INFO_CLIENT_ADDRESS)->client_addr;
  
					constexpr int c_cbIPADDRESS = 20;
   					char strIP[c_cbIPADDRESS];
					if(psockaddr->sa_family == AF_INET) { 
						sockaddr_in *v4 = reinterpret_cast<sockaddr_in*>(psockaddr); 
						inet_ntop(AF_INET, std::addressof(v4->sin_addr), strIP, c_cbIPADDRESS); 
					} else {
						ASSERT(psockaddr->sa_family == AF_INET6); 
						sockaddr_in6 *v6 = reinterpret_cast<sockaddr_in6*>(psockaddr); 
						inet_ntop(AF_INET6, std::addressof(v6->sin6_addr), strIP, c_cbIPADDRESS);
					}
  					
					auto EmptyResponse = [&](int nCode) {
						auto* presponse = MHD_create_response_from_buffer(0, nullptr, MHD_RESPMEM_PERSISTENT);
						ASSERT(presponse);
						VERIFYEQUAL(MHD_add_response_header(presponse, "Access-Control-Allow-Origin", strIP), MHD_YES);
						VERIFYEQUAL(MHD_queue_response(pconn, nCode, presponse), MHD_YES);
						MHD_destroy_response(presponse);
						return MHD_YES;
					};

					if(0==strcmp(szUrl, "/command")) {
						const char* szLeft = MHD_lookup_connection_value(pconn, MHD_GET_ARGUMENT_KIND, "left");
						const char* szRight = MHD_lookup_connection_value(pconn, MHD_GET_ARGUMENT_KIND, "right");

						if(szLeft && szRight) {
							try {						
								auto const nLeft = boost::lexical_cast<short>(szLeft);
								auto const nRight = boost::lexical_cast<short>(szRight);

								reinterpret_cast<SRobotConnection*>(pvData)->send_command(
									{ecmdMOVE, std::min(nLeft, c_nMaxFwdSpeed), std::min(nRight, c_nMaxFwdSpeed)}
								);
								return EmptyResponse(200);
							} catch(boost::bad_lexical_cast const&) {}
						}
						return EmptyResponse(400);
					}
					return EmptyResponse(404);
				},
				std::addressof(rc),
				MHD_OPTION_END
			);
			ASSERT(pdaemon);
			std::cout << "Started http server on port 8088." << std::endl;
		}

		t.detach();
		io_service.run();
		
		if(pdaemon) MHD_stop_daemon(pdaemon);

		return 0;
	} catch(boost::system::system_error const& s) {
		std::cerr << s.what();
		return 1;
	}
}
