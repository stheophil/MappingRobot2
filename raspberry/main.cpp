#include "error_handling.h"
#include "rover.h"
// #include "robot_controller_c.h"

#include <chrono>
#include <thread>

#include <iostream>
#include <cstring>

#include <unistd.h>			//Used for UART
#include <fcntl.h>			//Used for UART
#include <termios.h>		//Used for UART

struct serial_device {
	serial_device(const char* szDevice) 
		: m_fd(open(szDevice, O_RDWR | O_NOCTTY)) 
	{
		// O_NOCTTY - When set and path identifies a terminal device, 
		// 	open() shall not cause the terminal device to become the controlling terminal for the process.
		ASSERT(-1 != m_fd);
		VERIFY(0==tcgetattr(m_fd, &m_ttySaved));
		
		termios tty = m_ttySaved;
		speed_t spd = B230400;
		VERIFY(0==cfsetospeed(&tty, spd));
		VERIFY(0==cfsetispeed(&tty, spd));

		cfmakeraw(&tty); // returns void

		tty.c_cflag &= ~CSTOPB;  // send 1 stop bit
		tty.c_cflag &= ~CRTSCTS; // no HW flow control
		tty.c_cflag |= CREAD; // enable read

		VERIFY(0==tcsetattr(m_fd, TCSANOW, &tty));
	}

	template<typename TSend>
	void write(TSend const& t) noexcept {
		write(std::addressof(t), sizeof(t));
	}

	void write(const void* pvData, std::size_t cb) noexcept {
		VERIFY(cb==::write(m_fd, pvData, cb));
	}

	template<typename TReceive>
	TReceive read() noexcept {
		TReceive receive;
		unsigned char* pb = reinterpret_cast<unsigned char*>(std::addressof(receive));

		std::size_t cbRead = 0;
		while(cbRead<sizeof(TReceive)) {
			int cb = ::read(m_fd, pb, sizeof(TReceive)-cbRead);
			ASSERT(0<=cb);
			if(0<cb) {
				pb+=cb;
				cbRead+=static_cast<std::size_t>(cb);
			}
		}
		return receive;
	}

	~serial_device() {
		VERIFY(0==tcsetattr(m_fd, TCSANOW, &m_ttySaved));
		VERIFY(0==close(m_fd));
	}

private:
	int m_fd;
	termios m_ttySaved;
};

int main(int nArgs, char* aczArgs[]) {
	// TODO: Reset controller first
	// TODO: Replace serial_device with boost asio? 
	// TODO: Setup boost::asio TCP server to send map bitmaps?
	// OpenCV itself only uses one core, processing single sensor data takes about 0.03 s currently
	/*
	auto probotcontroller = robot_new_controller();
	while(true) {
		auto start = std::chrono::system_clock::now();

		SSensorData data = {
			0,
			0,
			200,
			{ 10, 10, 10, 10},
			ecmdSTOP
		};
		SRobotCommand cmd;
		bool bSend;
		SPose pose = robot_received_sensor_data(probotcontroller, data, &cmd, &bSend);

		auto end = std::chrono::system_clock::now();
		std::chrono::duration<double> diff = end-start;
		std::cout << "(" << pose.x << ", " << pose.y << "): " << diff.count() << " s\n";

		using namespace std::literals;
		// std::this_thread::sleep_for(1s);
	}
	*/
	if(nArgs<2) {
		std::cout << "Syntax: robot <ttyDevice>\n";
		return 1;
	}

	std::cout << "Opening " << aczArgs[1] << "\n";
	serial_device serial(aczArgs[1]);
	std::cout << "Waiting for Arduino\n";

	bool bSentReset = false;
	while(true) {
		auto chHandshake = serial.read<char>();
		std::cout << chHandshake;
		if(chHandshake==g_chHandshake) {
			break;
		} else if(!bSentReset) {
			bSentReset = true;
			serial.write(SRobotCommand::reset());
		}
	}

	std::cout << "\n Send Handshake\n";
	serial.write(g_chHandshake);

	auto start = std::chrono::system_clock::now();
	while(true) {
		SSensorData data = serial.read<SSensorData>();
		
		auto end = std::chrono::system_clock::now();
		std::chrono::duration<double> diff = end-start;

		std::cout << diff.count() << ": " << data.m_nYaw << " "
			<< data.m_nAngle << " "
			<< data.m_nDistance 
			<< std::endl;
	}
	return 0;
}