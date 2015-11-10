#include "error_handling.h"
#include "../arduino/rover.h"
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
		speed_t spd = B57600;
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

int main() {
	// TODO: Reset controller first
	serial_device serial("/dev/ttyACM0");
	std::cout << "Waiting for Arduino\n";
	while(true) {
		auto chHandshake = serial.read<char>();
		std::cout << chHandshake;
		if(chHandshake==g_chHandshake) break;
	}

	std::cout << "\n Send Handshake\n";
	serial.write(g_chHandshake);

	while(true) {
		SSensorData data = serial.read<SSensorData>();
		std::cout << data.m_nYaw << " "
			<< data.m_nAngle << " "
			<< data.m_nDistance << " "
			<< data.m_ecmdLast << std::endl;	
	}
	return 0;
}