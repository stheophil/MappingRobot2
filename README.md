# MappingRover2

This project is a platform that lets me explore my technical interests. I've 3D printed robot parts and soldered a Raspberry Pi robot hat. Mostly I'm reading Sebastian Thrun's "Probabilistic Robotics" and I'm implementing here what I learned in the book and in online lectures.      

This project is an enhanced version of [my original robot](https://github.com/stheophil/MappingRover) that required BLE connection to a Mac which ran the main algorithms. The robot hardware is in large parts identical. It is very much a work in progress.

**Hardware**

- Dagu Rover 5 robot with offroad wheels (for the looks)
- LidarLite v2 sensor mounted on Dynamixel XL-320 servo 
- Teensy 2.0++ microcontroller as sensor & motor interface (recommended for the large number of I/O pins, especially interrupt-enabled pins), 
- connected via USB to a Raspberry Pi (v3 is recommended for its performance, the faster the better) 

**Software**

- `arduino/` contains the microcontroller code and the header with the data definitions shared between the microcontroller and the Raspberry

	- It uses the http://platformio.org toolset to build and deploy the microcontroller software directly from the Raspberry
	- Several libraries are required 
		- PID motor control: https://github.com/br3ttb/Arduino-PID-Library 
		- Dynamixel communication: https://github.com/hackerspace-adelaide/XL320 and https://github.com/hackerspace-adelaide/HalfDuplexHardwareSerial
		- LIDARLite: https://github.com/PulsedLight3D/LIDARLite_v2_Arduino_Library
	- If you want to use the XL-320 servo, read http://hackerspace-adelaide.org.au/wiki/Dynamixel_XL-320

- `raspberry/` contains the more "interesting" parts of the code

	- It uses CMake as a build toolset and requires boost >= 1.55 and OpenCV >= 3.0
	- It includes 'libicp', an interative closest point solver from http://www.cvlibs.net/software/libicp/ which can use OpenMP if available
	- Build and run `rover --help` to get information about command line arguments. Currently, two modes of operation are supported:
		1. `./rover --port /dev/ttyUSBPORT --manual --log log.txt` tries to connect to the microcontroller on USB serial port `/dev/ttyUSBPORT`, let's you control the robot using the `WASD` keys and logs all sensor data to `log.txt`
		2. `./rover --input-file log.txt` reads the sensor data, runs a SLAM algorithm on the data, and outputs `log.txt.mov`. Useful for evaluating algorithms offline without powering up the robot. 
	- `raspberry/test` contains a sample log file and sample outputs of the algorithms implemented in `deadreckoning.cpp`, `particle_slam.cpp` and `scanmatching.cpp` respectively. 

# Build Setup 

_The following is ery incomplete. Haven't tried this out on a new machine in a while._

**PlatformIO**

`pip install -U platformio`

`sudo apt-get install cmake`

**Install boost (>=1.55)**
    
`sudo apt-get install libboost-all-dev`

**Build OpenCV**

See http://docs.opencv.org/3.0-beta/doc/tutorials/introduction/linux_install/linux_install.html

`sudo apt-get install libopencv-dev`

# Building 

    cd arduino 
    platformio run

    cd ../raspberry
    mkdir build
    cmake ..
    make
