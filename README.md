# MappingRover2

This project is a platform that lets me explore my technical interests. I've 3D printed robot parts and soldered a Raspberry Pi robot hat. Mostly I'm reading Sebastian Thrun's "Probabilistic Robotics" and I'm implementing here what I learned in the book and in online lectures.      

This project is an enhanced version of [my original robot](https://github.com/stheophil/MappingRover) that required BLE connection to a Mac which ran the main algorithms. The robot hardware is in large parts identical. It is very much a work in progress.

**Hardware**

- Dagu Rover 5 robot with offroad wheels (for the looks)
- [Neato XV-11 Lidar](https://xv11hacking.wikispaces.com/LIDAR+Sensor)
- Teensy 2.0++ microcontroller as motor interface (recommended for the large number of I/O pins, especially interrupt-enabled pins), 
- connected via USB to an [up-board](http://www.up-board.org/up/specifications/) (quad-core Intel Atom x5-Z8350)

**Software**

- `arduino/` contains the microcontroller code and the header with the data definitions shared between the microcontroller and the Raspberry

	- It uses the http://platformio.org toolset to build and deploy the microcontroller software directly from the Raspberry
	- Uses PID motor control library: https://github.com/br3ttb/Arduino-PID-Library 

- `raspberry/` contains the more "interesting" parts of the code

	- It uses CMake as a build toolset and requires boost >= 1.55 and OpenCV >= 3.0
	- It includes 'libicp', an interative closest point solver from http://www.cvlibs.net/software/libicp/ which can use OpenMP if available
	- Build and run `rover --help` to get information about command line arguments. Currently, two modes of operation are supported:
		1. `./rover --port /dev/ttyUSBPORT --lidar /dev/ttyLIDARPORT --manual --map map.png` tries to connect to the microcontroller on USB serial port `/dev/ttyUSBPORT` and the Neato lidar on port `/dev/ttyLIDARPORT`, let's you control the robot using the `ERT - DG - CVB` keys and outputs the current map with the robot's pose to `map.png`. <br/><br/>
		The robot can also be controlled with a gamepad. Use e.g. nginx to host `raspberry/html/map.html` and open the page in a modern browser that supports the gamepad API, e.g., the current version of Chrome. If you have a supported gamepad, the website will send control commands to the `rover` executable which is listening on port 8088 for control commands. Use the `--map` argument to overwrite the hosted map `raspberry/html/map.png` regularly. This way, you can control the robot via the browser and see the generated map in the browser.

		2. `./rover --input-file log.txt` reads the sensor data, runs a SLAM algorithm on the data, and outputs `log.txt.mov`. Useful for evaluating algorithms offline without powering up the robot. 
	- `raspberry/test` contains a sample log file and sample outputs of the algorithms implemented in `deadreckoning.cpp`, `particle_slam.cpp` and `scanmatching.cpp` respectively. 

# Build Setup 

_The following is very incomplete. Haven't tried this out on a new machine in a while._

**PlatformIO**

`pip install -U platformio`

`sudo apt-get install cmake`

**Install boost (>=1.55)**
    
`sudo apt-get install libboost-all-dev`

**Build OpenCV**

See http://docs.opencv.org/3.0-beta/doc/tutorials/introduction/linux_install/linux_install.html

`sudo apt-get install libopencv-dev`

**Install libmicrohttpd**
    
`sudo apt-get install libmicrohttpd`

# Building 

    cd arduino 
    platformio run

    cd ../raspberry
    mkdir build
    cmake ..
    make
