cd arduino
platformio run
cd ../raspberry
make
./robot /dev/ttyACM0 ./log.txt
