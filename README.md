# AntBridge

The primary code is int "fortius_code"
The code runs in two loops.  A fortius loop polls the Fortius trainer.  The CANTMaster loop reads values from the Fortius object and from the ANT stick.


Building instructions.
sudo apt-get install libusb-dev
sudo apt-get install libgoogle-glog-dev
sudo apt-get install libusb-1.0-0-dev

cd ant_code
make;make;		// for some reason the first time thru, the archiver gets permission denied...make twice cause I'm too lazy to fix it.

cd ../fortius_code
make
