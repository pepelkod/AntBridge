# AntBridge

The primary code is int "fortius_code"
The code runs in two loops.  A fortius loop polls the Fortius trainer.  The CANTMaster loop reads values from the Fortius object and from the ANT stick.


Building instructions.
sudo apt-get install libusb-dev
sudo apt-get install libgoogle-glog-dev
sudo apt-get install libusb-1.0-0-dev


make; make;		# still having problems with libanty.a not getting permission to build the first time.
sudo make install	# installs to /usr/local/lib and /usr/local/bin
