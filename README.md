# AntBridge

The primary code is int "fortius_code"
The code runs in two loops.  A fortius loop polls the Fortius trainer.  The CANTMaster loop reads values from the Fortius object and from the ANT stick.

Remove usb-serial-simple.ko and usbserial.ko. These drivers will grab the dynastream usb device denying access to it from our application.
LOC=/lib/modules/`uname -r`/kernel/drivers/usb/serial/
sudo mv $LOC/usb-serial-simple.ko ~/Documents
sudo mv $LOC/usbserial.ko ~/Documents
You may also need to rmmod the modules from running.
sudo rmmod usb_serial_simple usbserial

Building instructions.
sudo apt install libusb-dev
sudo apt install libgoogle-glog-dev
sudo apt install libusb-1.0-0-dev
sudo apt install pkg-config

make; make;		# still having problems with libanty.a not getting permission to build the first time.
sudo make install	# installs to /usr/local/lib and /usr/local/bin
