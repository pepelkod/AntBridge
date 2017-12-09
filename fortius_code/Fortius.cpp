/*
 * Copyright (c) 2011 Mark Liversedge (liversedge@gmail.com)
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 2 of the License, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc., 51
 * Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 */

#include <iostream>
#include <string.h>

#include "Fortius.h"
#include "EndianSwap.h"


#define MAX_LOAD_WATTS 1000

//
// Outbound control message has the format:
// Byte          Value / Meaning
// 0             0x01 CONSTANT
// 1             0x08 CONSTANT
// 2             0x01 CONSTANT
// 3             0x00 CONSTANT
// 4             Brake Value - Lo Byte
// 5             Brake Value - Hi Byte
// 6             Echo cadence sensor
// 7             0x00 -- UNKNOWN
// 8             0x02 -- 0 - idle, 2 - Active, 3 - Calibration
// 9             0x52 -- Mode 0a = ergo, weight for slope mode (48 = 72kg), 52 = idle (in conjunction with byte 8)
// 10            Calibration Value - Lo Byte
// 11            Calibration High - Hi Byte

// Encoded Calibration is 130 x Calibration Value + 1040 so calibration of zero gives 0x0410

const static uint8_t ergo_command[12] = {
	// 0     1     2     3     4     5     6     7    8      9     10    11
	0x01, 0x08, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x0a, 0x10, 0x04
};

const static uint8_t slope_command[12] = {
	// 0     1     2     3     4     5     6     7    8      9     10    11
	0x01, 0x08, 0x01, 0x00, 0x6c, 0x01, 0x00, 0x00, 0x02, 0x48, 0x10, 0x04
};



/* ----------------------------------------------------------------------
 * CONSTRUCTOR/DESRTUCTOR
 * ---------------------------------------------------------------------- */
Fortius::Fortius()
{

	devicePower = deviceHeartRate = deviceCadence = deviceSpeed = 0.00;
	mode = FT_IDLE;
	load = DEFAULT_LOAD;
	gradient = DEFAULT_GRADIENT;
	weight = DEFAULT_WEIGHT;
	brakeCalibrationFactor = DEFAULT_CALIBRATION;
	powerScaleFactor = DEFAULT_SCALING;
	deviceStatus = 0;

	/* 12 byte control sequence, composed of 8 command packets
	 * where the last packet sets the load. The first byte
	 * is a CRC for the value being issued (e.g. Load in WATTS)
	 *
	 * these members are modified as load / gradient are set
	 */
	memcpy(ERGO_Command, ergo_command, 12);
	memcpy(SLOPE_Command, slope_command, 12);

	std::cout << "\nFortius::Fortius: new LibUsb\n";
	// for interacting over the USB port
	usb2 = new LibUsb(TYPE_FORTIUS);

	std::cout << "\nFortius::Fortius: pthread_mutex_init\n";
	pthread_mutex_init(&pvars, NULL);
}

Fortius::~Fortius()
{
	pthread_mutex_destroy(&pvars);
}

/* ----------------------------------------------------------------------
 * SET
 * ---------------------------------------------------------------------- */
void Fortius::setMode(int mode)
{
	pthread_mutex_lock(&pvars);
	this->mode = mode;
	pthread_mutex_unlock(&pvars);
}

// Alters the relationship between brake setpoint at load.
void Fortius::setBrakeCalibrationFactor(double brakeCalibrationFactor)
{
	pthread_mutex_lock(&pvars);
	this->brakeCalibrationFactor = brakeCalibrationFactor;
	pthread_mutex_unlock(&pvars);
}

// output power adjusted by this value so user can compare with hub or crank based readings
void Fortius::setPowerScaleFactor(double powerScaleFactor)
{
	if (weight < 0.8) {
		weight = 0.8;
	}
	if (weight > 1.2) {
		weight = 1.2;
	}

	pthread_mutex_lock(&pvars);
	this->powerScaleFactor = powerScaleFactor;
	pthread_mutex_unlock(&pvars);
}

// User weight used by brake in slope mode
void Fortius::setWeight(double weight)
{
	// need to apply range as same byte used to signify erg mode
	if (weight < 50) {
		weight = 50;
	}
	if (weight > 120) {
		weight = 120;
	}

	pthread_mutex_lock(&pvars);
	this->weight = weight;
	pthread_mutex_unlock(&pvars);
}
void Fortius::setLoadPercentage(double loadPercentage)
{
	double loadWatts;

	loadWatts = (loadPercentage/100)*MAX_LOAD_WATTS;

	setLoad(loadWatts);
}

double Fortius::getLoadPercentage()
{
	double loadWatts;
	double loadPercentage;

	loadWatts = getLoad();

	loadPercentage = 100 * ( loadWatts/MAX_LOAD_WATTS );
	
	return loadPercentage;
}

// Load in watts when in power mode
void Fortius::setLoad(double load)
{
	// we can only do 50-1000w on a Fortius
	if (load > MAX_LOAD_WATTS) {
		load = MAX_LOAD_WATTS;
	}
	if (load < 50) {
		load = 50;
	}

	pthread_mutex_lock(&pvars);
	this->load = load;
	pthread_mutex_unlock(&pvars);
}

// Load as slope % when in slope mode
void Fortius::setGradient(double gradient)
{
	if (gradient > 20) {
		gradient = 20;
	}
	if (gradient < -5) {
		gradient = -5;
	}

	pthread_mutex_lock(&pvars);
	this->gradient = gradient;
	pthread_mutex_unlock(&pvars);
}


/* ----------------------------------------------------------------------
 * GET
 * ---------------------------------------------------------------------- */
void Fortius::getTelemetry(double& powerWatts, double& heartrateBPM, double& cadenceRPM, double& speedKPH, double& distanceM, int& buttons, int& steering, int& status)
{

	pthread_mutex_lock(&pvars);
	powerWatts = devicePower;
	heartrateBPM = deviceHeartRate;
	cadenceRPM = deviceCadence;
	speedKPH = deviceSpeed;
	distanceM = deviceDistance;
	buttons = deviceButtons;
	steering = deviceSteering;
	status = deviceStatus;

	// work around to ensure controller doesn't miss button press.
	// The run thread will only set the button bits, they don't get
	// reset until the ui reads the device state
	deviceButtons = 0;
	pthread_mutex_unlock(&pvars);
}

int Fortius::getMode()
{
	int  tmp;
	pthread_mutex_lock(&pvars);
	tmp = mode;
	pthread_mutex_unlock(&pvars);
	return tmp;
}

double Fortius::getLoad()
{
	double tmp;
	pthread_mutex_lock(&pvars);
	tmp = load;
	pthread_mutex_unlock(&pvars);
	return tmp;
}

double Fortius::getGradient()
{
	double tmp;
	pthread_mutex_lock(&pvars);
	tmp = gradient;
	pthread_mutex_unlock(&pvars);
	return tmp;
}

double Fortius::getWeight()
{
	double tmp;
	pthread_mutex_lock(&pvars);
	tmp = weight;
	pthread_mutex_unlock(&pvars);
	return tmp;
}

double Fortius::getBrakeCalibrationFactor()
{
	double tmp;
	pthread_mutex_lock(&pvars);
	tmp = brakeCalibrationFactor;
	pthread_mutex_unlock(&pvars);
	return tmp;
}

double Fortius::getPowerScaleFactor()
{
	double tmp;
	pthread_mutex_lock(&pvars);
	tmp = powerScaleFactor;
	pthread_mutex_unlock(&pvars);
	return tmp;
}

int
Fortius::start()
{
	pthread_mutex_lock(&pvars);
	this->deviceStatus = FT_RUNNING;
	pthread_mutex_unlock(&pvars);

	std::cout << "\nFortius::start: pthread_create\n";
	pthread_create(&thread_handle, NULL, Fortius::run_helper, this);
	return 0;
}

int
Fortius::join()
{
	pthread_join(thread_handle, NULL);
	return 0;
}


/* ----------------------------------------------------------------------
 * EXECUTIVE FUNCTIONS
 *
 * start() - start/re-start reading telemetry in a thread
 * stop() - stop reading telemetry and terminates thread
 * pause() - discards inbound telemetry (ignores it)
 *
 *
 * THE MEAT OF THE CODE IS IN RUN() IT IS A WHILE LOOP CONSTANTLY
 * READING TELEMETRY AND ISSUING CONTROL COMMANDS WHILST UPDATING
 * MEMBER VARIABLES AS TELEMETRY CHANGES ARE FOUND.
 *
 * run() - bg thread continuosly reading/writing the device port
 *         it is kicked off by start and then examines status to check
 *         when it is time to pause or stop altogether.
 * ---------------------------------------------------------------------- */
int Fortius::restart()
{
	int status;

	// get current status
	pthread_mutex_lock(&pvars);
	status = this->deviceStatus;
	pthread_mutex_unlock(&pvars);
	// what state are we in anyway?
	if (status & FT_RUNNING && status & FT_PAUSED) {
		status &= ~FT_PAUSED;
		pthread_mutex_lock(&pvars);
		this->deviceStatus = status;
		pthread_mutex_unlock(&pvars);
		return 0; // ok its running again!
	}
	return 2;
}

int Fortius::stop()
{
	// what state are we in anyway?
	pthread_mutex_lock(&pvars);
	deviceStatus = 0; // Terminate it!
	pthread_mutex_unlock(&pvars);
	return 0;
}

int Fortius::pause()
{
	int status;

	// get current status
	pthread_mutex_lock(&pvars);
	status = this->deviceStatus;
	pthread_mutex_unlock(&pvars);

	if (status & FT_PAUSED) {
		return 2;    // already paused you muppet!
	} else if (!(status & FT_RUNNING)) {
		return 4;    // not running anyway, fool!
	} else {

		// ok we're running and not paused so lets pause
		status |= FT_PAUSED;
		pthread_mutex_lock(&pvars);
		this->deviceStatus = status;
		pthread_mutex_unlock(&pvars);

		return 0;
	}
}

// used by thread to set variables and emit event if needed
// on unexpected exit
int Fortius::quit(int code)
{
	pthread_mutex_lock(&pvars);
	this->deviceStatus = FT_ERROR;
	pthread_mutex_unlock(&pvars);

	// event code goes here!
	//exit(code);
	return 0; // never gets here obviously but shuts up the compiler!
}

void* Fortius::run_helper(void* This)
{
	std::cout <<"\nFortius::run_helper: entry\n";

	Fortius*    me = (Fortius*)This;

	me->run();

	return NULL;
}
/*----------------------------------------------------------------------
 * THREADED CODE - READS TELEMETRY AND SENDS COMMANDS TO KEEP FORTIUS ALIVE
 *----------------------------------------------------------------------*/
void Fortius::run()
{
	std::cout<<"\nFortius::run: starting\n";

	// newly read values - compared against cached values
	bool isDeviceOpen = false;

	// ui controller state
	int curstatus;

	// variables for telemetry, copied to fields on each brake update
	double curPower;                      // current output power in Watts
	double curHeartRate;                  // current heartrate in BPM
	double curCadence;                    // current cadence in RPM
	double curSpeed;                      // current speed in KPH
	double curDistance;                    // odometer?
	uint32_t	startDistanceDoubleRevs; // every revolution of the roller counts twice for these two variaboueles 
	uint32_t	curDistanceDoubleRevs;	// nu
	int curButtons;                       // Button status
	int curSteering;                    // Angle of steering controller
	//int curStatus;
	uint8_t pedalSensor;                // 1 when using is cycling else 0, fed back to brake although appears unnecessary

	// we need to average out power for the last second
	// since we get updates every 10ms (100hz)
	int powerhist[10];     // last 10 values received
	int powertot = 0;      // running total
	int powerindex = 0;    // index into the powerhist array
	for (int i = 0; i < 10; i++) {
		powerhist[i] = 0;
	}

	// initialise local cache & main vars
	pthread_mutex_lock(&pvars);
	//curStatus = this->deviceStatus;
	curPower = this->devicePower = 0;
	curHeartRate = this->deviceHeartRate = 0;
	curCadence = this->deviceCadence = 0;
	curSpeed = this->deviceSpeed = 0;
	curDistance = this->deviceDistance = 0;
	curSteering = this->deviceSteering = 0;
	curButtons = this->deviceButtons = 0;
	pedalSensor = 0;
	pthread_mutex_unlock(&pvars);


	// open the device
	if (openPort()) {
		std::cout<<"\nFortius::run: openPort failed with "<<strerror(errno)<<"\n";
		quit(2);
		return; // open failed!
	} else {
		isDeviceOpen = true;
		sendOpenCommand();
	}


	while(1) {
		//printf("*");
		if (isDeviceOpen == true) {
			int rc = sendRunCommand(pedalSensor) ;
			if (rc < 0) {
				std::cout << "\nFortius::run: usb write error " << rc;
				// send failed - ouch!
				//closePort(); // need to release that file handle!!
				//quit(4);
				//return; // couldn't write to the device
				continue;	// ignore this error?
			}
			int actualLength = readMessage();
			if (actualLength < 0) {
				std::cout << "\nForitus::run: usb read error " << actualLength;
			}
			if (actualLength >= 24) {

				//----------------------------------------------------------------
				// UPDATE BASIC TELEMETRY (HR, CAD, SPD et al)
				// The data structure is very simple, no bit twiddling needed here
				//----------------------------------------------------------------

				// buf[14] changes from time to time (controller status?)

				// buttons
				curButtons = buf[13];

				// steering angle
				curSteering = buf[18] | (buf[19] << 8);

				// update public fields
				pthread_mutex_lock(&pvars);
				deviceButtons |= curButtons;    // workaround to ensure controller doesn't miss button pushes
				deviceSteering = curSteering;
				pthread_mutex_unlock(&pvars);
			}
			if (actualLength == 48) {
				//printf("+");
				// brake status status&0x04 == stopping wheel
				//              status&0x01 == brake on
				//curBrakeStatus = buf[42];

				// pedal sensor is 0x01 when cycling
				pedalSensor = buf[46];

				// current distance
				curDistanceDoubleRevs = (buf[28] | (buf[29] << 8) | (buf[30] << 16) | (buf[31] << 24));
				if(startDistanceDoubleRevs == 0 || startDistanceDoubleRevs == 4100){
					startDistanceDoubleRevs = curDistance;
				}
				//printf("revs = %f\n", curDistance);
				//curDistance = curDistance * 0.063403614457831;
				curDistance = ((double)curDistanceDoubleRevs) * HALF_ROLLER_CIRCUMFERENCE_M ;	//0.06264880952;
				//curDistance = (buf[28] | (buf[29] << 8) | (buf[30] << 16) | (buf[31] << 24))/8.076009501187648;
				//printf("curDistance meters %f\n", curDistance);

				// cadence - confirmed correct
				//std::cout << "raw cadence " << buf[44];
				curCadence = buf[44];

				// speed
				//std::cout << "raw speed " << buf[32];
				curSpeed = 1.3f * (double)(FromLittleEndian<uint16_t>((uint16_t*)&buf[32])) / (3.6f * 100.00f);

				// power - changed scale from 10 to 13, seems correct in erg mode, slope mode needs work
				//std::cout << "raw power " << buf[38];
				curPower = FromLittleEndian<int16_t>((int16_t*)&buf[38]) / 13;
				if (curPower < 0.0) {
					curPower = 0.0;    // brake power can be -ve when coasting.
				}

				// average power over last 1s
				powertot += curPower;
				powertot -= powerhist[powerindex];
				powerhist[powerindex] = curPower;

				curPower = powertot / 10;
				powerindex = (powerindex == 9) ? 0 : powerindex + 1;

				curPower *= powerScaleFactor; // apply scale factor

				// heartrate - confirmed correct
				//std::cout << "raw heartrate " << buf[12];
				curHeartRate = buf[12];

				// update public fields
				pthread_mutex_lock(&pvars);
				deviceSpeed = curSpeed;
				deviceDistance = curDistance;// - (startDistanceDoubleRevs * HALF_ROLLER_CIRCUMFERENCE_M);
				deviceCadence = curCadence;
				deviceHeartRate = curHeartRate;
				devicePower = curPower;
				pthread_mutex_unlock(&pvars);


			}
			if(actualLength != 24 && actualLength != 48) {
				printf("\nFortius::run: error, got a length of %d --------------------------------------------------------------------\n", actualLength);
			}
		}

		//----------------------------------------------------------------
		// LISTEN TO GUI CONTROL COMMANDS
		//----------------------------------------------------------------
		pthread_mutex_lock(&pvars);
		curstatus = this->deviceStatus;
		pthread_mutex_unlock(&pvars);

		/* time to shut up shop */
		if (!(curstatus & FT_RUNNING)) {
			// time to stop!

			sendCloseCommand();

			closePort(); // need to release that file handle!!
			quit(0);
			return;
		}

		if ((curstatus & FT_PAUSED) && isDeviceOpen == true) {

			closePort();
			isDeviceOpen = false;

		} else if (!(curstatus & FT_PAUSED) && (curstatus & FT_RUNNING) && isDeviceOpen == false) {

			if (openPort()) {
				quit(2);
				return; // open failed!
			}
			isDeviceOpen = true;
			sendOpenCommand();

		}


		// The controller updates faster than the brake. Setting this to a low value (<50ms) increases the frequency of controller
		// only packages (24byte).
		//usleep(10000);	// 10ms
	}
}

/* ----------------------------------------------------------------------
 * HIGH LEVEL DEVICE IO ROUTINES
 *
 * sendOpenCommand() - initialises training session
 * sendCloseCommand() - finalises training session
 * sendRunCommand(int) - update brake setpoint
 *
 * ---------------------------------------------------------------------- */
int Fortius::sendOpenCommand()
{

	uint8_t open_command[] = {0x02, 0x00, 0x00, 0x00};

	int retCode = rawWrite(open_command, 4);
	//std::cout << "usb status " << retCode;
	return retCode;
}

int Fortius::sendCloseCommand()
{
	uint8_t close_command[] = {0x01, 0x08, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x52, 0x10, 0x04};

	int retCode = rawWrite(close_command, 12);
	//std::cout << "usb status " << retCode;
	return retCode;
}

int Fortius::sendRunCommand(int16_t pedalSensor)
{
	int retCode = 0;
	pthread_mutex_lock(&pvars);
	int mode = this->mode;
	int16_t gradient = (int16_t)this->gradient;
	int16_t load = (int16_t)this->load;
	unsigned int weight = (unsigned int)this->weight;
	int16_t brakeCalibrationFactor = (int16_t)this->brakeCalibrationFactor;
	pthread_mutex_unlock(&pvars);

	if (mode == FT_ERGOMODE) {
		//std::cout << "send load " << load;
		ToLittleEndian<int16_t>(13 * load, (int16_t*)&ERGO_Command[4]);
		ERGO_Command[6] = pedalSensor;

		ToLittleEndian<int16_t>(130 * brakeCalibrationFactor + 1040, (int16_t*)&ERGO_Command[10]);

		retCode = rawWrite(ERGO_Command, 12);
	} else if (mode == FT_SSMODE) {
		// Tacx driver appears to add an offset to create additional load at
		// zero slope, also seems to be slightly dependent on weight but have
		// ignored this for now.
		ToLittleEndian<int16_t>(1300 * gradient + 507, (int16_t*)&SLOPE_Command[4]);
		SLOPE_Command[6] = pedalSensor;
		SLOPE_Command[9] = weight;

		ToLittleEndian<int16_t>((int16_t)(130 * brakeCalibrationFactor + 1040), (int16_t*)(&SLOPE_Command[10]));

		retCode = rawWrite(SLOPE_Command, 12);
	} else if (mode == FT_IDLE) {
		retCode = sendOpenCommand();
	} else if (mode == FT_CALIBRATE) {
		// Not yet implemented, easy enough to start calibration but appears that the calibration factor needs
		// to be calculated by observing the brake power and speed after calibration starts (i.e. it's not returned
		// by the brake).
	}

	//std::cout << "usb status " << retCode;
	return retCode;
}

/* ----------------------------------------------------------------------
 * LOW LEVEL DEVICE IO ROUTINES - PORT TO QIODEVICE REQUIRED BEFORE COMMIT
 *
 *
 * readMessage()        - reads an inbound message
 * openPort() - opens serial device and configures it
 * closePort() - closes serial device and releases resources
 * rawRead() - non-blocking read of inbound data
 * rawWrite() - non-blocking write of outbound data
 * discover() - check if a ct is attached to the port specified
 * ---------------------------------------------------------------------- */
int Fortius::readMessage()
{
	int rc;

	rc = rawRead(buf, 64);
	//std::cout << "usb status " << rc;
	return rc;
}

int Fortius::closePort()
{
	usb2->close();
	return 0;
}

bool Fortius::find()
{
	int rc;
	rc = usb2->find();
	//std::cout << "usb status " << rc;
	return rc;
}

int Fortius::openPort()
{
	int rc;
	// on windows we try on USB2 then on USB1 then fail...
	rc = usb2->open();
	//std::cout << "usb status " << rc;
	return rc;
}

int Fortius::rawWrite(uint8_t* bytes, int size) // unix!!
{
	return usb2->write((char*)bytes, size, FT_USB_TIMEOUT);
}

int Fortius::rawRead(uint8_t bytes[], int size)
{
	return usb2->read((char*)bytes, size, FT_USB_TIMEOUT);
}

// check to see of there is a port at the device specified
// returns true if the device exists and false if not
bool Fortius::discover(char*)
{
	return true;
}
