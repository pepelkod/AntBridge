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
#include <glog/logging.h>


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
const static uint8_t calibrate_command[12] = {
	// 0     1     2     3     4     5     6     7     8     9    10    11
	0x01, 0x08, 0x01, 0x00, 0xa3, 0x16, 0x00, 0x00, 0x03, 0x52, 0x00, 0x00	// direct read from Fortius software usb capture
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
	brakeCalibrationLoadRaw = DEFAULT_CALIBRATION_LOAD_RAW;			//	650  			// 0-1300 seems reasonable
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
	memcpy(CALIBRATE_Command, calibrate_command, 12);

	VLOG(1) << "Fortius::Fortius: new LibUsb";
	// for interacting over the USB port
	usb2 = new LibUsb(TYPE_FORTIUS);

 	VLOG(1) << "Fortius::Fortius: pthread_mutex_init";
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

double Fortius::getBrakeCalibrationLoadRaw()
{
	double calibLoad;

	pthread_mutex_lock(&pvars);
	calibLoad = this->brakeCalibrationLoadRaw;
	pthread_mutex_unlock(&pvars);
	return calibLoad;
}


void Fortius::setBrakeCalibrationLoadRaw(double load)
{
	pthread_mutex_lock(&pvars);
	this->brakeCalibrationLoadRaw = load;
	pthread_mutex_unlock(&pvars);
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

	VLOG(1) << "Fortius::start: pthread_create";
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

	VLOG(1) << "Exit code: " << code;
	//printf("exit code %d\n", code);

	// event code goes here!
	//exit(code);
	return 0; // never gets here obviously but shuts up the compiler!
}

void* Fortius::run_helper(void* This)
{
	VLOG (1) << "Fortius::run_helper: entry";

	Fortius*    me = (Fortius*)This;

	me->run();

	return NULL;
}

void Fortius::hex_dump(uint8_t* data, int data_size)
{
	int cnt;

	printf ("\n");
  for (cnt=0; cnt < data_size; cnt++) {
  	printf ("%02x ", data[cnt]);
    if(cnt%8==7) {
    	printf(" ");
    }
		if(cnt%16==15){
			printf("\n");
		}
  }
}

double Fortius::calculateWattageFromRaw(double curRawPower, double curRawSpeed){
	double curBrakeCalibrationLoadRaw;
	double slopeCalc;
	double offsetCalc;
	double powerCalcWatts;

	pthread_mutex_lock(&pvars);
	curBrakeCalibrationLoadRaw = brakeCalibrationLoadRaw;	// get member
	pthread_mutex_unlock(&pvars);

	// old slopeCalc = 0.001366 * curDeviceSpeed + 0.0308;
	// newer slopeCalc = 0.191 * curDeviceSpeed + 0.076;
	slopeCalc = 0.00000670 * (curRawSpeed) + 0.002;

	//offsetCalc = -0.03526 * curBrakeCalibrationLoadRaw + 1.708;
	offsetCalc = 0;

	//printf("\n slope %f offset %f\n", slopeCalc, offsetCalc);
	powerCalcWatts = (slopeCalc * curRawPower) + offsetCalc;

	return powerCalcWatts;

}
double Fortius::calculateRawLoadFromWattage(double requiredWatts){
	double curBrakeCalibrationLoadRaw;
	double slopeCalc;
	double offsetCalc;
	double powerRaw;
	double curRawSpeed;

	pthread_mutex_lock(&pvars);
	curBrakeCalibrationLoadRaw = brakeCalibrationLoadRaw;	// get member
	curRawSpeed = rawSpeed;
	pthread_mutex_unlock(&pvars);

	// oldslopeCalc = 0.001366 * curDeviceSpeed + 0.0308;
	// newer slopeCalc = 0.191 * curDeviceSpeed + 0.076;
	if(curRawSpeed==0){
		curRawSpeed=2200;	// minimum 5 mph
	}
	slopeCalc = 0.00000670 * (curRawSpeed) + 0.002;

	//offsetCalc = -0.03526 * curBrakeCalibrationLoadRaw + 1.708;
	offsetCalc = 0;	// debug

	powerRaw = ((requiredWatts - offsetCalc)/slopeCalc);

	//printf("\n pwer raw %f slope calc %f offsetCalc %f required watts %f\n", powerRaw, slopeCalc, offsetCalc, requiredWatts);
	return powerRaw;
}


/*----------------------------------------------------------------------
 * THREADED CODE - READS TELEMETRY AND SENDS COMMANDS TO KEEP FORTIUS ALIVE
 *----------------------------------------------------------------------*/
void Fortius::run()
{
	VLOG (1) << "Fortius::run: starting";

	// newly read values - compared against cached values
	bool isDeviceOpen = false;

	// ui controller state
	int curstatus;

	// variables for telemetry, copied to fields on each brake update
	double curPower;                      // current output power in Watts
	double nextPower;                     // used for exponential moving average of power
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
	double next_calibration_load_raw;
	double cur_calibration_load_raw;
	double curRawSpeed;
	double curRawPower;			// read the raw power number from 48 byte message...THIS IS NOT WATTS? is it TORQUE?
	timespec last_measured_time;

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

	// Store currrent time
	clock_gettime (CLOCK_MONOTONIC, &last_measured_time);

	while(1) {
		//printf("*");
		if (isDeviceOpen == true) {
			// Sleep for 250 msec after read before writing
			go_sleep (&last_measured_time, FT_READ_DELAY);
			// do calibration mode
			int rc = sendRunCommand(pedalSensor);

			// Store currrent time
			clock_gettime (CLOCK_MONOTONIC, &last_measured_time);

			if (rc < 0) {
				std::cout << "Fortius::run: usb write error" << rc;
				// send failed - ouch!
				closePort(); // need to release that file handle!!
				if(openPort()){
					std::cout << "Fortius::run: failed attempt to close and reopen port";
					quit(2);
					return;
				}
				continue;	// ignore this error?
			}
			// Sleep for 70 msec after write before reading again
			go_sleep (&last_measured_time, FT_WRITE_DELAY);
			int actualLength = readMessage();
			VLOG (2) << actualLength;

			// Store currrent time
			clock_gettime (CLOCK_MONOTONIC, &last_measured_time);

			if (actualLength < 0) {
				std::cout << "Fortius::run: usb read error " << actualLength;
				closePort(); // need to release that file handle!!
				if(openPort()){
					std::cout << "Fortius::run: failed attempt to close and reopen port";
					quit(2);
					return;
				}
				continue;	// ignore this error?
			}
			else {
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

			  if (actualLength >= 48) {
				//printf("+");
				// brake status status&0x04 == stopping wheel
				//              status&0x01 == brake on
				//curBrakeStatus = buf[42];

					// pedal sensor is 0x01 when cycling
					pedalSensor = buf[46];

					// current distance
					curDistanceDoubleRevs = (buf[28] | (buf[29] << 8) | (buf[30] << 16) | (buf[31] << 24));
					if (startDistanceDoubleRevs == 0 || startDistanceDoubleRevs == 4100){
						startDistanceDoubleRevs = curDistance;
					}
					curDistance = ((double)curDistanceDoubleRevs) * HALF_ROLLER_CIRCUMFERENCE_M ;	//0.06264880952;

					// cadence - confirmed correct
					curCadence = buf[44];

					// speed
					curRawSpeed =  (double)(FromLittleEndian<uint16_t>((uint16_t*)&buf[32]));
					curSpeed = 1.3f * curRawSpeed / (3.6f * 100.00f);

					// power
					curRawPower = FromLittleEndian<int16_t>((int16_t*)&buf[38]);
					if(FT_CALIBRATE == mode){
						next_calibration_load_raw = curRawPower;
						next_calibration_load_raw *= 0.9;
						cur_calibration_load_raw = getBrakeCalibrationLoadRaw();
						cur_calibration_load_raw *= 0.1;
						cur_calibration_load_raw += next_calibration_load_raw;
						setBrakeCalibrationLoadRaw(cur_calibration_load_raw);
					}

					nextPower = calculateWattageFromRaw(curRawPower, curRawSpeed);
					if (nextPower < 0.0) {
						nextPower = 0.0;    // brake power can be -ve when coasting.
					}

					// EMA power
					nextPower *= 0.25;
					curPower *= 0.75;
					curPower += nextPower;

					curPower *= powerScaleFactor; // apply scale factor

					// heartrate - confirmed correct
					curHeartRate = buf[12];

					// update public fields
					pthread_mutex_lock(&pvars);
					deviceSpeed = curSpeed;
					deviceDistance = curDistance;
					deviceCadence = curCadence;
					deviceHeartRate = curHeartRate;
					devicePower = curPower;

					rawPower = curRawPower;
					rawSpeed = curRawSpeed;
					pthread_mutex_unlock(&pvars);

			  }

			  if(actualLength != 24 && actualLength != 48) {
					std::cout << "Fortius::run: error, got a length of " << actualLength << std::endl;
			   }
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

		//ToLittleEndian<int16_t>(13 * load, (int16_t*)&ERGO_Command[4]);
		ToLittleEndian<int16_t>(calculateRawLoadFromWattage(load), (int16_t*)&ERGO_Command[4]);
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
		retCode = rawWrite(CALIBRATE_Command, 12);
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

void Fortius::go_sleep (timespec *last_measured_time, int delay_msec)
{
	timespec 	current_time;
	timespec	time_passed;
	int				time_passed_msec;

	clock_gettime (CLOCK_MONOTONIC, &current_time);

	time_passed = timespec_diff (last_measured_time, &current_time);
	time_passed_msec = (int)time_passed.tv_sec * 1000 + (double)time_passed.tv_nsec/1000000.0;
	if (delay_msec - time_passed_msec > 0) {
		VLOG (2) << "Delay: " << delay_msec - time_passed_msec << " [msec]";
		usleep ((delay_msec - time_passed_msec) * 1000);
	}
}

timespec Fortius::timespec_diff(timespec *start, timespec *end)
{
	timespec temp;

	if ((end->tv_nsec-start->tv_nsec)<0) {
		temp.tv_sec = end->tv_sec-start->tv_sec-1;
		temp.tv_nsec = 1000000000+end->tv_nsec-start->tv_nsec;
	} else {
		temp.tv_sec = end->tv_sec-start->tv_sec;
		temp.tv_nsec = end->tv_nsec-start->tv_nsec;
	}
	return temp;
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
