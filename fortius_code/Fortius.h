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


// I have consciously avoided putting things like data logging, lap marking,
// intervals or any load management functions in this class. It is restricted
// to controlling an reading telemetry from the device
//
// I expect higher order classes to implement such functions whilst
// other devices (e.g. ANT+ devices) may be implemented with the same basic
// interface
//
// I have avoided a base abstract class at this stage since I am uncertain
// what core methods would be required by say, ANT+ or Tacx devices


#ifndef _GC_Fortius_h
#define _GC_Fortius_h 1
//#include "GoldenCheetah.h"

/*
#include <QString>
#include <QDialog>
#include <QDebug>
#include <QThread>
#include <QMutex>
#include <QFile>
#include <QtCore/qendian.h>
#include "RealtimeController.h"
*/

#include "LibUsb.h"

#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <stdlib.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/types.h>
#include <pthread.h>
#include <time.h>

/* Device operation mode */
#define FT_IDLE        0x00
#define FT_ERGOMODE    0x01
#define FT_SSMODE      0x02
#define FT_CALIBRATE   0x04

/* Buttons */
#define FT_PLUS        0x04
#define FT_MINUS       0x02
#define FT_CANCEL      0x08
#define FT_ENTER       0x01

/* Control status */
#define FT_RUNNING     0x01
#define FT_PAUSED      0x02
#define FT_ERROR       0x04

// Delays in msec
#define FT_READ_DELAY		240
#define FT_WRITE_DELAY	70

#define DEFAULT_LOAD         100.00
#define DEFAULT_GRADIENT     2.00
#define DEFAULT_WEIGHT       77
#define DEFAULT_CALIBRATION  0
#define DEFAULT_SCALING      1.00

#define DEFAULT_CALIBRATION_LOAD_RAW	650  			// 0-1300 seems reasonable

#define HALF_ROLLER_CIRCUMFERENCE_M 0.06264880952	// distance given is number of

#define FT_USB_TIMEOUT      500

class Fortius
{

public:
	Fortius();
	~Fortius();


	// HIGH-LEVEL FUNCTIONS
	int start();                                // Calls pthread to start
	int restart();                              // restart after paused
	int pause();                                // pauses data collection, inbound telemetry is discarded
	int stop();                                 // stops data collection thread
	int quit(int error);                        // called by thread before exiting
	int join();				    // wait on thread
	void hex_dump(uint8_t* data, int data_size);// debug
	double calculateWattageFromRaw(double powerRaw, double speedKph);	// convert raw power number into a wattage. depends on speed
	double calculateRawLoadFromWattage(double requiredWatts); // figures out what raw load numbers to put in for a desired wattage load in erg mode
	void run();                                 // called by start to kick off the CT control thread
	static void* run_helper(void* This);	    // static version that calls run so pthread can call it


	bool find();                                // either unconfigured or configured device found
	bool discover(char* deviceFilename);        // confirm CT is attached to device

	// SET
	void setLoad(double load);                  // set the load to generate in ERGOMODE
	void setLoadPercentage(double load);        // put in load in range 0-100%
	void setGradient(double gradient);          // set the load to generate in SSMODE
	void setBrakeCalibrationFactor(double calibrationFactor);     // Impacts relationship between brake setpoint and load
	void setPowerScaleFactor(double calibrationFactor);         // Scales output power, so user can adjust to match hub or crank power meter
	void setMode(int mode);
	void setWeight(double weight);                 // set the total weight of rider + bike in kg's
	void setBrakeCalibrationLoadRaw(double load);

	int getMode();
	double getGradient();
	double getLoad();
	double getLoadPercentage();			// get load back in 0-100%
	double getBrakeCalibrationFactor();
	double getPowerScaleFactor();
	double getWeight();
	double getBrakeCalibrationLoadRaw();

	// GET TELEMETRY AND STATUS
	// direct access to class variables is not allowed because we need to use wait conditions
	// to sync data read/writes between the run() thread and the main gui thread
	void getTelemetry(double& powerWatts, double& heartrateBPM, double& cadenceRPM, double& speedKMH, double& distanceM, int& buttons, int& steering, int& status);

private:
	pthread_t           thread_handle;
	pthread_mutex_t    pvars;


	uint8_t ERGO_Command[12],
		SLOPE_Command[12],
		CALIBRATE_Command[12];

	// Utility and BG Thread functions
	int openPort();
	int closePort();

	// Protocol encoding
	int sendRunCommand(int16_t pedalSensor);
	int sendOpenCommand();
	int sendCloseCommand();

	// Protocol decoding
	int readMessage();
	//void unpackTelemetry(int &b1, int &b2, int &b3, int &buttons, int &type, int &value8, int &value12);

	void go_sleep (timespec *last_measured_time, int delay_msec);
	timespec timespec_diff (timespec *start, timespec *end);


	// INBOUND TELEMETRY - all volatile since it is updated by the run() thread
	volatile double devicePower;            // current output power in Watts
	volatile double deviceHeartRate;        // current heartrate in BPM
	volatile double deviceCadence;          // current cadence in RPM
	volatile double deviceSpeed;            // current speed in KPH
	volatile double deviceDistance;         // odometer in meters
	volatile int    deviceButtons;          // Button status
	volatile int    deviceStatus;           // Device status running, paused, disconnected
	volatile int    deviceSteering;            // Steering angle

	// OUTBOUND COMMANDS - all volatile since it is updated by the GUI thread
	volatile int mode;
	volatile double load;
	volatile double gradient;
	volatile double brakeCalibrationFactor;
	volatile double brakeCalibrationLoadRaw;
	volatile double powerScaleFactor;
	volatile double weight;

	// i/o message holder
	uint8_t buf[64];

	// device port
	LibUsb* usb2;                   // used for USB2 support

	// raw device utils
	int rawWrite(uint8_t* bytes, int size); // unix!!
	int rawRead(uint8_t* bytes, int size); // unix!!

public:
	volatile double rawPower;
	volatile double rawSpeed;
};

#endif // _GC_Fortius_h
