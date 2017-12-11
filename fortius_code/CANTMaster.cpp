/*
 * CANTMaster.cpp
 *
 * Copyright 2016 Douglas Pepelko
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.
 * The ASF licenses this file to You under the Apache License, Version 2.0
 * (the "License"); you may not use this file except in compliance with
 * the License.  You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include "ant.h"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <pthread.h>
#include <glog/logging.h>
#include <math.h>

#include "CANTMaster.h"

#define USER_ANTCHANNEL 0
#define DEVICE_ID	1147

#define FE_STATE_ASLEEP 1
#define FE_STATE_READY 2
#define FE_STATE_IN_USE 3
#define FE_STATE_FINISHED 4

#define EBS_UP		1
#define EBS_DOWN	2
#define EBS_LAP		3

// Indexes into message received from ANT
#define MESSAGE_ID_INDEX ((uint8_t) 1)
#define MESSAGE_RESULT_INDEX ((uint8_t) 2)

#define ANTPLUS_NETWORK_KEY  {0xB9, 0xA5, 0x21, 0xFB, 0xBD, 0x72, 0xC3, 0x45}
#define HRM_DEVICETYPE   0xF8	// 0x78 with top bit set to turn on pairing
#define FEC_DEVICETYPE   0x11	// 0x11 with top bit set to turn on pairing
#define HRM_RFFREQUENCY  0x39   //Set the RF frequency to channel 57 - 2.457GHz
#define HRM_MESSAGEPERIOD  8070    //Set the message period to 8070 counts specific for the HRM

#define GRAVITY 9.80665


#define COMMAND_STATUS_PASS		0
#define COMMAND_STATUS_FAILED		1
#define COMMAND_STATUS_NOT_SUPPORTED	2
#define COMMAND_STATUS_REJECTED		3
#define COMMAND_STATUS_PENDING		4


#pragma pack(push, 1)
typedef struct general_fe_s {
	uint8_t		data_page_number;	// 0x10 16
	uint8_t		equipment_type_bitfield;	// 25 == trainer  20 == eliptical? 21 == stationary bike?
	uint8_t		accumulated_time_quarter_seconds;	// in .25 sec increments. loops over at 64 seconds
	uint8_t		accumulated_distance;	// 1 meter increments.  loops at 256
	uint16_t	speed;			// .001 m/s 0-65.534 meters per second
	uint8_t		heart_rate;		// 0xFF for invalid?
	uint8_t		capabilities:4, state:4; // first four bits are capabilities table 7-9, second state set to 3 (in use)
} general_fe_t;
typedef struct general_settings_s {
	uint8_t		data_page_number;	// 0x11 17
	uint16_t	reserved;		// 0xFFFF
	uint8_t		cycle_length;		// units 0.01m  2.54 meters max (wheel circumference for trainer)
	int16_t		incline_percent;	// percentage -100 to 100. 0x7FFF indicates invalid
	uint8_t		resistance_fec;		// units 0.5% 0-100% or 0-200  0xFF invalid
	uint8_t		capabilites:4,state:4;	// set state to 3 (in use) same as state in general_fe
} general_settings_t;

typedef struct specific_trainer_s {
	uint8_t		data_page_number;	// 0x19 25 trainer specific data
	uint8_t		update_event_counter;	// increment with each update.
	uint8_t		instantaneous_cadence;	// crank cadence or 0xFF for invalid 0-254 rpm
	uint16_t	accumulated_power;	// 1 watt resolution, rollover at 65536
	uint16_t	instantaneous_power:12;	// 1 watt res 0-4096
	uint8_t		trainer_status:4;	// 0 no calib needed
	uint8_t		flags:4;		// 0
	uint8_t		fe_state:4;		// 3 in use
	/*
	0 Bicycle Power Calibration
		0 – Calibration complete/not required
		1 – Bicycle power measurement (i.e. Zero Offset) calibration required
	1 Resistance Calibration
		0 – Calibration complete/not required
		1 – Resistance calibration (i.e. Spin-Down Time) required
	2 User Configuration
		0 – Configuration complete/not required
		1 – User configuration required
	3 Reserved Reserved for future use. Set to 0
	*/
} specific_trainer_t;
typedef struct fe_capabilities_s {
	uint8_t		data_page_number;	// 0x36 page 54
	uint32_t	reserved	;	// 0xFFFFFFFF
	uint16_t	maximum_resistance;	// 0-65535 Newtons torque?
	uint8_t		capabilities;		// 0x7 == supports basic resistance, target power and simulation mode
} fe_capabilities_t;
typedef struct manufacturer_information_s {
	uint8_t	data_page_number;		// 0x50 page 80
	uint16_t reserved;			// 0xFFFF
	uint8_t hardware_rev;			// set by manufacturer
	uint16_t manufacturer_id;		// 0xFF development
	uint16_t model_id;			// self selected
} manufacturer_information_t;

typedef struct product_information_s {
	uint8_t data_page_number;		// 0x51 page 80
	uint8_t reserved;			// 0xFF
	uint16_t sw_revision;			//
	uint32_t serial_number;			//
} product_information_t;

#pragma pack(pop)

bool CANTMaster::send_request_page(uint8_t request_page)
{
	request_t	request;

	request.data_page_number = 0x46;
	request.slave_serial =   0x201;	// ?? 0x04030201;	// same as one we sent
	request.descriptor = 0xFFFF;
	request.response_cnt = 10;
	request.response_try = 0;
	request.requested_page_number = request_page;	// user configuration data 55
	request.command_type = 3;		// data page from slave

	printf("\nsend request page\n");
	hex_dump((uint8_t*)&request, sizeof(request));

	return send( (uint8_t*)&request);
}

// send message
bool CANTMaster::send_general_fe()
{
	double		heartrate_bpm;
	double		speed_kph;
	double		distance_meters;
	general_fe_t	general_fe;
	timespec	current_time;

	pthread_mutex_lock(&m_vars_mutex);
	heartrate_bpm = m_heartrate_bpm;
	speed_kph = m_speed_kph;
	distance_meters = m_distance_meters;
	pthread_mutex_unlock(&m_vars_mutex);	

	clock_gettime(CLOCK_MONOTONIC, &current_time);
	uint32_t current_seconds = to_seconds(&current_time);
	uint32_t elapsed_time_seconds = (current_seconds-m_start_seconds);

	general_fe.data_page_number = 0x10;
	general_fe.equipment_type_bitfield = 25; // 0x19 trainer
	general_fe.accumulated_time_quarter_seconds = elapsed_time_seconds * 4;
	general_fe.accumulated_distance = distance_meters;
	
	//printf("\nkph %f\n", speed_kph);
	double speed_kps = speed_kph/3600;			// convert kph to kps (60 minutes * 60 seconds)
	//printf("\nkps %f\n", speed_kps);
	double speed_mps = speed_kps * 1000; 		// convert kps to meters per second
	//printf("\nspeed mps %f\n", speed_mps);
	double speed_mmps = speed_mps * 1000;		// convert to mm per second.
	//printf("\nmm per second %f\n", speed_mmps);
	general_fe.speed = speed_mmps;			// speed millimeters per second
	general_fe.heart_rate = heartrate_bpm;
	general_fe.capabilities = 4;			// Distance traveled enabled
	general_fe.state = 0x3;				// in use; and lap toggle

	//printf("\ngeneral_fe\n");
	//hex_dump((uint8_t*)&general_fe, sizeof(general_fe));
	//printf("\n");

	return send( (uint8_t*)&general_fe);
}
bool CANTMaster::send_general_settings()
{
	double			slope;
	double			resistance;
	general_settings_t	general_settings;

	slope = m_fortius->getGradient();
	resistance = m_fortius->getLoadPercentage();


	general_settings.data_page_number = 0x11;
	general_settings.reserved = 0xFFFF;
	general_settings.cycle_length = 211;		// cm
	slope *= 100; 			// convert to 0.01% scale
	general_settings.incline_percent = slope;
	general_settings.resistance_fec = resistance*2;	// scale is 0.5% increments
	general_settings.capabilites = 0;
	general_settings.state = FE_STATE_IN_USE;		// 3 is in use

	//printf("\ngeneral settings\n");
	//hex_dump((uint8_t*)&general_settings, sizeof(general_settings));
	//printf("\n");

	return send( (uint8_t*)&general_settings);
}

bool CANTMaster::send_specific_trainer()
{
	static uint16_t		accumulated_power_watts=0;
	uint8_t			cadence_rpm;
	uint16_t		power_produced_watts;

	pthread_mutex_lock(&m_vars_mutex);
	power_produced_watts = m_power_produced_watts;
	cadence_rpm =m_cadence_rpm;
	pthread_mutex_unlock(&m_vars_mutex);


	accumulated_power_watts += power_produced_watts;

	specific_trainer_t	specific_trainer;

	specific_trainer.data_page_number = 0x19;
	specific_trainer.update_event_counter = m_sequence_number;
	specific_trainer.instantaneous_cadence = cadence_rpm;
	specific_trainer.accumulated_power = accumulated_power_watts;
	specific_trainer.instantaneous_power = power_produced_watts & 0xFFF; // should not need this
	specific_trainer.trainer_status = 0;	// no calibration needed
	specific_trainer.flags = 0;		// trainer operating at target power..
	specific_trainer.fe_state = FE_STATE_IN_USE;

	if(USER_CONFIG_STATE_EMPTY == m_user_config_state) { // no data in user config
		//printf("\nrequesting user config\n");
		// request it
		specific_trainer.trainer_status = 4;	// request user config
	}
	//printf("\nspecific_trainer accum watts 0x%x inst watts 0x%x\n", accumulated_power_watts, power_produced_watts);
	//hex_dump((uint8_t*)&specific_trainer, sizeof(specific_trainer));


	return send( (uint8_t*)&specific_trainer);
}

bool CANTMaster::send_fe_capabilities()
{
	fe_capabilities_t	fe_capabilities;

	fe_capabilities.data_page_number = 0x36;
	fe_capabilities.reserved = 0xFFFFFFFF;
	fe_capabilities.maximum_resistance = 1061;			// 1000 watts at 20mph requires 1061 Newton Meters
	fe_capabilities.capabilities = 0x7;		// 0x01 = basic resistance 0x02 = target power, and 0x03 simulation mode;

	//printf("\nfe_capabilites\n:");
	//hex_dump((uint8_t*)&fe_capabilities, sizeof(fe_capabilities));
	//printf("\n");

	return send( (uint8_t*)&fe_capabilities);
}
bool CANTMaster::send_manufacturer_information()
{
	manufacturer_information_t	manufacturer_information;

	manufacturer_information.data_page_number = 0x50;
	manufacturer_information.reserved = 0xFFFF;
	manufacturer_information.hardware_rev = 1;
	manufacturer_information.manufacturer_id = 0xFF;//WAHOO_FITNESS;	// 0xFF is development
	manufacturer_information.model_id = 0x01;		// first one

	//printf("\nmanufacturer_information\n");
	//hex_dump((uint8_t*)&manufacturer_information, sizeof(manufacturer_information));
	//printf("\n");
	return send( (uint8_t*)&manufacturer_information);
}
bool CANTMaster::send_product_information()
{
	product_information_t	product_information;

	product_information.data_page_number = 0x51;	// == 81
	product_information.reserved = 0xFF;
	product_information.sw_revision = 1;		//
	product_information.serial_number = 0x04030201;	//

	//printf("\nproduct_information\n");
	//hex_dump((uint8_t*)&product_information, sizeof(product_information));
	//printf("\n");
	return send( (uint8_t*)&product_information);
}

bool CANTMaster::send_command_status(){
	command_status_t command_status;

	command_status.data_page_number = PAGE_COMMAND_STATUS;

	command_status.last_rx_command_id = m_last_rx_command_id;
	command_status.sequence_number = m_sequence_number;
	command_status.command_status = m_command_status;
	memset(command_status.data, 0xFF, sizeof(command_status.data));

	return send( (uint8_t*)&command_status);
}

// received pages

bool CANTMaster::process_basic_resistance(basic_resistance_t*	basic_resistance)
{
	double		target_resistance_percentage;
	double		target_power_watts;

	if(PAGE_BASIC_RESISTANCE != basic_resistance->data_page_number) {
		return false;
	}
	target_resistance_percentage = basic_resistance->resistance_percentage;
	target_resistance_percentage /= 2;		// resistance comes in 0.5% increments so div by 2
	target_power_watts = 1000 * (target_resistance_percentage/100);

	m_fortius->setLoad(target_power_watts);

	return true;
}

bool CANTMaster::process_target_power(target_power_t*	target_power)
{
	double 		target_power_watts;

	if(PAGE_TARGET_POWER != target_power->data_page_number) {
		return false;
	}
	target_power_watts = target_power->target_power_quarter_watts;	// the message comes in 1/4 watt increments
	target_power_watts /= 4;					// so wait until it is in the double and div by 4

	
	pthread_mutex_lock(&m_vars_mutex);
	m_target_power_watts = target_power_watts;
	m_requested_mode = FT_ERGOMODE;
	pthread_mutex_unlock(&m_vars_mutex);

	printf("\nSET: target_power_watts %f\n", target_power_watts);

	return true;

}

bool CANTMaster::process_wind_resistance(wind_resistance_t* wind_resistance)
{
	double wind_speed_kph;
	double drafting_factor;
	double wind_resistance_coef;

	if(PAGE_WIND_RESISTANCE != wind_resistance->data_page_number) {
		return false;
	}
	if(wind_resistance->wind_resistance_coef == 0xFF){
		// set to default
		wind_resistance_coef = 0.51;
	}else{
		wind_resistance_coef = wind_resistance->wind_resistance_coef;
		wind_resistance_coef *= 0.01;  	// convert to kg/m
	}
	if(((uint8_t)wind_resistance->wind_speed) == 0xFF ){
		// default
		wind_speed_kph = 0;
	}else{
		wind_speed_kph = wind_resistance->wind_speed;
	}
	if(wind_resistance->drafting_factor == 0xFF){
		// default
		drafting_factor = 1.0;
	}else{
		drafting_factor = wind_resistance->drafting_factor;
		drafting_factor *= 0.01;	// convert to range 0-1
	}

	pthread_mutex_lock(&m_vars_mutex);
	m_wind_resistance_coef = wind_resistance_coef;
	m_wind_speed_kph = wind_speed_kph;
	m_drafting_factor = drafting_factor;
	pthread_mutex_unlock(&m_vars_mutex);
	printf("\nSET: wind resistance coef %f wind speed %f drafting factor %f\n", wind_resistance_coef, wind_speed_kph, drafting_factor);
	return true;
}

bool CANTMaster::process_track_resistance(track_resistance_t* track_resistance)
{
	double slope;
	double crr;

	if(PAGE_TRACK_RESISTANCE != track_resistance->data_page_number) {
		return false;
	}
	if(((uint16_t)track_resistance->slope) == 0xFFFF){
		/// default;
		slope = 0;
	}else{
		slope = track_resistance->slope;
		slope *= 0.01;
		slope -= 200;
		//printf("\nrequested slope %f\n", slope;
	}
	if(track_resistance->coefficient_of_rolling == 0xFF){
		// default
		crr = 0.004;
	}else{
		crr = track_resistance->coefficient_of_rolling;
		crr *= 0.00005;
	}

	pthread_mutex_lock(&m_vars_mutex);
	m_slope = slope;
	m_crr = crr;
	m_requested_mode = FT_SSMODE;	// we will calculate power
	pthread_mutex_unlock(&m_vars_mutex);

	/* TODO: resolve the Slope mode issue
	m_fortius->setMode(FT_SSMODE);
	m_fortius->setGradient(slope_double/100);
	*/
	printf("\nSET: slope %f crr %f\n", slope, crr);
	return true;
}

bool CANTMaster::process_user_configuration(user_configuration_t* user_configuration)
{
	double wheel_diameter_mm;
	double wheel_circumference_mm;
	double user_weight_kg;
	double bike_weight_kg;

	if(PAGE_USER_CONFIGURATION != user_configuration->data_page_number) {
		return false;
	}
	if(user_configuration->user_weight == 0xFFFF){
		// default
		user_weight_kg = 75;	// default from section 7.8.4.1 of D000001231_-_ANT+_Device_Profile_-_Fitness_Equipment_-_Rev_4.2.pdf
	}else{
		user_weight_kg = user_configuration->user_weight;		// in 0.01kg
		user_weight_kg *= 0.01;			// convert to kg
	}
	if(user_configuration->wheel_diameter_offset == 0xF){
		// default;
		wheel_diameter_mm = 0;	// start with no offset. we will add the actual diameter later
	}else{
		wheel_diameter_mm = user_configuration->wheel_diameter_offset;	// 1mm start with this offset and then add the actual size later
	}
	if(user_configuration->bike_weight == 0xFFF){
		// default;
		bike_weight_kg = 10.0;			// from ant FEC doc. section 7.8.4.1 says
	}else{
		bike_weight_kg = user_configuration->bike_weight;		// 0.05kg
		bike_weight_kg *= 0.05;			// convert to kg
	}
	if(user_configuration->wheel_diameter == 0xFF){
		// defaut
		wheel_diameter_mm += 700;		// 700mm
	}else{
		wheel_diameter_mm += (10 * user_configuration->wheel_diameter);		// 0.01m (extract and convert to mm)
	}
	wheel_circumference_mm = M_PI * wheel_diameter_mm;

	//uint8_t		gear_ratio;

	
	if( (user_weight_kg <= 0) || (user_weight_kg > 300) || (bike_weight_kg <= 0) || (bike_weight_kg >=100)){
		printf("\ninvalid settings: user weight kg %f bike weight %f wheel circumference mm %f\n", user_weight_kg, bike_weight_kg, wheel_circumference_mm);
		return false;
	}
	pthread_mutex_lock(&m_vars_mutex);
	m_user_weight_kg = user_weight_kg;
	m_bike_weight_kg = bike_weight_kg;
	m_wheel_circumference_mm = wheel_circumference_mm;
	m_user_config_state = USER_CONFIG_STATE_RX;
	pthread_mutex_unlock(&m_vars_mutex);
	printf("\nSET: user weight kg %f bike weight %f wheel circumference mm %f\n", user_weight_kg, bike_weight_kg, wheel_circumference_mm);

	return true;
}
bool CANTMaster::process_request(request_t* request)
{
	if(PAGE_REQUEST != request->data_page_number) {
		return false;
	}
	if(request->command_type != REQUEST_DATA_PAGE) {
		printf("\nerror, we can only handle REQUEST_DATA_PAGE\n");
		return false;
	}
	switch(request->requested_page_number) {

	case(PAGE_GENERAL_FE):
		return send_general_fe();
		break;
	case(PAGE_GENERAL_SETTINGS):
		return send_general_settings();
		break;
	case(PAGE_SPECIFIC_TRAINER):
		return send_specific_trainer();
		break;
	case(PAGE_FE_CAPABILITIES):
		return send_fe_capabilities();
		break;
	case(PAGE_MANUFACTURER_INFORMATION):
		return send_manufacturer_information();
		break;
	case(PAGE_PRODUCT_INFORMATION):
		return send_product_information();
		break;
	case(PAGE_COMMAND_STATUS):
		return send_command_status();
		break;
	default:
		printf("\nunsupported page %d\n", request->requested_page_number);
	}

	return true;
}

double CANTMaster::calc_power_required_watts(){
	double speed_kph;
	double weight_kg;

	// semi constant
	//double rho = 1.226;
	//double cd = 0.63;
	//double area_m = 0.509;
	double wind_resistance_coef;	// was a multiple of above values which was 0.39. Ant requests a drag coef of .51 for default
	double crr;			// ant wants default = 0.004.  I had default of 0.005;	

	// leave these alone
	double f_gravity;
	double f_rolling;
	double f_drag;			
	double slope;
	double speed_ms;
	double power_watts;
	double drafting_factor;

	// my user specs...get them
	pthread_mutex_lock(&m_vars_mutex);
	weight_kg = m_user_weight_kg + m_bike_weight_kg;
	wind_resistance_coef = m_wind_resistance_coef;
	crr = m_crr;
	drafting_factor = m_drafting_factor;
	speed_kph = m_speed_kph + m_wind_speed_kph;
	slope = m_slope/100;				// slope is a percent...change it to a range  -1..1
	pthread_mutex_unlock(&m_vars_mutex);

	f_gravity = GRAVITY * sin(atan(slope)) * weight_kg;

	f_rolling = GRAVITY * cos(atan(slope)) * weight_kg * crr;

	speed_ms = (speed_kph/3600)*1000;				// divide hours to seconds..and muliply km to meters
	//f_drag = 0.5 * cd * area_m * rho * speed_ms * speed_ms;	// old formula
	f_drag = 0.5 * wind_resistance_coef * (speed_ms * speed_ms);
	f_drag *= drafting_factor;	// scales wind resistance from 0-100%

	power_watts = (f_gravity + f_rolling + f_drag) * speed_ms;

	return power_watts;
}


void CANTMaster::hex_dump(uint8_t* data, int data_size)
{
	int cnt;

	for(cnt=0; cnt < data_size; cnt++) {
		printf("\n%x ", data[cnt]);
		if(cnt%8==7) {
			printf("\n");
		}
	}
}

CANTMaster* CANTMaster::this_ptr = NULL;

bool CANTMaster::send(uint8_t* data){
	m_sequence_number++;

	return ANT_SendBroadcastData(m_channel_number, data);
}

void* CANTMaster::mainloop_helper(void *context)
{
	CANTMaster* local_this_ptr = static_cast<CANTMaster*>(context);
	return local_this_ptr->mainloop();
}
CANTMaster::CANTMaster()
{

	m_last_rx_command_id = 0xFF;	// no control page received yet
	m_sequence_number = 0xFF;	// no control page rx
	m_command_status = 0xFF;	// no control page rx
	m_exit_flag = false;
	this_ptr = this;
	m_channel_open = false;
	m_retry_count = 0;
	m_fortius = NULL;
	m_channel_number = USER_ANTCHANNEL;
	m_device_id = DEVICE_ID;
	m_slope = 0;
	m_speed_kph = 0;
	m_requested_mode = FT_ERGOMODE;
	pthread_mutex_init(&m_vars_mutex, NULL);
	// set some defaults
	m_target_power_watts = 100;	// watts
	m_user_weight_kg = 93;	// 205 lbs
	m_bike_weight_kg = 8.6;	// 19 lbs
	m_wheel_circumference_mm = 2105;	// 700x25
	m_user_config_state = USER_CONFIG_STATE_EMPTY;	// no data in user config

	m_wind_resistance_coef = 0.51;// road  bike hoods 
	m_wind_speed_kph = 0;	
	m_drafting_factor = 1.0;

}

CANTMaster::~CANTMaster()
{
	ANT_UnassignAllResponseFunctions();
	ANT_Nap(2000);
	ANT_Close();
}
bool CANTMaster::init(Fortius* fortius)
{
	m_fortius = fortius;
	// set default load

	//*
	m_fortius->setMode(FT_ERGOMODE);
	m_fortius->setLoad(50);
	/*/
	m_fortius->setMode(FT_SSMODE);
	m_fortius->setGradient(1);
	//*/
	m_retry_count=0;
	if(false == ANT_Init(0,57600)) {
		printf("\nFailed ANT_Init\n");
		return FALSE;
	}
	printf("\nAnt Assign Response Function\n");
	ANT_AssignResponseFunction(CANTMaster::response_callback, m_response_buffer);
	printf("\nAnt Assign Event Funcion\n");
	ANT_AssignChannelEventFunction(m_channel_number,CANTMaster::channel_callback, m_channel_buffer);
	printf("\nResetSystem\n");
	if(false == ANT_ResetSystem()) {
		printf("\nFailed ANT_ResetSystem\n");
		return FALSE;
	}
	printf("\nNap\n");
	ANT_Nap(2000);
	printf("\nDone napping\n");
	return TRUE;
}
bool CANTMaster::start()
{

	pthread_create(&m_pthread,NULL,&CANTMaster::mainloop_helper,this);

	return TRUE;
}
bool CANTMaster::join()
{
	void *    pthread_return;

	pthread_join(m_pthread, &pthread_return);
	return TRUE;
}
bool CANTMaster::kill()
{
	pthread_cancel(m_pthread);
	return TRUE;
}

double CANTMaster::to_seconds(struct timespec* ts)
{

	return (double)ts->tv_sec + (double)ts->tv_nsec / 1000000000.0;
}

bool CANTMaster::stop()
{
	ANT_CloseChannel(m_channel_number);
	ANT_UnassignAllResponseFunctions();
	ANT_UnAssignChannel(m_channel_number);
	m_exit_flag = true;
	ANT_Close();

	return TRUE;
}


void* CANTMaster::mainloop(void)
{
	uint8_t		count_mod_8;
	uint8_t		enter_button_state = EBS_UP;
	bool		toggle = true;
	uint32_t	count = 0;
	uint8_t		network_key[8] = ANTPLUS_NETWORK_KEY;
	uint8_t		requested_mode;
	double		target_power_watts;
	double		power_produced_watts;
	double		speed_kph;
	double		heartrate_bpm;
	double		cadence_rpm;
	double		distance_meters;
	double		slope;
	double		power_required_watts;
	int		buttons;
	int		steering;
	int		status;
	timespec	start_time;


	m_channel_open = FALSE;
	//STEP1 ANT_SetNetworkKey
	if(false == ANT_SetNetworkKey(0, network_key)) {
		printf("\nmainloop: failed to set network key\n");
		return NULL;
	}
	while(FALSE == m_channel_open && false == m_exit_flag) {
		printf("\nwait for channel open\n");
		usleep(25000);	// wait 250ms (1/4 second)
		m_retry_count++;
		if(25==m_retry_count) {	// 10 seconds
			stop();
			return NULL;
		}
	}
	printf("\nchannel open\n");
	// start time
	clock_gettime(CLOCK_MONOTONIC, &start_time);
	m_start_seconds = to_seconds(&start_time);
	while(false == m_exit_flag) {
		if(m_channel_open == FALSE) {
			stop();
			return NULL;
		}

		// read stats from the Fortius
		m_fortius->getTelemetry(power_produced_watts, heartrate_bpm, cadence_rpm, speed_kph, distance_meters, buttons, steering, status);


		// slope was sent in on track_resistance page
		pthread_mutex_lock(&m_vars_mutex);
		// get mode and requested load
		target_power_watts = m_target_power_watts;
		requested_mode = m_requested_mode;
		slope = m_slope;
		
		// set everything else
		m_power_produced_watts = power_produced_watts;
		m_heartrate_bpm = heartrate_bpm;
		m_cadence_rpm = cadence_rpm;
		m_speed_kph = speed_kph;
		m_distance_meters = distance_meters;
		m_buttons = buttons;
		pthread_mutex_unlock(&m_vars_mutex);

		if(requested_mode == FT_ERGOMODE){
			power_required_watts = target_power_watts;
		}else if(requested_mode == FT_SSMODE){
			power_required_watts = calc_power_required_watts();
		}

		m_fortius->setMode(FT_ERGOMODE);
		m_fortius->setLoad(power_required_watts);


		printf("\rpower mk %fw, cadence %f, speed %fmph, power nd %fw slp %f md %d",
			   power_produced_watts,
			   cadence_rpm,
			   m_speed_kph*0.62137100000000001,
			   power_required_watts,
			   slope,
			   m_requested_mode);

		fflush(stdout);

		/* debug
		power_produced_watts = 403;
		cadence_rpm = 96;
		m_speed_kph = 21;
		distance_meters += 23;	// 23 meters every 1/4 second
		*/

		// enter button is lap button
		if(enter_button_state == EBS_UP) {
			if(0 != (buttons & FT_ENTER)) {
				enter_button_state = EBS_DOWN;
			}
		} else if(enter_button_state == EBS_DOWN) {
			if(0 == (buttons & FT_ENTER)) {
				enter_button_state = EBS_LAP;
			}
		}

		count_mod_8 = count%8;

		if(count == 64 || count == 65) {
			if(toggle == true) {
				// send common page 80
				if(false == send_manufacturer_information()) {
					printf("\nfailed to send common data page 80\n");
					// TODO: reset connection?
				}
				toggle = false;
			} else {
				// send common page 81
				if(false == send_product_information()) {
					printf("\nfailed to send common data page 81\n");
					// TODO: reset
				}
				toggle = true;
			}
		} else if((count_mod_8 == 0) || (count_mod_8 == 1) || (count_mod_8 == 4) || (count_mod_8 == 5)) {
			// send page 0x10 16
			if(false == send_general_fe()) {
				printf("\nfailed to send general fe 0x10 (16)\n");
				// TODO reset
			}
		} else if( (count_mod_8 == 2) || (count_mod_8 == 6)) {
			// send FE specific page
			if(false == send_specific_trainer()) {	//uint8_t cadence, uint16_t power)){
				printf("\nfailed to send specifi trainer \n");
				// TODO: reset
			}
		} else if((count_mod_8 == 3) || (count_mod_8 == 7)) {
			// send page 17
			if(false == send_general_settings()) {	//  (int16_t incline_percent, uint8_t resistance)
				printf("\nfailed to send general settings 0x11 (17)\n");
				// TODO: reset
			}
		} else {
			printf("\nunaccounted for count %d countmod8 %d\n", count,  count%8);
		}

		count = (count+1)%66;	// inc and loop back every 66

		usleep(250000);	// sleep 1/4 second

	}
	return NULL;
}

int8_t CANTMaster::channel_callback(uint8_t channel_number, uint8_t event)
{
	return this_ptr->channel_handler(channel_number, event);
}

int8_t CANTMaster::channel_handler(uint8_t channel_number, uint8_t event)
{
	//printf("\nRx Channel Event:"<<(int)event<<",channel:"<<(int)channel_number;

	switch(event) {
	case EVENT_RX_FLAG_ACKNOWLEDGED:
	case EVENT_RX_FLAG_BURST_PACKET:
	case EVENT_RX_FLAG_BROADCAST:
		printf("\nRx Channel FLAG %d, event %d\n", channel_number, event);
		break;

	case EVENT_RX_ACKNOWLEDGED:
	case EVENT_RX_BURST_PACKET:
	case EVENT_RX_BROADCAST:
		switch(m_channel_buffer[1]) {
		m_last_rx_command_id = m_channel_buffer[1];
		m_command_status = COMMAND_STATUS_PASS;			// default to this, then switch to an error if needed
		case(PAGE_BASIC_RESISTANCE):
			if(false == process_basic_resistance((basic_resistance_t*)&m_channel_buffer[1])) {
				m_command_status = COMMAND_STATUS_FAILED;
				printf("\nfailed to process basic resistance message\n");
			}
			break;
		case(PAGE_TARGET_POWER):
			if(false == process_target_power((target_power_t*)&m_channel_buffer[1])) {
				m_command_status = COMMAND_STATUS_FAILED;
				printf("\nfailed to process target power message\n");
			}
			break;
		case(PAGE_WIND_RESISTANCE):
			if(false == process_wind_resistance((wind_resistance_t*)&m_channel_buffer[1])) {
				m_command_status = COMMAND_STATUS_FAILED;
				printf("\nfailed to process wind resistance message\n");
			}
			break;
		case(PAGE_TRACK_RESISTANCE):
			if(false == process_track_resistance((track_resistance_t*)&m_channel_buffer[1])) {
				m_command_status = COMMAND_STATUS_FAILED;
				printf("\nfailed to process track resistance message\n");
			}
			break;
		case(PAGE_USER_CONFIGURATION):
			if(false == process_user_configuration((user_configuration_t*)&m_channel_buffer[1])) {
				m_command_status = COMMAND_STATUS_FAILED;
				printf("\nfailed to process user configuration message\n");
			}
			break;
		case(PAGE_REQUEST):
			if(false == process_request((request_t*)&m_channel_buffer[1])) {
				m_command_status = COMMAND_STATUS_FAILED;
				printf("\nfailed to process page request message\n");
			}
			break;
		default:
			printf("\nunknown message\n");
			m_command_status = COMMAND_STATUS_NOT_SUPPORTED;
			hex_dump((uint8_t*)&m_channel_buffer[0], sizeof(m_channel_buffer));
			break;
		}

		break;

	case EVENT_RX_EXT_ACKNOWLEDGED:
	case EVENT_RX_EXT_BURST_PACKET:
	case EVENT_RX_EXT_BROADCAST:
		printf("\nRx Channel EXT %d, event %d\n", channel_number, event);
		break;
	case EVENT_TX:
		// ignore tx events
		break;
	default:
		printf("\nRx Channel %d, event %d unknown\n", channel_number, event);
	}

	return TRUE;
}
int8_t CANTMaster::response_callback(uint8_t channel_number, uint8_t message_id)
{
	return this_ptr->response_handler(channel_number, message_id);
}
int8_t CANTMaster::response_handler(uint8_t channel_number, uint8_t message_id)
{
	if(channel_number != m_channel_number) {
		printf("\nInvalid channel %d message received.  Expected messages for channel %d.\n", channel_number, m_channel_number);
		return FALSE;
	}

	switch(message_id) {
	case MESG_RESPONSE_EVENT_ID: {
		fec_init(m_response_buffer[MESSAGE_ID_INDEX],m_response_buffer[MESSAGE_RESULT_INDEX]);
		break;
	}
	default:
		break;
	}
	return TRUE;
}

bool CANTMaster::fec_init(uint8_t message_id,uint8_t result)
{
	uint8_t network_key[8] = ANTPLUS_NETWORK_KEY;

	if(RESPONSE_NO_ERROR!=result) {
		return FALSE;
	}

	switch(message_id) {
	//step 1 setneworkkey
	case MESG_NETWORK_KEY_ID:
		if(false == ANT_AssignChannel(0/*channel*/, 0x10/*master*/, 0/*network_num*/)) {
			printf("\nFailed ANT_AssignChannel\n");
		}
		break;

	//step2 assignchannelid
	case MESG_ASSIGN_CHANNEL_ID:
		ANT_SetChannelId(0, m_device_id, FEC_DEVICETYPE, 0x05);
		break;

	//step3 ANT_SetChannelId
	case MESG_CHANNEL_ID_ID:
		ANT_SetChannelRFFreq(0, HRM_RFFREQUENCY/*USER_RADIOFREQ*/);
		break;

	//step4 ANT_SetChannelRFFreq
	case MESG_CHANNEL_RADIO_FREQ_ID:
		ANT_SetChannelPeriod(0, 8182);//HRM_MESSAGEPERIOD);
		break;

	//step5 ANT_SetChannelPeriod
	case  MESG_CHANNEL_MESG_PERIOD_ID:
		ANT_OpenChannel(0);
		break;

	//step6 ANT_OpenChannel
	case MESG_OPEN_CHANNEL_ID:
		//we success do it!
		m_channel_open = TRUE;
		break;

	default:
		printf("\nUnknown MESG type %d.", message_id);
		m_retry_count++;
		if(m_retry_count>=10) {
			printf("\nFailed after %d attempts.\n", m_retry_count);
			stop();
		} else {
			printf("\nRetrying again %d.\n", m_retry_count);
			ANT_SetNetworkKey(0, network_key);
		}
		break;
	}
	return TRUE;
}

