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
typedef struct common_data_page_80_s {
	uint8_t	data_page_number;		// 0x50 page 80
	uint16_t reserved;			// 0xFFFF
	uint8_t hardware_rev;			// set by manufacturer
	uint16_t manufacturer_id;		// 0xFF development
	uint16_t model_id;			// self selected
} common_data_page_80_t;

typedef struct common_data_page_81_s {
	uint8_t data_page_number;		// 0x51 page 80
	uint8_t reserved;			// 0xFF
	uint16_t sw_revision;			//
	uint32_t serial_number;			//
} common_data_page_81_t;

#pragma pack(pop)

bool CANTMaster::send_request_page(uint8_t request_page){
	request_t	request;

	request.data_page_number = 0x46;
	request.slave_serial =   0x201;	// ?? 0x04030201;	// same as one we sent
	request.descriptor = 0xFFFF;
	request.response_cnt = 10;
	request.response_try = 0;
	request.requested_page_number = request_page;	// user configuration data 55
	request.command_type = 3;		// data page from slave
	
	printf("send request page\n");
	hex_dump((uint8_t*)&request, sizeof(request));

	return ANT_SendBroadcastData(m_channel_number, (uint8_t*)&request);
}

// send message
bool CANTMaster::send_general_fe(double accumulated_time_seconds, uint8_t  accumulated_distance_m, uint16_t speed_kph, uint8_t heartrate_bpm, uint8_t lap)
{
	general_fe_t	general_fe;
	static uint8_t	accumulated_lap=0;

	accumulated_lap ^= lap;

	general_fe.data_page_number = 0x10;
	general_fe.equipment_type_bitfield = 25; // 0x19 trainer
	general_fe.accumulated_time_quarter_seconds = accumulated_time_seconds*4;
	general_fe.accumulated_distance = accumulated_distance_m;
	double speed_kph_double = speed_kph;		// get it out of an int and into a double
	//printf("kph %f\n", speed_kph);
	double speed_kps = speed_kph_double/3600;			// convert kph to kps (60 minutes * 60 seconds)
	//printf("kps %f\n", speed_kps);
	double speed_mps = speed_kps * 1000; 		// convert kps to meters per second
	//printf("speed mps %f\n", speed_mps);
	double speed_mmps = speed_mps * 1000;		// convert to mm per second.
	//printf("mm per second %f\n", speed_mmps);
	general_fe.speed = speed_mmps;			// speed millimeters per second
	general_fe.heart_rate = heartrate_bpm;
	general_fe.capabilities = 4;			// Distance traveled enabled
	general_fe.state = 0x3 | accumulated_lap<<3;	// in use; and lap toggle

	//printf("general_fe\n");
	//hex_dump((uint8_t*)&general_fe, sizeof(general_fe));
	//printf("\n");

	return ANT_SendBroadcastData(m_channel_number, (uint8_t*)&general_fe);
}
bool CANTMaster::send_general_settings(double incline_percent_double, uint8_t resistance)
{
	general_settings_t	general_settings;

	general_settings.data_page_number = 0x11;
	general_settings.reserved = 0xFFFF;
	general_settings.cycle_length = 211;		// cm
	incline_percent_double *= 100; 			// convert to 0.01% scale
	general_settings.incline_percent = incline_percent_double;
	general_settings.resistance_fec = resistance*2;	// scale is 0.5% increments 
	general_settings.capabilites = 0;
	general_settings.state = FE_STATE_IN_USE;		// 3 is in use

	//printf("general settings\n");
	//hex_dump((uint8_t*)&general_settings, sizeof(general_settings));
	//printf("\n");

	return ANT_SendBroadcastData(m_channel_number, (uint8_t*)&general_settings);
}

bool CANTMaster::send_specific_trainer(uint8_t cadence_rpm, uint16_t power_watts)
{
	static uint8_t		event_counter=0;
	static uint16_t		accumulated_power_watts=0;

	accumulated_power_watts += power_watts;

	specific_trainer_t	specific_trainer;

	specific_trainer.data_page_number = 0x19;
	specific_trainer.update_event_counter = event_counter;
	specific_trainer.instantaneous_cadence = cadence_rpm;
	specific_trainer.accumulated_power = accumulated_power_watts;
	specific_trainer.instantaneous_power = power_watts & 0xFFF; // should not need this
	specific_trainer.trainer_status = 0;	// no calibration needed
	specific_trainer.flags = 0;		// trainer operating at target power..
	specific_trainer.fe_state = FE_STATE_IN_USE;

	if(USER_CONFIG_STATE_EMPTY == m_user_config_state){ // no data in user config
		//printf("requesting user config\n");
		// request it
		specific_trainer.trainer_status = 4;	// request user config
	} 
	//printf("specific_trainer accum watts 0x%x inst watts 0x%x\n", accumulated_power_watts, power_watts);
	//hex_dump((uint8_t*)&specific_trainer, sizeof(specific_trainer));

	event_counter++;

	return ANT_SendBroadcastData(m_channel_number, (uint8_t*)&specific_trainer);
}

bool CANTMaster::send_fe_capabilities()
{
	fe_capabilities_t	fe_capabilities;

	fe_capabilities.data_page_number = 0x36;
	fe_capabilities.reserved = 0xFFFFFFFF;
	fe_capabilities.maximum_resistance = 1061;			// 1000 watts at 20mph requires 1061 Newton Meters
	fe_capabilities.capabilities = 0x7;		// 0x01 = basic resistance 0x02 = target power, and 0x03 simulation mode;

	printf("fe_capabilites\n:");
	hex_dump((uint8_t*)&fe_capabilities, sizeof(fe_capabilities));
	printf("\n");

	return ANT_SendBroadcastData(m_channel_number, (uint8_t*)&fe_capabilities);
}
bool CANTMaster::send_common_data_page_80()
{
	common_data_page_80_t	common_data_page_80;

	common_data_page_80.data_page_number = 0x50;
	common_data_page_80.reserved = 0xFFFF;
	common_data_page_80.hardware_rev = 1;
	common_data_page_80.manufacturer_id = 0xFF;//WAHOO_FITNESS;	// 0xFF is development
	common_data_page_80.model_id = 0x01;		// first one

	//printf("\ncommon_data_page_80\n");
	//hex_dump((uint8_t*)&common_data_page_80, sizeof(common_data_page_80));
	//printf("\n");
	return ANT_SendBroadcastData(m_channel_number, (uint8_t*)&common_data_page_80);
}
bool CANTMaster::send_common_data_page_81()
{
	common_data_page_81_t	common_data_page_81;

	common_data_page_81.data_page_number = 0x51;	// == 81
	common_data_page_81.reserved = 0xFF;
	common_data_page_81.sw_revision = 1;		//
	common_data_page_81.serial_number = 0x04030201;	//

	//printf("common_data_page_81\n");
	//hex_dump((uint8_t*)&common_data_page_81, sizeof(common_data_page_81));
	//printf("\n");
	return ANT_SendBroadcastData(m_channel_number, (uint8_t*)&common_data_page_81);
}




// received pages

bool CANTMaster::process_basic_resistance(basic_resistance_t*	basic_resistance)
{
	double		output_percentage;
	double		output_target_power;

	if(PAGE_BASIC_RESISTANCE != basic_resistance->data_page_number) {
		return false;
	}
	output_percentage = basic_resistance->resistance_percentage;
	output_percentage /= 2;		// resistance comes in 0.5% increments so div by 2
	output_target_power = 1000 * (output_percentage/100);

	m_fortius->setLoad(output_target_power);

	return true;
}

bool CANTMaster::process_target_power(target_power_t*	target_power)
{
	double 		output_target_power;

	if(PAGE_TARGET_POWER != target_power->data_page_number) {
		return false;
	}
	output_target_power = target_power->target_power_quarter_watts;	// the message comes in 1/4 watt increments
	output_target_power /= 4;					// so wait until it is in the double and div by 4

	//printf("target_power\n");
	m_fortius->setLoad(output_target_power);

	return true;

}
bool CANTMaster::process_track_resistance(track_resistance_t* track_resistance){
	if(PAGE_TRACK_RESISTANCE != track_resistance->data_page_number){
		return false;
	}
	double slope_fraction = track_resistance->slope;
	slope_fraction *= 0.01; 	
	slope_fraction -= 200;
	slope_fraction *= 2;	// zwift weirdness
	//printf("requested slope %f\n", slope_fraction);

	pthread_mutex_lock(&m_vars_mutex);
        this->m_slope = slope_fraction;
        pthread_mutex_unlock(&m_vars_mutex);
	/*
	m_fortius->setMode(FT_SSMODE);
	m_fortius->setGradient(slope_double/100);
	*/
	return true;
}	
bool CANTMaster::process_request(request_t* request){
	if(PAGE_REQUEST != request->data_page_number){
		return false;
	}
	printf("------page request !!! %d command_type %d\n", request->requested_page_number, request->command_type);
	if(PAGE_COMMAND_STATUS == request->requested_page_number){
		printf("command status requested\n");
	}
	return true;
}

bool CANTMaster::process_user_configuration(user_configuration_t* user_configuration){
	double wheel_diameter_mm;
	if(PAGE_USER_CONFIGURATION != user_configuration->data_page_number){
		return false;
	}
	m_user_weight_kg = user_configuration->user_weight;		// in 0.01kg
	m_user_weight_kg *= 100;			// convert to kg
	wheel_diameter_mm = user_configuration->wheel_diameter_offset;	// 1mm
	m_bike_weight_kg = user_configuration->bike_weight;		// 0.05kg
	m_bike_weight_kg *= 20;			// convert to kg
	wheel_diameter_mm = (10 * user_configuration->wheel_diameter);		// 0.01m (extract and convert to mm)
	m_wheel_circumference_mm = M_PI * wheel_diameter_mm;
	//uint8_t		gear_ratio;	
	printf("user weight kg %f bike weight %f wheel circumference mm %f\n", m_user_weight_kg, m_bike_weight_kg, m_wheel_circumference_mm);

	m_user_config_state = USER_CONFIG_STATE_RX;	
	return true;
}

double CANTMaster::calc_power_required_watts(double velocity_kph, double grade_percent){

	// my user specs...get them 
	double weight_lbs = 225;

	// semi constant
	double rho = 1.226;
	double cd = 0.63;
	double area_m = 0.509;
	double crr = 0.005;	// typical?

	// leave these alone
	double f_gravity;
	double f_rolling;
	double f_drag;
	double weight_kg = weight_lbs * 0.453592;
	double grade = grade_percent/100;
	double velocity_ms = (velocity_kph/3600)*1000;	// divide hours to seconds..and muliply km to meters
	double power_watts;
	

	f_gravity = GRAVITY * sin(atan(grade)) * weight_kg;

	f_rolling = GRAVITY * cos(atan(grade)) * weight_kg * crr;

	f_drag = 0.5 * cd * area_m * rho * velocity_ms * velocity_ms;

	power_watts = (f_gravity + f_rolling + f_drag) * velocity_ms;

	return power_watts;
}


void CANTMaster::hex_dump(uint8_t* data, int data_size)
{
	int cnt;

	for(cnt=0; cnt < data_size; cnt++) {
		printf("%x ", data[cnt]);
		if(cnt%8==7) {
			printf("\n");
		}
	}
}

CANTMaster* CANTMaster::this_ptr = NULL;
void* CANTMaster::mainloop_helper(void *context){
	CANTMaster* local_this_ptr = static_cast<CANTMaster*>(context);
	return local_this_ptr->mainloop();
}
CANTMaster::CANTMaster(){
	m_exit_flag = false;
	this_ptr = this;
	m_channel_open = false;
	m_retry_count = 0;
	m_fortius = NULL;
	m_channel_number = USER_ANTCHANNEL;
	m_slope = 0;
	m_speed_kph = 0;
	pthread_mutex_init(&m_vars_mutex, NULL);

	m_user_weight_kg = 0;
	m_bike_weight_kg = 0;
	m_wheel_circumference_mm = 0;
	m_user_config_state = USER_CONFIG_STATE_EMPTY;	// no data in user config
	m_device_id = DEVICE_ID;
}
	
CANTMaster::~CANTMaster(){
	ANT_UnassignAllResponseFunctions();
	ANT_Nap(2000);
	ANT_Close();
}
bool CANTMaster::init(Fortius* fortius){
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
		printf("Failed ANT_Init\n");
		return FALSE;
	}
	printf("Ant Assign Response Function\n");
	ANT_AssignResponseFunction(CANTMaster::response_callback, m_response_buffer);
	printf("Ant Assign Event Funcion\n");
	ANT_AssignChannelEventFunction(m_channel_number,CANTMaster::channel_callback, m_channel_buffer);
	printf("ResetSystem\n");
	if(false == ANT_ResetSystem()) {
		printf("Failed ANT_ResetSystem\n");
		return FALSE;
	}
	printf("Nap\n");
	ANT_Nap(2000);
	printf("Done napping\n");
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
	uint8_t		lap = 0;
	uint8_t		enter_button_state = EBS_UP;
	bool		toggle = true;
	uint32_t	count = 0;
	uint8_t		network_key[8] = ANTPLUS_NETWORK_KEY;
	double 		gradient;
	double		load_percentage;
	double		power_watts;
	double		heartrate_bpm;
	double		cadence_rpm;
	double		distance_meters;
	double		slope_fraction;
	double		power_required_watts;
	int		buttons;
	int		steering;
	int		status;
	timespec	start_time;
	timespec	current_time;


	m_channel_open = FALSE;
	//STEP1 ANT_SetNetworkKey
	if(false == ANT_SetNetworkKey(0, network_key)) {
		printf("mainloop: failed to set network key\n");
		return NULL;
	}
	while(FALSE == m_channel_open && false == m_exit_flag) {
		printf("wait for channel open\n");
		usleep(25000);	// wait 250ms (1/4 second)
		m_retry_count++;
		if(25==m_retry_count) {	// 10 seconds
			stop();
			return NULL;
		}
	}
	printf("channel open\n");
	// start time
	clock_gettime(CLOCK_MONOTONIC, &start_time);
	double start_seconds = to_seconds(&start_time);
	while(false == m_exit_flag) {
		if(m_channel_open == FALSE) {
			stop();
			return NULL;
		}

		// read stats from the Fortius
		m_fortius->getTelemetry(power_watts, heartrate_bpm, cadence_rpm, m_speed_kph, distance_meters, buttons, steering, status);


		// slope was sent in on track_resistance page 	
		pthread_mutex_lock(&m_vars_mutex);
		slope_fraction = m_slope;
		pthread_mutex_unlock(&m_vars_mutex);
		power_required_watts = calc_power_required_watts(m_speed_kph, slope_fraction);
		m_fortius->setMode(FT_ERGOMODE);
		m_fortius->setLoad(power_required_watts);

		
		printf("\rpower mk %fw, cadence %f, speed %fmph, power nd %fw slp %f",
			 power_watts,
			 cadence_rpm,
			 m_speed_kph*0.62137100000000001,
			 power_required_watts,
			slope_fraction);

		fflush(stdout);

		/* debug
		power_watts = 403;
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
				if(false == send_common_data_page_80()) {
					printf("failed to send common data page 80\n");
					// TODO: reset connection?
				}
				toggle = false;
			} else {
				// send common page 81
				if(false == send_common_data_page_81()) {
					printf("failed to send common data page 81\n");
					// TODO: reset
				}
				toggle = true;
			}
		} else if((count_mod_8 == 0) || (count_mod_8 == 1) || (count_mod_8 == 4) || (count_mod_8 == 5)) {
			clock_gettime(CLOCK_MONOTONIC, &current_time);
			double current_seconds = to_seconds(&current_time);
			if(enter_button_state == EBS_LAP) {
				lap = 1;
				enter_button_state = EBS_UP;
			} else {
				lap = 0;
			}
			// send page 0x10 16
			if(false == send_general_fe((current_seconds-start_seconds), distance_meters, m_speed_kph, heartrate_bpm, lap)) {
				printf("failed to send general fe 0x10 (16)\n");
				// TODO reset
			}
		} else if( (count_mod_8 == 2) || (count_mod_8 == 6)) {
			// send FE specific page
			if(false == send_specific_trainer(cadence_rpm, power_watts)) {	//uint8_t cadence, uint16_t power)){
				printf("failed to send specifi trainer \n");
				// TODO: reset
			}
		} else if((count_mod_8 == 3) || (count_mod_8 == 7)) {
			// send page 17
			gradient = m_fortius->getGradient();
			load_percentage = m_fortius->getLoadPercentage();
			if(false == send_general_settings(gradient, load_percentage)) {	//  (int16_t incline_percent, uint8_t resistance)
				printf("failed to send general settings 0x11 (17)\n");
				// TODO: reset
			}
		} else {
			printf("unaccounted for count %d countmod8 %d\n", count,  count%8);
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
	//printf("Rx Channel Event:"<<(int)event<<",channel:"<<(int)channel_number;

	switch(event) {
	case EVENT_RX_FLAG_ACKNOWLEDGED:
	case EVENT_RX_FLAG_BURST_PACKET:
	case EVENT_RX_FLAG_BROADCAST:
		printf("Rx Channel FLAG %d, event %d\n", channel_number, event);
		break;

	case EVENT_RX_ACKNOWLEDGED:
	case EVENT_RX_BURST_PACKET:
	case EVENT_RX_BROADCAST:
		printf("\nEvent rx broadcast\n");
		hex_dump(m_channel_buffer, 9);
		switch(m_channel_buffer[1]){
			case(PAGE_TARGET_POWER):
				if(false == process_target_power((target_power_t*)&m_channel_buffer[1])){
					printf("failed to process target power message\n");
				}
				break;
			case(PAGE_BASIC_RESISTANCE):
				if(false == process_basic_resistance((basic_resistance_t*)&m_channel_buffer[1])){
					printf("failed to process basic resistance message\n");
				}
				break;
			case(PAGE_TRACK_RESISTANCE):
				if(false == process_track_resistance((track_resistance_t*)&m_channel_buffer[1])){
					printf("failed to process track resistance message\n");
				}
				break;
			case(PAGE_USER_CONFIGURATION):
				if(false == process_user_configuration((user_configuration_t*)&m_channel_buffer[1])){
					printf("failed to process user configuration message\n");
				}
				break;
			default:
				printf("unknown message\n");
				hex_dump((uint8_t*)&m_channel_buffer[0], sizeof(m_channel_buffer));
				break;
		}

		break;

	case EVENT_RX_EXT_ACKNOWLEDGED:
	case EVENT_RX_EXT_BURST_PACKET:
	case EVENT_RX_EXT_BROADCAST:
		printf("Rx Channel EXT %d, event %d\n", channel_number, event);
		break;
	case EVENT_TX:
		// ignore tx events
		break;
	default:
		printf("Rx Channel %d, event %d unknown\n", channel_number, event);
	}

	return TRUE;
}
int8_t CANTMaster::response_callback(uint8_t channel_number, uint8_t message_id)
{
	return this_ptr->response_handler(channel_number, message_id);
}
int8_t CANTMaster::response_handler(uint8_t channel_number, uint8_t message_id)
{
	if(channel_number != m_channel_number){
		printf("Invalid channel %d message received.  Expected messages for channel %d.\n", channel_number, m_channel_number);
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
			printf("Failed ANT_AssignChannel\n");
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
		printf("Unknown MESG type %d.", message_id);
		m_retry_count++;
		if(m_retry_count>=10) {
			printf("Failed after %d attempts.\n", m_retry_count);
			stop();
		} else {
			printf("Retrying again %d.\n", m_retry_count);
			ANT_SetNetworkKey(0, network_key);
		}
		break;
	}
	return TRUE;
}

