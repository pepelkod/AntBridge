/*
 * CANTMaster.h
 *
 * Copyright 2017 Douglas Pepelko pepelkod@mega-mouse.net
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
#include "Fortius.h"
#include "ant.h"
#include "ManufacturersList.h"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <pthread.h>
#include <glog/logging.h>


#define MAX_CHANNEL_EVENT_SIZE   (MESG_MAX_SIZE_VALUE)     // Channel event buffer size, assumes worst case extended message size
#define MAX_RESPONSE_SIZE        (MESG_MAX_SIZE_VALUE)     // Protocol response buffer size

#define PAGE_BASIC_RESISTANCE	0x30
#define PAGE_TARGET_POWER	0x31
#define PAGE_TRACK_RESISTANCE	0x33
#define PAGE_USER_CONFIGURATION	0x37	// 55
#define PAGE_COMMAND_STATUS	0x47
#define PAGE_REQUEST		0x46


#define USER_CONFIG_STATE_EMPTY 1
#define USER_CONFIG_STATE_WAITING 2
#define USER_CONFIG_STATE_RX 3

#pragma pack(push, 1)
typedef struct basic_resistance_s {
	uint8_t		data_page_number;	// 0x30 page 48
	uint8_t		reserved[6];		// 0xFF
	uint8_t		resistance_percentage;	// 0-200 in 0.5% increments
} basic_resistance_t;
typedef struct target_power_s {
	uint8_t		data_page_number;	// 0x31 page 49
	uint8_t		reserved[5];		// 0xFF
	uint16_t	target_power_quarter_watts;	// 0.25w increments 0-4000w
} target_power_t;
typedef struct track_resistance_s {
	uint8_t		data_page_number;	// 0x33 page 51
	uint32_t	reserved;		// 0xFFFFFFFF
	int16_t		slope;			// .01% -200 to 200 %
	uint8_t		coefficient_of_rolling;	// 5x10-5 0-0.0127
} track_resistance_t;
typedef struct user_configuration_s{
	uint8_t		data_page_number;	// 0x37 page 55
	uint16_t	user_weight;		// in 0.01kg
	uint8_t		reserved;		// 0xFF
	uint8_t		wheel_diameter_offset:4;	// 1mm
	uint16_t	bike_weight:12;		// 0.05kg
	uint8_t		wheel_diameter;		// 0.01m (1cm 
	uint8_t		gear_ratio;		// ignore?
}user_configuration_t;

typedef struct common_page_70_s{
	uint8_t		data_page_number;	// 0x46 page 70
	uint16_t	slave_serial;		// serial number of slave that is requesting
	uint16_t	descriptor;		// data specification (what data is being requested?)
	uint8_t		response_cnt:7;		// how many times to retransmit
	uint8_t		response_try:1;		// 1 == try forever...0 = dont try forever
	uint8_t		requested_page_number;	// 0x47 perhaps?  what page do you want back?
	uint8_t		command_type;		// 1 data page, 2 ant fs session, 3 data page from slave, 4 data page set
}request_t, common_page_70_t;
	
typedef struct common_page_71_s{
	uint8_t		data_page_number;	// 0x47 page 71
	uint8_t		last_rx_command_id;	// 0xFF means no command rxed
	uint8_t		sequence_number;	//
	uint8_t		command_status;		// 0 pass, 1 fail 2 not supported, 3 rejected 4 pending, 5-254 reserved, 255 uninitialized
	uint8_t		data[4];		// data pertaining to the last command
}command_status_t, common_page_71_t;

#pragma pack(pop)

//this class for ANT+ Master
class CANTMaster
{
public:
		CANTMaster();
		~CANTMaster();
	bool	init(Fortius* fortius);
	bool	start();
	bool	join();
	bool	stop();
	bool	kill();

	static void*	mainloop_helper(void *context);
	void*		mainloop(void);
private:

	void		hex_dump(uint8_t* data, int data_size);
	double		to_seconds(struct timespec* ts);
	static int8_t	channel_callback(uint8_t channel_number, uint8_t event);
	int8_t		channel_handler(uint8_t channel_number, uint8_t event);
	static int8_t	response_callback(uint8_t channel_number, uint8_t message_id);
	int8_t		response_handler(uint8_t channel_number, uint8_t message_id);
	bool		fec_init(uint8_t message_id,uint8_t result);//called by response_callback
	double		calc_power_required_watts(double velocity_kph, double grade);

	bool		send_request_page(uint8_t request_page);
	bool		send_general_fe(double accumulated_time_seconds, uint8_t  distance, uint16_t speed, uint8_t heart_rate, uint8_t lap);
	bool		send_general_settings(double incline_percent, uint8_t resistance);
	bool		send_specific_trainer(uint8_t cadence, uint16_t power);
	bool		send_fe_capabilities();
	bool		send_target_power(uint16_t input_target_power_watts);
	bool		send_common_data_page_80();
	bool		send_common_data_page_81();

	bool		process_basic_resistance(basic_resistance_t* basic_resistance);
	bool		process_target_power(target_power_t* target_power);
	bool		process_track_resistance(track_resistance_t* track_resistance);
	bool		process_request(request_t* request);
	bool		process_user_configuration(user_configuration_t*);
public:
	static CANTMaster*	this_ptr;
private:
	uint8_t			m_channel_buffer[MAX_CHANNEL_EVENT_SIZE];
	uint8_t 		m_response_buffer[MAX_RESPONSE_SIZE];
	bool	 		m_exit_flag;
	bool	 		m_channel_open;
	pthread_t		m_pthread;
	int			m_retry_count;
	Fortius*		m_fortius;
	uint8_t			m_channel_number;

	double			m_user_weight_kg;
	double			m_bike_weight_kg;
	double			m_wheel_circumference_mm;


	uint8_t			m_user_config_state;			// USER_CONFIG_STATE_EMPTY->USER_CONFIG_STATE_WAITING->USER_CONFIG_STATE_RX;	
	uint16_t		m_device_id;
	double			m_speed_kph;
	double			m_slope;
	pthread_mutex_t		m_vars_mutex;
};

