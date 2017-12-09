
#include <stdio.h>
#include <stdlib.h>

#include "Fortius.h"
#include "CANTMaster.h"

bool		exit_main_loop = false;

CANTMaster*		ant_master=NULL;

void ctrlc_handler(int sig)
{
	static bool first_time=true;

	printf("Caught ctrl-c.\n");
	if(first_time == true) {
		first_time = false;
		printf("first time.\n");

		exit_main_loop = true;
	} else {
		printf("second or later time\n");
		if(ant_master) {
			ant_master->kill();
		}
	}
}

typedef struct fortius_telemetry_s {
	double power;
	double heartrate;
	double cadence;
	double speed;
	double distance;
	int buttons;
	int steering;
	int status;
} fortius_telemetry_t;

int main(int argc, char* argv[])
{
	Fortius*		fortius=NULL;
	fortius_telemetry_t	fortius_telemetry;

	// catch ctrl-c
	signal(SIGINT, ctrlc_handler);

	fortius = new Fortius();
	if(fortius) {
		printf("Hello Fortius.\n");
	} else {
		printf("Failed to create Fortius.\n");
		goto exit_main;
	}
	ant_master = new CANTMaster();
	if(ant_master) {
		printf("Hello ANT Dongle.\n");
		if(false == ant_master->init(fortius)) {
			printf("Failed to init ant\n");
			goto exit_main;
		}
	} else {
		printf("Failed to create ANT Dongle.\n");
		goto exit_main;
	}

	fortius->start();
	ant_master->start();

	do {

		// check on  Fortius
		fortius->getTelemetry(	fortius_telemetry.power, fortius_telemetry.heartrate,
								fortius_telemetry.cadence, fortius_telemetry.speed,
								fortius_telemetry.distance, fortius_telemetry.buttons,
								fortius_telemetry.steering, fortius_telemetry.status);
		if(fortius_telemetry.status == FT_ERROR) {
			printf("Error in Fortius.\n");
			break;
		}
		sleep(1);
	} while(false == exit_main_loop);

exit_main:
	if(fortius) {
		printf("Stopping Fortius.\n");
		fortius->stop();
	}
	if(ant_master) {
		printf("Stopping Ant.\n");
		ant_master->stop();
	}

	if(fortius) {
		printf("Closing Foritus.\n");
		fortius->join();
		delete fortius;
		fortius = NULL;
		printf("Fortius Closed.\n");
	}
	if(ant_master) {
		printf("Closing Ant.\n");
		ant_master->join();
		delete ant_master;
		ant_master = NULL;
		printf("Ant Closed.\n");
	}
	printf("Goodbye.\n");

}