
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "Fortius.h"
#include "CANTMaster.h"
#include "cxxopts.hpp"

bool		exit_main_loop = false;

CANTMaster*		ant_master=NULL;

void ctrlc_handler(int sig)
{
	static bool first_time=true;

	printf("Caught ctrl-c. sig %d\n", sig);
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

#define PIDFILE "/var/run/%s.pid"

int main(int argc, char* argv[])
{
	Fortius*						fortius=NULL;
	fortius_telemetry_t	fortius_telemetry;
	char								pid_file_path[100];
	FILE*								pid_fd;
	double							user_weight	= DEFAULT_WEIGHT;
	double							bike_weight = 8.6;
	double 							wheel_circumference_mm = 2105;

	// catch ctrl-c
	signal(SIGINT, ctrlc_handler);

	// Initialize Google Logging
	google::InitGoogleLogging (argv [0]);
	FLAGS_logtostderr = 1;
	FLAGS_v = 0;
	VLOG (1) << "Logging initialized";

	// Parse commandline arguments
	try {
		cxxopts::Options options(argv [0], "Transform your Tacx Fortius into an ANT+ compatible trainer");

		// Supported commandline options
		options.add_options ("Basic")
  		("d,debug", "Set debug level", cxxopts::value<int>(),"LEVEL")
			("u,userweight", "Set rider weight in [kg]", cxxopts::value<int>(), "WEIGHT")
			("b,bikeweight", "Set bike weight in [kg]", cxxopts::value<int>(), "WEIGHT")
			("c,wheelcircum", "Set wheel circumference in [mm]", cxxopts::value<int>(), "CIRCUMFERENCE")
			("h,help", "Print help")
  	;

		// Parse
		auto result = options.parse(argc, argv);

		if (result.count("h")) {
			std::cout << options.help({"", "Basic"}) << std::endl;
		  exit(0);
		};

		if (result.count("d")) {
			FLAGS_v = result["d"].as<int>();
			if ((FLAGS_v < 0) || (FLAGS_v > 5)) {
				std::cout << "Invalid debug level" << std::endl;
				exit (1);
			}
		};

		if (result.count("u")) {
			user_weight = result["u"].as<int>();
			if ((user_weight < 0) || (user_weight > 300)) {
				std::cout << "Invalid user weight" << std::endl;
				exit (1);
			}
		};

		if (result.count("b")) {
			bike_weight = result["b"].as<int>();
			if ((bike_weight < 0) || (bike_weight > 30)) {
				std::cout << "Invalid bike weight" << std::endl;
				exit (1);
			}
		};

		if (result.count("c")) {
			wheel_circumference_mm = result["c"].as<int>();
			if ((wheel_circumference_mm < 0) || (wheel_circumference_mm > 2500)) {
				std::cout << "Invalid wheel circumference" << std::endl;
				exit (1);
			}
		};

	} catch (const cxxopts::OptionException& e) {
    std::cout << "error parsing options: " << e.what() << std::endl;
    exit(1);
  }

	// Print intial Settings
	std::cout << "Initial settings\n";
	std::cout << "----------------\n";
	std::cout << "User weight         : " << user_weight << " [kg]\n";
	std::cout << "Bike weight         : " << bike_weight << " [kg]\n";
	std::cout << "Wheel circumference : " << wheel_circumference_mm << " [mm]\n" << std::endl;

	// setup pidfile
	snprintf(pid_file_path, sizeof(pid_file_path), PIDFILE, argv[0]);
	pid_fd = fopen(pid_file_path, "wb");
	if(!pid_fd){
		printf("Failed to create pid file. Please run as SU.\n");
//		return -1;
	}
	pid_t	my_pid = getpid();
	fprintf(pid_fd, "%d", my_pid);
	fclose(pid_fd);

	// Initialize Tacx Fortius
	fortius = new Fortius();
	if(fortius) {
		std::cout << "Fortius initialized" << std::endl;
	} else {
		std::cout << "Failed to create Fortius connection" << std::endl;
		fortius->stop ();
		exit (1);
	}

	// Initialize ANT dongle
	ant_master = new CANTMaster();
	if (ant_master) {
		std::cout << "ANT+ dongle initialized" << std::endl;
		if (ant_master->init (fortius) == FALSE) {
			std::cout << "Failed to init ANT+ dongle" << std::endl;
			fortius->stop ();
			ant_master->stop ();
			exit (1);
		}
	} else {
		std::cout << "Failed to initialize ANT+ dongle" << std::endl;
		fortius->stop ();
		ant_master->stop ();
		exit (1);
	}

	// Start reading from Fortius
	fortius->start();
	fortius->setWeight (user_weight);

	// Start reading from ANT+ module
	ant_master->start();
	ant_master->set_defaults (user_weight, bike_weight, wheel_circumference_mm);

	do {
		// check on  Fortius
		fortius->getTelemetry(	fortius_telemetry.power, fortius_telemetry.heartrate,
														fortius_telemetry.cadence, fortius_telemetry.speed,
														fortius_telemetry.distance, fortius_telemetry.buttons,
														fortius_telemetry.steering, fortius_telemetry.status);

		if (fortius_telemetry.status == FT_ERROR) {
			std::cout << "Error in Fortius" << std::endl;
			break;
		}
		// Wait for a second
		sleep(1);
	} while (exit_main_loop == FALSE);

	if (fortius) {
		std::cout << "Stopping Fortius" << std::endl;
		fortius->stop ();
	}

	if (ant_master) {
		std::cout << "Stopping ANT+ module" << std::endl;
		ant_master-> stop();
	}

	if (fortius) {
		std::cout << "Closing Fortius" << std::endl;
		fortius->join();
		delete fortius;
		fortius = NULL;
		std::cout << "Fortius closed" << std::endl;
	}

	if (ant_master) {
		std::cout << "Closing ANT+ module" << std::endl;
		ant_master-> join();
		delete ant_master;
		ant_master = NULL;
		std::cout << "ANT+ module closed" << std::endl;
	}
}
