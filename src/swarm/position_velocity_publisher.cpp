/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file position_velocity_publisher.cpp
 * Application that publish the position and velocity of the drone
 *
 * @author Matteo Tadiello <matteo.tadiello@studenti.unitn.it>
 */


#include "position_velocity_publisher.h"

#include <px4_getopt.h>
#include <px4_log.h>
#include <px4_posix.h>
#include <poll.h>
#include <uORB/uORB.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_combined.h>
//#include <uORB/topics/ca_trajectory.h>
#include <uORB/topics/vehicle_global_position.h>
//#include <v1.0/custom_messages/mavlink_msg_ca_trajectory.h>

int position_velocity_publisher::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
The module provide a simple publisher of position given by the GPS and the velocity
That run a task in the background.

### Implementation
Section describing the high-level implementation of this module.

### Examples
CLI usage example:
$ position_velocity_publisher start

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("position_velocity_publisher", "start");
	PRINT_MODULE_USAGE_COMMAND("start");
	//PRINT_MODULE_USAGE_PARAM_FLAG('f', "Optional example flag", true);
	//PRINT_MODULE_USAGE_PARAM_INT('p', 0, 0, 1000, "Optional example parameter", true);
	//PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int position_velocity_publisher::print_status()
{
	PX4_INFO("Running");
	// TODO: print additional runtime information about the state of the module

	return 0;
}

int position_velocity_publisher::custom_command(int argc, char *argv[])
{
	/*
	if (!is_running()) {
		print_usage("not running");
		return 1;
	}

	// additional custom commands can be handled like this:
	if (!strcmp(argv[0], "do-something")) {
		get_instance()->do_something();
		return 0;
	}
	 */

	return print_usage("unknown command");
}


int position_velocity_publisher::task_spawn(int argc, char *argv[])
{
	_task_id = px4_task_spawn_cmd("module",
				      SCHED_DEFAULT,
				      SCHED_PRIORITY_DEFAULT,
				      1024,
				      (px4_main_t)&run_trampoline,
				      (char *const *)argv);

	if (_task_id < 0) {
		_task_id = -1;
		return -errno;
	}

	return 0;
}

position_velocity_publisher *position_velocity_publisher::instantiate(int argc, char *argv[])
{
	int example_param = 0;
	bool example_flag = false;
	bool error_flag = false;

	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;

	// parse CLI arguments
	while ((ch = px4_getopt(argc, argv, "p:f", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'p':
			example_param = (int)strtol(myoptarg, nullptr, 10);
			break;

		case 'f':
			example_flag = true;
			break;

		case '?':
			error_flag = true;
			break;

		default:
			PX4_WARN("unrecognized flag");
			error_flag = true;
			break;
		}
	}

	if (error_flag) {
		return nullptr;
	}

	position_velocity_publisher *instance = new position_velocity_publisher(example_param, example_flag);

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
	}

	return instance;
}

position_velocity_publisher::position_velocity_publisher(int example_param, bool example_flag)
	: ModuleParams(nullptr)
{
}

void position_velocity_publisher::run()
{
	int sensor_sub_fd = orb_subscribe(ORB_ID(vehicle_global_position));
	/* limit the update rate to 5 Hz */
	orb_set_interval(sensor_sub_fd, 200);

	px4_pollfd_struct_t fds[1];
    	fds[0].fd = sensor_sub_fd;   
    	fds[0].events = POLLIN;
    	fds[0].revents = POLLIN;

	while(true){
		//int poll_ret = px4_poll(fds, 1, 1000);

		if (fds[0].revents & POLLIN) {
        	/* obtained data for the first file descriptor */
        	struct vehicle_global_position_s raw;
        	/* copy sensors raw data into local buffer */
        	orb_copy(ORB_ID(vehicle_global_position), sensor_sub_fd, &raw);
        	PX4_INFO("Position:\n Latitudine: %f \n Longitudine %f \n \n ", //Altitudine: %f \n vel_n: %f \n vel_e: %f \n vel_d: %f \n \n ",
                    (double)raw.lat,
                    (double)raw.lon);
                    // (float)raw.alt,
                    // (float)raw.vel_n,
                    // (float)raw.vel_e,
                    // (float)raw.vel_d);
    		}

    	}
	// // Example: run the loop synchronized to the sensor_combined topic publication
	// int sensor_combined_sub = orb_subscribe(ORB_ID(sensor_combined));

	// px4_pollfd_struct_t fds[1];
	// fds[0].fd = sensor_combined_sub;
	// fds[0].events = POLLIN;

	// // initialize parameters
	// int parameter_update_sub = orb_subscribe(ORB_ID(parameter_update));
	// parameters_update(parameter_update_sub, true);

	// while (!should_exit()) {

	// 	// wait for up to 1000ms for data
	// 	int pret = px4_poll(fds, (sizeof(fds) / sizeof(fds[0])), 1000);

	// 	if (pret == 0) {
	// 		// Timeout: let the loop run anyway, don't do `continue` here

	// 	} else if (pret < 0) {
	// 		// this is undesirable but not much we can do
	// 		PX4_ERR("poll error %d, %d", pret, errno);
	// 		usleep(50000);
	// 		continue;

	// 	} else if (fds[0].revents & POLLIN) {

	// 		struct sensor_combined_s sensor_combined;
	// 		orb_copy(ORB_ID(sensor_combined), sensor_combined_sub, &sensor_combined);
	// 		// TODO: do something with the data...

	// 	}


	// 	parameters_update(parameter_update_sub);
	// }

	// orb_unsubscribe(sensor_combined_sub);
	// orb_unsubscribe(parameter_update_sub);
}

void position_velocity_publisher::parameters_update(int parameter_update_sub, bool force)
{
	bool updated;
	struct parameter_update_s param_upd;

	orb_check(parameter_update_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(parameter_update), parameter_update_sub, &param_upd);
	}

	if (force || updated) {
		updateParams();
	}
}

int position_velocity_publisher_main(int argc, char *argv[])
{
    return position_velocity_publisher::main(argc, argv);
}




