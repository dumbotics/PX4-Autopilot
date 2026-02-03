/****************************************************************************
 *
 *   Copyright (c) 2026.
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
 * @file intercept.cpp
 *
 * CLI helper to publish intercept_setpoint and request AUTO_INTERCEPT.
 * Intended for dev/SITL bringup before MAVLink/QGC integration exists.
 */

#include <drivers/drv_hrt.h>
#include <mathlib/mathlib.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/module.h>

#include <uORB/Publication.hpp>
#include <uORB/topics/intercept_setpoint.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_status.h>

#include <cstdlib>

extern "C" __EXPORT int intercept_main(int argc, char *argv[]);

static void usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
Publish an intercept plan (intercept_setpoint) and switch to AUTO_INTERCEPT.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("intercept", "command");
	PRINT_MODULE_USAGE_COMMAND_DESCR("set", "Publish intercept_setpoint and switch to AUTO_INTERCEPT");
	PRINT_MODULE_USAGE_ARG("<lat>", "Intercept latitude [deg]", false);
	PRINT_MODULE_USAGE_ARG("<lon>", "Intercept longitude [deg]", false);
	PRINT_MODULE_USAGE_ARG("<alt_amsl>", "Intercept altitude [m AMSL]", false);
	PRINT_MODULE_USAGE_ARG("<course_deg>", "Approach course [deg], 0=N, +CW", false);
	PRINT_MODULE_USAGE_ARG("<standoff_m>", "Standoff distance behind intercept point [m]", false);
	PRINT_MODULE_USAGE_ARG("[run_m]", "Egress/run length [m] (optional)", true);
	PRINT_MODULE_USAGE_ARG("[cruise_mps]", "Cruise speed [m/s] (optional)", true);
	PRINT_MODULE_USAGE_ARG("[acc_rad_m]", "Acceptance radius [m] (optional)", true);
	PRINT_MODULE_USAGE_ARG("[rtl_on_complete]", "1=yes (default), 0=no", true);
}

static void publish_intercept_and_switch(const intercept_setpoint_s &sp)
{
	uORB::Publication<intercept_setpoint_s> intercept_pub{ORB_ID(intercept_setpoint)};
	intercept_pub.publish(sp);

	vehicle_command_s cmd{};
	cmd.timestamp = hrt_absolute_time();
	cmd.command = vehicle_command_s::VEHICLE_CMD_SET_NAV_STATE;
	cmd.param1 = vehicle_status_s::NAVIGATION_STATE_AUTO_INTERCEPT;
	cmd.from_external = false;

	uORB::Publication<vehicle_command_s> cmd_pub{ORB_ID(vehicle_command)};
	cmd_pub.publish(cmd);
}

int intercept_main(int argc, char *argv[])
{
	if (argc < 2) {
		usage(nullptr);
		return 1;
	}

	if (!strcmp(argv[1], "help") || !strcmp(argv[1], "--help") || !strcmp(argv[1], "-h")) {
		usage(nullptr);
		return 0;
	}

	if (!strcmp(argv[1], "set")) {
		if (argc < 7) {
			usage("missing arguments");
			return 1;
		}

		intercept_setpoint_s sp{};
		sp.timestamp = hrt_absolute_time();
		sp.valid = true;
		sp.frame = 0; // GLOBAL (WGS84)

		sp.lat = strtod(argv[2], nullptr);
		sp.lon = strtod(argv[3], nullptr);
		sp.alt_amsl = strtof(argv[4], nullptr);
		sp.approach_course = math::radians(strtof(argv[5], nullptr));
		sp.standoff_distance = strtof(argv[6], nullptr);

		sp.run_length = (argc > 7) ? strtof(argv[7], nullptr) : 0.f;
		sp.cruise_speed = (argc > 8) ? strtof(argv[8], nullptr) : -1.f;
		sp.acceptance_radius = (argc > 9) ? strtof(argv[9], nullptr) : 0.f;

		const bool rtl_on_complete = (argc > 10) ? (strtol(argv[10], nullptr, 10) != 0) : true;
		sp.flags = rtl_on_complete ? 1u : 0u;

		publish_intercept_and_switch(sp);

		PX4_INFO("intercept_setpoint published (lat=%.7f lon=%.7f alt=%.1f course=%.1fdeg standoff=%.1f run=%.1f)",
			 (double)sp.lat, (double)sp.lon, (double)sp.alt_amsl,
			 (double)math::degrees(sp.approach_course),
			 (double)sp.standoff_distance, (double)sp.run_length);

		return 0;
	}

	usage("unknown command");
	return 1;
}

