/****************************************************************************
 *
 *   Copyright (c) 2012-2019 PX4 Development Team. All rights reserved.
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
 * @file angles_test.cpp
 * Test for tilting multicopters.
 * It sends a desired orientation to the drone while it is flying
 *
 * It requires 2 inputs:
 * 	1) the type of angle: "r"=roll, "p"=pitch, "b"= both
 * 	2) the angle value in degrees
 *
 * @author Salvatore Marcellini <salvatore.marcellini@gmail.com>
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/posix.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>
#include <math.h>

#include <uORB/uORB.h>

#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/tilting_mc_desired_angles.h>

extern "C" __EXPORT int angles_test_main(int argc, char *argv[]);

int angles_test_main(int argc, char *argv[])
{
	PX4_INFO("Angles test");

	/* advertise tilting_angles topic */
	struct tilting_mc_desired_angles_s angles;


	if( strcmp(argv[1], "p") == 0 ){
		angles.pitch_body = (float)atof(argv[2]) * M_DEG_TO_RAD_F;
		PX4_INFO("Publish %2.2f° pitch", (double)atof(argv[2]));
	}
	else if ( strcmp(argv[1], "r") == 0 ){
		angles.roll_body = (float)atof(argv[2]) * M_DEG_TO_RAD_F;
		PX4_INFO("Publish %2.2f° roll", (double)atof(argv[2]));
	}
	else if ( strcmp(argv[1], "b") == 0 ){
		angles.pitch_body = (float)atof(argv[2]) * M_DEG_TO_RAD_F;
		angles.roll_body = (float)atof(argv[2]) * M_DEG_TO_RAD_F;
		PX4_INFO("Publish %2.2f° both", (double)atof(argv[2]));
	}
	angles.timestamp = hrt_absolute_time();

	orb_advert_t angles_pub = orb_advertise(ORB_ID(tilting_mc_desired_angles), &angles);

	orb_publish(ORB_ID(tilting_mc_desired_angles), angles_pub, &angles);

	PX4_INFO("Done!");



	return 0;
}
