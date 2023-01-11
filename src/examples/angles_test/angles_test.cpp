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
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/tilting_mc_desired_angles.h>
#include <mathlib/math/Functions.hpp>
#include <matrix/matrix/math.hpp>

extern "C" __EXPORT int angles_test_main(int argc, char *argv[]);

matrix::Vector<float, 4> coefficients(float start, float end, float tf){
	matrix::Vector<float,4> coeff;
	coeff(0) = start;
	coeff(1) = 0.0f;
	coeff(3) = (start - end) * 2 / powf(tf,3);
	coeff(2) = -1.5f * coeff(3) * tf;

	return coeff;
}

int angles_test_main(int argc, char *argv[])
{
	PX4_INFO("Angles test");

	/* advertise tilting_angles topic */
	struct tilting_mc_desired_angles_s angles;
	float des_pitch = 0.0f;
	float des_roll = 0.0f;
	float pitch = 0.0f;
	float roll = 0.0f;
	float dt = 0.01f;
	bool p = false;
	bool r = false;
	uORB::Publication<tilting_mc_desired_angles_s> tilting_angles_pub{ORB_ID(tilting_mc_desired_angles)};
	uORB::Subscription vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
	hrt_abstime stime;
	vehicle_attitude_s att;

	float tf = (float)atof(argv[3]);
	float des_angle = (float)atof(argv[2]) * M_DEG_TO_RAD_F;

	if( strcmp(argv[1], "p") == 0 ){
		des_pitch = des_angle;
		des_roll = 0.0f;
		p = true;
		PX4_INFO("Publish %2.2f° pitch in %2.2f s, starting from %2.3f", (double)des_angle, (double)tf, (double)pitch);
	}
	else if ( strcmp(argv[1], "r") == 0 ){
		des_roll = des_angle;
		des_pitch = 0.0f;
		r = true;
		PX4_INFO("Publish %2.2f° roll in %2.2f s", (double)des_angle, (double)tf);
	}
	else if ( strcmp(argv[1], "b") == 0 ){
		des_roll = des_pitch = des_angle;
		p = r = true;
		PX4_INFO("Publish %2.2f° both in %2.2f s", (double)des_angle, (double)tf);
	}

	stime = hrt_absolute_time();
	while( (hrt_absolute_time() - stime) < 5000 ){
		if (vehicle_attitude_sub.updated()) {

			if (vehicle_attitude_sub.copy(&att)) {
				matrix::Quatf q{att.q};
				pitch = matrix::Eulerf(q).theta();
				roll = matrix::Eulerf(q).phi();
			}
		}
	}

	int steps = lround(tf/dt);
	matrix::Vector<float,4> coeff;
	if(p && !r)
		coeff = coefficients(pitch, des_pitch, tf);
	else if(r && !p)
		coeff = coefficients(roll, des_roll, tf);

	// float s = 0.0f;
	stime = hrt_absolute_time();
	int count = 0;
	// int t = 0.0f;
	while(count < steps){
		if((hrt_absolute_time() - stime) > 7000){

			//dt = dt*powf(10,-6);
			// s = coeff(3)*powf(t,3) + coeff(2)*powf(t,2) + coeff(1)*t + coeff(0);

			if(p && !r){
				// angles.pitch_body = pitch + dt *( (des_pitch-pitch))/(sqrtf(powf(des_pitch,2) - powf(pitch,2)));
				angles.pitch_body = pitch + (des_pitch - pitch) * count/steps;
				angles.roll_body = 0.0f;
			}
			else if(r && !p){
				angles.roll_body = roll + (des_roll - roll) * count/steps;
				angles.pitch_body = 0.0f;
			}
			angles.timestamp = hrt_absolute_time();
			tilting_angles_pub.publish(angles);
			PX4_INFO("%d angle: %2.2f",count, (double)angles.pitch_body);


			count++;
			stime = hrt_absolute_time();
		}
	}

	PX4_INFO("Done!");

	return 0;

}
