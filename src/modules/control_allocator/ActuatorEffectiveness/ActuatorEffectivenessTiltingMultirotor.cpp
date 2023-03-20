/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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
 * @file ActuatorEffectivenessTiltingMultirotor.hpp
 *
 * Actuator effectiveness computed from rotors position and orientation
 *
 * @author Salvatore Marcellini <salvatore.marcellini@gmail.com>
 */

#include "ActuatorEffectivenessTiltingMultirotor.hpp"

using namespace matrix;

ActuatorEffectivenessTiltingMultirotor::ActuatorEffectivenessTiltingMultirotor(ModuleParams *parent)
	: ModuleParams(parent)
{

	_tilting_type_handle = param_find("CA_TILTING_TYPE");
	for(int i=0; i<NUM_SERVOS_MAX; i++){
		char buffer[17];
		snprintf(buffer, sizeof(buffer), "CA_SV_TL%u_MINA", i);
		_servo_param_handles[i].angle_min = param_find(buffer);
		snprintf(buffer, sizeof(buffer), "CA_SV_TL%u_MAXA", i);
		_servo_param_handles[i].angle_max = param_find(buffer);
	}
	_servo_count_handle = param_find("CA_SV_TL_COUNT");

	updateParams();

	_tilts = new ActuatorEffectivenessTilts(this);
}

void ActuatorEffectivenessTiltingMultirotor::updateParams()
{
	ModuleParams::updateParams();

	if (param_get(_tilting_type_handle, &_tilting_type) != 0) {
		PX4_ERR("param_get failed");
		return;
	}

	if (param_get(_servo_count_handle, &_servo_count) != 0) {
		PX4_ERR("param_get failed");
		return;
	}

	for(int i=0; i<_servo_count; i++){
		param_get(_servo_param_handles[i].angle_min, &_servo_param[i].angle_min);
		param_get(_servo_param_handles[i].angle_max, &_servo_param[i].angle_max);
	}
	for(int i=_servo_count; i<(NUM_SERVOS_MAX-_servo_count); i++){
		_servo_param[i].angle_max = 0.00f;
		_servo_param[i].angle_min = 0.00f;
	}

	// PX4_INFO("Tilting-tilt type: %i \n", (int)_tilting_type);
	if(_tilting_type == 0) //Non omnidirectional
	{
		_mc_rotors = new ActuatorEffectivenessRotors(this, ActuatorEffectivenessRotors::AxisConfiguration::FixedUpwards);
		_mc_rotors_vertical = nullptr;
		_mc_rotors_lateral = nullptr;
	}
	else{ //Omnidirectional tilting

		_mc_rotors = nullptr;
		_mc_rotors_vertical = new ActuatorEffectivenessRotors(this, ActuatorEffectivenessRotors::AxisConfiguration::FixedUpwards,
								      false, true);

		_mc_rotors_lateral = new ActuatorEffectivenessRotors(this, ActuatorEffectivenessRotors::AxisConfiguration::FixedUpwards,
								      false, true);
	}
}

bool
ActuatorEffectivenessTiltingMultirotor::getEffectivenessMatrix(Configuration &configuration,
		EffectivenessUpdateReason external_update)
{
	if (external_update == EffectivenessUpdateReason::NO_EXTERNAL_UPDATE) {
		return false;
	}

	// Motors
	// Update matrix with tilts in vertical position when update is triggered by a manual
	// configuration (parameter) change. This is to make sure the normalization
	// scales are tilt-invariant. Note: configuration updates are only possible when disarmed.
	// const float tilt_control_applied = (external_update == EffectivenessUpdateReason::CONFIGURATION_UPDATE) ? -1.f :
	// 				   _last_tilt_control;
	// _nontilted_motors = _mc_rotors_vertical->updateAxisFromTilts(_tilts, tilt_control_applied)
	// 		    << configuration.num_actuators[(int)ActuatorType::MOTORS];

	bool rotors_added_successfully = false;

	//Rotors
	if(_tilting_type == 0){ //Non omnidirectional{
		configuration.selected_matrix = 0;
		rotors_added_successfully = _mc_rotors->addActuators(configuration);

		// Tilts
		configuration.selected_matrix = 0;
		_first_tilt_idx = configuration.num_actuators_matrix[configuration.selected_matrix];
		_tilts->updateTorqueSign(_mc_rotors->geometry(), true /* disable pitch to avoid configuration errors */);

	}
	else{ //Omnidirectional tilting

		//Vertical forces matrix
		configuration.selected_matrix = 0;
		rotors_added_successfully = _mc_rotors_vertical->addActuators(configuration);

		// Lateral forces matrix
		configuration.selected_matrix = 1;
		rotors_added_successfully = _mc_rotors_lateral->addActuators(configuration);

		*configuration.num_actuators /=2;

		// Tilts
		configuration.selected_matrix = 0;
		_first_tilt_idx = configuration.num_actuators_matrix[configuration.selected_matrix];
		// _tilts->updateTorqueSign(_mc_rotors_vertical->geometry(), true /* disable pitch to avoid configuration errors */);
		// _tilts->updateTorqueSign(_mc_rotors_lateral->geometry(), true /* disable pitch to avoid configuration errors */);

	}

	const bool tilts_added_successfully = _tilts->addActuators(configuration);

	return (rotors_added_successfully && tilts_added_successfully);
}

void
ActuatorEffectivenessTiltingMultirotor::updateSetpoint(const matrix::Vector<float, NUM_AXES> &control_sp, int matrix_index,
			    ActuatorVector &actuator_sp){

	actuator_controls_s actuator_controls_0;
	if(_tilting_type == 0){
		if (_actuator_controls_0_sub.copy(&actuator_controls_0)){

			float tilt_sp = actuator_controls_0.control[actuator_controls_s::INDEX_FLAPS];

			//TO DO: change min e max with the servo limits
			tilt_sp = tilt_sp < -0.99f ? -1.f : tilt_sp;
			tilt_sp = tilt_sp > 0.99f ? 1.f : tilt_sp;

			// initialize _last_tilt_control
			if (!PX4_ISFINITE(_last_tilt_control)) {
				_last_tilt_control = tilt_sp;

			// To DO: check if 0.02f is a good threshold
			} else if (fabsf(tilt_sp - _last_tilt_control) > 0.02f) {
				_tilt_updated = true;
				_last_tilt_control = tilt_sp;
			}

			for (int i = 0; i < _tilts->count(); ++i) {
				if (_tilts->config(i).tilt_direction == ActuatorEffectivenessTilts::TiltDirection::TowardsFront) {
					actuator_sp(i + _first_tilt_idx) += tilt_sp;
				}
			}
		}
	}
	else if(_tilting_type!=0 && matrix_index == 1){

		for(int i=0; i<_servo_count; i++){

			actuator_sp(i) = actuator_sp(i) <  math::radians(_servo_param[i].angle_min) ?  math::radians(_servo_param[i].angle_min) : actuator_sp(i);
			actuator_sp(i) = actuator_sp(i) >  math::radians(_servo_param[i].angle_max) ?  math::radians(_servo_param[i].angle_max) : actuator_sp(i);

			// PX4_INFO("%d) tilt_sp: %f", i, (double)tilt_sp[i]);
		}
	}
}
