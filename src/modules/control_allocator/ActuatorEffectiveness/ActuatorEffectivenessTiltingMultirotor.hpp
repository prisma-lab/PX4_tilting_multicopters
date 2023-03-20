/****************************************************************************
 *
 *   Copyright (c) 2020-2022 PX4 Development Team. All rights reserved.
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

#pragma once

#include "ActuatorEffectiveness.hpp"
#include "ActuatorEffectivenessRotors.hpp"
#include "ActuatorEffectivenessTilts.hpp"
#include <px4_platform_common/module_params.h>

#include <uORB/topics/actuator_controls.h>
#include <uORB/Subscription.hpp>

class ActuatorEffectivenessTiltingMultirotor : public ModuleParams, public ActuatorEffectiveness
{
public:
	ActuatorEffectivenessTiltingMultirotor(ModuleParams *parent);
	virtual ~ActuatorEffectivenessTiltingMultirotor() = default;

	bool getEffectivenessMatrix(Configuration &configuration, EffectivenessUpdateReason external_update) override;

	int numMatrices() const override { return _tilting_type == 0 ? 1 : 2; }

	void getDesiredAllocationMethod(AllocationMethod allocation_method_out[MAX_NUM_MATRICES]) const override
	{
		if(_tilting_type == 0){ //Non omnidirectional
			allocation_method_out[0] = AllocationMethod::SEQUENTIAL_DESATURATION;
		}
		else{ //Omnidirectional tilting
			allocation_method_out[0] = AllocationMethod::SEQUENTIAL_DESATURATION;
			allocation_method_out[1] = AllocationMethod::SEQUENTIAL_DESATURATION;
		}
	}

	void getNormalizeRPY(bool normalize[MAX_NUM_MATRICES]) const override
	{
		if(_tilting_type == 0){ //Non omnidirectional
			normalize[0] = true;
		}
		else{ //Omnidirectional tilting
			normalize[0] = true;
			normalize[1] = true;
		}
	}

	void updateSetpoint(const matrix::Vector<float, NUM_AXES> &control_sp, int matrix_index,
			    ActuatorVector &actuator_sp) override;

	const char *name() const override { return "Tilting Multirotor"; }

protected:
	void updateParams() override;

	ActuatorEffectivenessRotors *_mc_rotors;
	ActuatorEffectivenessRotors *_mc_rotors_vertical;
	ActuatorEffectivenessRotors *_mc_rotors_lateral;
	ActuatorEffectivenessTilts *_tilts;

	param_t _tilting_type_handle;

	uint32_t _nontilted_motors{}; ///< motors that are not tiltable

	int _first_tilt_idx{0};
	float _last_tilt_control{NAN};
	bool _tilt_updated{true};
	int32_t _tilting_type{0};

	static constexpr int NUM_SERVOS_MAX = 5;
	struct ServoParamHandles{
		param_t angle_min;
		param_t angle_max;
	};
	param_t _servo_count_handle;
	int32_t _servo_count{0};

	struct ServoParam{
		float angle_min;
		float angle_max;
	};
	
	ServoParamHandles _servo_param_handles[NUM_SERVOS_MAX];
	ServoParam _servo_param[NUM_SERVOS_MAX];

	uORB::Subscription _actuator_controls_0_sub{ORB_ID(actuator_controls_0)};
};
