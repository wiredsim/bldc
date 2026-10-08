/*
	Copyright 2026 wiredsim

	This file is part of the VESC firmware.

	The VESC firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    The VESC firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * AC induction motor mode for the FOC driver. See documentation/acim/DESIGN.md.
 *
 * Everything is off unless "Enable ACIM Mode" is set on the ACIM page in VESC Tool
 * (custom config). Only motor 1 is supported.
 */

#ifndef ACIM_H_
#define ACIM_H_

#include <stdbool.h>
#include "datatypes.h"

void acim_init(void);

// True while ACIM mode owns the motor. Latched only while the motor is undriven.
bool acim_active(bool is_second_motor);

// ADC ISR, motor undriven. Latches the enable flag, tracks the decaying rotor flux and
// returns true (with the flux angle in *phase) when ACIM mode is active.
bool acim_isr_undriven(bool is_second_motor, const mc_configuration *conf, float dt,
		float enc_phase, bool enc_valid, float rs, float *phase);

// ADC ISR, motor driven. With takeover the flux angle and the Id/Iq setpoints are
// overwritten; without (open-loop and handbrake modes) the estimator only tracks.
void acim_isr_driven(bool is_second_motor, const mc_configuration *conf, float dt, bool takeover,
		float i_alpha, float i_beta, float v_alpha, float v_beta,
		float enc_phase, bool enc_valid, float rs, float iq_request,
		float *phase, float *id_set, float *iq_set);

// Rotor electrical angle, fed to the speed PLL and tachometer so they report rotor speed
float acim_rotor_phase(bool is_second_motor);

// Current-controller feed-forward using the stator frequency and the rotor flux
void acim_decoupling(bool is_second_motor, const mc_configuration *conf, float id, float iq,
		float *dec_vd, float *dec_vq, float *dec_bemf);

// BEMF feed-forward (q axis) the current controller will add, for the undriven integrator preset
float acim_bemf_q(bool is_second_motor, const mc_configuration *conf);

// How long the field should still be held without a torque request [s]
float acim_hold_time_left(bool is_second_motor);

#endif /* ACIM_H_ */
