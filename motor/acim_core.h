/*
	Copyright 2026 Adam (wiredsim)

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
 * AC induction motor (ACIM) estimator and state machine.
 *
 * Pure math, no firmware dependencies, so that the same file is compiled into the
 * firmware and into the host-side test (documentation/acim/sim) that runs it against
 * the Python motor model. The structure follows documentation/acim/DESIGN.md and
 * documentation/acim/sim/acim_sim.py (AcimController) line by line.
 */

#ifndef ACIM_CORE_H_
#define ACIM_CORE_H_

#include <stdbool.h>
#include <stdint.h>

typedef enum {
	ACIM_STATE_OFF = 0,
	ACIM_STATE_FLUXING,
	ACIM_STATE_RUN,
	ACIM_STATE_HOLD,     // RUN with no torque request: field kept, Iq = 0
	ACIM_STATE_FAULT
} acim_state_t;

typedef enum {
	ACIM_FAULT_NONE = 0,
	ACIM_FAULT_FLUX,
	ACIM_FAULT_SLIP,
	ACIM_FAULT_ENCODER
} acim_fault_t;

// Configuration in SI units (electrical Hz, A, s, H)
typedef struct {
	bool sensorless;
	float id_mag;
	float id_min_frac;      // light-load flux reduction: floor as a fraction of id_mag, 0 = off
	float tau_r;
	float lm;
	float lr_lm;
	float slip_max_hz;
	float flux_build_time;  // 0 = 3 * tau_r
	float flux_hold_time;
	float current_max;
	float sl_min_hz;
	float sl_if_ramp;
	float obs_bw;
	float fault_flux_err;   // fraction, 0.5 = 50 %
	float fault_slip_fac;
} acim_core_conf_t;

typedef struct {
	float dt;               // control period [s]
	bool driven;            // PWM on
	float i_alpha;
	float i_beta;
	float v_alpha_prev;     // voltage applied over the period that just ended
	float v_beta_prev;
	float enc_phase;        // rotor electrical angle from the encoder [rad]
	bool enc_valid;
	float iq_request;       // what the rest of the firmware asks for [A]
	float iq_request_min;   // |iq_request| below this is "no torque request"
	float i_avail;          // largest current magnitude the loop may drive [A], 0 = not limited
	float rs;               // stator resistance [Ohm]
	float sigma_ls;         // transient inductance [H] (foc_motor_l)
} acim_core_in_t;

typedef struct {
	float phase;            // angle for Park / inverse Park [rad]
	float id_ref;
	float iq_ref;
	acim_fault_t fault;     // set on the sample a fault is detected
} acim_core_out_t;

typedef struct {
	// Current model, flux in amps (i_mr = psi_r / Lm)
	float imr_a, imr_b;
	float theta;
	float k_decay;          // 1 - exp(-dt / tau_r), cached
	float k_dt, k_tau;      // dt and tau_r k_decay was computed for

	// Encoder speed PLL
	bool enc_init;
	float enc_prev;
	float pll_th, pll_w;

	// Voltage-model observer
	float ps_a, ps_b;       // stator flux [Vs]
	float ci_a, ci_b;       // corrector integrators
	float psr_vm_a, psr_vm_b;
	float vm_th, vm_w, vm_w_int;
	float vm_w_gate;        // |vm_w| low-passed, gates the voltage-model checks [rad/s]
	float id_dyn;           // flux target with light-load reduction [A]
	float vm_th_raw, vm_th_raw_prev;
	float w_sl_obs;

	// Sensorless
	float wr_hat;
	bool if_active;
	float if_th, if_f;

	// State machine
	acim_state_t state;
	acim_fault_t fault;
	float t_state;
	float t_handover;
	float flux_err_t, slip_err_t, slip_plaus_t;
	float hold_left;        // remaining time the field should be kept without a request

	// Telemetry
	float wr;               // rotor electrical speed [rad/s]
	float w_s;              // stator (flux) electrical speed [rad/s]
	float imr;              // |i_mr| [A]
	float slip_hz;
	float rotor_phase;      // rotor electrical angle [rad], for the speed PLL / tachometer
	float id, iq;           // measured, in the flux frame
	float id_ref, iq_ref;
	float iq_unclamped;
} acim_core_t;

void acim_core_reset(acim_core_t *s);
void acim_core_update(acim_core_t *s, const acim_core_conf_t *c,
		const acim_core_in_t *in, acim_core_out_t *out);

#endif /* ACIM_CORE_H_ */
