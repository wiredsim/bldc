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
 * ACIM indirect field-oriented control: rotor-flux current model, hybrid
 * voltage/current-model observer, flux state machine, slip clamp and faults.
 *
 * The rotor flux is tracked in amps (i_mr = psi_r / Lm), a convention also used by
 * ODrive (fw-v0.5.6 acim_estimator.cpp, MIT, Copyright (c) 2016-2018 ODrive Robotics).
 * No ODrive code is used; the equations are textbook IFOC. See DESIGN.md section 7.
 */

#include "acim_core.h"
#include <math.h>
#include <string.h>

#define ACIM_PI             3.14159265358979f
#define ACIM_TWO_PI         6.28318530717959f

// Encoder speed PLL and voltage-model PLL bandwidths [rad/s], and the sensorless
// speed-estimate filter [Hz]. Validated in documentation/acim/sim.
#define ACIM_ENC_PLL_BW     300.0f
#define ACIM_VM_PLL_BW      400.0f
#define ACIM_WR_FILTER_HZ   50.0f

// Fault timing [s]
#define ACIM_FLUX_ERR_TIME  0.1f
#define ACIM_SLIP_ERR_TIME  0.2f
#define ACIM_PLAUS_TIME     0.05f
#define ACIM_HANDOVER_BLANK 0.25f
#define ACIM_VM_GATE_TAU    0.05f  // s, filter on |vm_w| before it gates the plausibility checks

#ifdef ACIM_HOST_TEST
static void acim_sincos(float a, float *s, float *c) {
	*s = sinf(a);
	*c = cosf(a);
}
#else
void utils_fast_sincos_better(float angle, float *sin, float *cos);
#define acim_sincos utils_fast_sincos_better
#endif

// Same approximation as utils_fast_atan2(), kept here so the host test runs the
// exact arithmetic the firmware does.
static float acim_atan2(float y, float x) {
	float abs_y = fabsf(y) + 1e-20f;
	float angle;

	if (x >= 0.0f) {
		float r = (x - abs_y) / (x + abs_y);
		angle = ((0.1963f * r * r) - 0.9817f) * r + (ACIM_PI / 4.0f);
	} else {
		float r = (x + abs_y) / (abs_y - x);
		angle = ((0.1963f * r * r) - 0.9817f) * r + (3.0f * ACIM_PI / 4.0f);
	}

	if (angle != angle) {
		angle = 0.0f;
	}

	return y < 0.0f ? -angle : angle;
}

static float norm_angle(float a) {
	while (a > ACIM_PI) {
		a -= ACIM_TWO_PI;
	}
	while (a < -ACIM_PI) {
		a += ACIM_TWO_PI;
	}
	return a;
}

static float clampf(float x, float lim) {
	if (x > lim) {
		return lim;
	} else if (x < -lim) {
		return -lim;
	}
	return x;
}

void acim_core_reset(acim_core_t *s) {
	memset(s, 0, sizeof(acim_core_t));
	s->if_active = true;
	s->state = ACIM_STATE_OFF;
	s->fault = ACIM_FAULT_NONE;
}

// Rotate i_mr by the rotor electrical angle increment dth, then decay exactly toward i.
// Rotating by the measured angle increment (not a filtered speed * dt) keeps speed-filter
// lag out of the slip: summed increments are exact up to one encoder count.
static void current_model(acim_core_t *s, float ia, float ib, float dth) {
	// |dth| < 0.25 rad up to 600 Hz electrical at 15 kHz, so a short series is exact
	// to float precision.
	float d2 = dth * dth;
	float sn = dth * (1.0f - d2 * (1.0f / 6.0f) * (1.0f - d2 * (1.0f / 20.0f)));
	float cs = 1.0f - d2 * 0.5f * (1.0f - d2 * (1.0f / 12.0f));
	float a = cs * s->imr_a - sn * s->imr_b;
	float b = sn * s->imr_a + cs * s->imr_b;
	s->imr_a = a + s->k_decay * (ia - a);
	s->imr_b = b + s->k_decay * (ib - b);
}

// Hybrid flux observer: voltage model pulled toward the current model with a PI corrector
// (crossover obs_bw). Returns the voltage-model rotor-flux angle. In encoder mode the
// orientation comes from the current model and this is only a plausibility check.
static float observer(acim_core_t *s, const acim_core_conf_t *c, const acim_core_in_t *in, float dth_cm) {
	const float dt = in->dt;
	const float ia = in->i_alpha;
	const float ib = in->i_beta;
	const float lm = c->lm;
	const float lr = c->lm * c->lr_lm;
	const float sls = in->sigma_ls;
	const float wc = ACIM_TWO_PI * c->obs_bw;
	const float kp = 2.0f * wc;
	const float ki = wc * wc;

	current_model(s, ia, ib, dth_cm);
	float ps_cm_a = (lm / lr) * lm * s->imr_a + sls * ia;
	float ps_cm_b = (lm / lr) * lm * s->imr_b + sls * ib;

	float ea = ps_cm_a - s->ps_a;
	float eb = ps_cm_b - s->ps_b;
	s->ci_a += ki * ea * dt;
	s->ci_b += ki * eb * dt;
	float ua = kp * ea + s->ci_a;
	float ub = kp * eb + s->ci_b;
	s->ps_a += (in->v_alpha_prev - in->rs * ia + ua) * dt;
	s->ps_b += (in->v_beta_prev - in->rs * ib + ub) * dt;

	float pra = (lr / lm) * (s->ps_a - sls * ia);
	float prb = (lr / lm) * (s->ps_b - sls * ib);
	s->psr_vm_a = pra;
	s->psr_vm_b = prb;
	float th = acim_atan2(prb, pra);
	s->vm_th_raw_prev = s->vm_th_raw;
	s->vm_th_raw = th;

	// PLL on the voltage-model flux angle: stator (synchronous) frequency
	const float kpp = 2.0f * ACIM_VM_PLL_BW;
	const float kip = ACIM_VM_PLL_BW * ACIM_VM_PLL_BW;
	float err = norm_angle(th - s->vm_th);
	s->vm_th = norm_angle(s->vm_th + (s->vm_w_int + kpp * err) * dt);
	s->vm_w_int += kip * err * dt;
	s->vm_w = s->vm_w_int + kpp * err;

	float mag2 = pra * pra + prb * prb;
	float w_sl = 0.0f;
	if (mag2 > 1e-12f) {
		w_sl = lm * (pra * ib - prb * ia) / (c->tau_r * mag2);
	}
	s->w_sl_obs = w_sl;

	if (c->sensorless && s->state >= ACIM_STATE_RUN && s->state <= ACIM_STATE_HOLD && !s->if_active) {
		float w_r = s->vm_w - w_sl;
		float k = dt * ACIM_TWO_PI * ACIM_WR_FILTER_HZ;
		if (k > 1.0f) {
			k = 1.0f;
		}
		s->wr_hat += (w_r - s->wr_hat) * k;
	}

	return th;
}

static void set_fault(acim_core_t *s, acim_core_out_t *out, acim_fault_t f) {
	s->state = ACIM_STATE_FAULT;
	s->fault = f;
	out->fault = f;
	out->id_ref = 0.0f;
	out->iq_ref = 0.0f;
}

void acim_core_update(acim_core_t *s, const acim_core_conf_t *c,
		const acim_core_in_t *in, acim_core_out_t *out) {
	const float dt = in->dt;
	const float ia = in->i_alpha;
	const float ib = in->i_beta;
	bool run_state = s->state == ACIM_STATE_RUN || s->state == ACIM_STATE_HOLD;

	out->fault = ACIM_FAULT_NONE;
	out->id_ref = 0.0f;
	out->iq_ref = 0.0f;

	if (dt != s->k_dt || c->tau_r != s->k_tau) {
		s->k_dt = dt;
		s->k_tau = c->tau_r;
		s->k_decay = 1.0f - expf(-dt / fmaxf(c->tau_r, 1e-4f));
	}

	// A fault is latched until the modulation has stopped
	if (s->state == ACIM_STATE_FAULT && !in->driven) {
		s->state = ACIM_STATE_OFF;
	}

	// 1. Rotor speed and flux angle
	float wr, dth, theta;
	if (!c->sensorless) {
		if (!in->enc_valid) {
			s->wr = 0.0f;
			out->phase = s->theta;
			if (in->driven && s->state != ACIM_STATE_FAULT) {
				set_fault(s, out, ACIM_FAULT_ENCODER);
			}
			return;
		}

		if (!s->enc_init) {
			s->pll_th = in->enc_phase;
			s->enc_prev = in->enc_phase;
			s->enc_init = true;
		}
		dth = norm_angle(in->enc_phase - s->enc_prev);
		s->enc_prev = in->enc_phase;
		// Same structure as foc_pll_run(). pll_w alone lags by 2a/wn under
		// acceleration; pll_w + kp * err does not.
		const float kp = 2.0f * ACIM_ENC_PLL_BW;
		const float ki = ACIM_ENC_PLL_BW * ACIM_ENC_PLL_BW;
		float err = norm_angle(in->enc_phase - s->pll_th);
		s->pll_th = norm_angle(s->pll_th + (s->pll_w + kp * err) * dt);
		s->pll_w += ki * err * dt;
		wr = s->pll_w + kp * err;

		observer(s, c, in, dth);
		theta = acim_atan2(s->imr_b, s->imr_a);
		s->rotor_phase = in->enc_phase;
	} else {
		if (!run_state) {
			// Standstill (flux build) or off: nothing to observe, assume no rotation
			s->wr_hat = 0.0f;
		} else if (s->if_active) {
			// I/f: the rotor is assumed to follow the field
			s->wr_hat = ACIM_TWO_PI * s->if_f;
		}
		wr = s->wr_hat;

		if (run_state && !s->if_active) {
			// Rotor angle increment implied by the observer: flux angle increment minus slip
			dth = norm_angle(s->vm_th_raw - s->vm_th_raw_prev) - s->w_sl_obs * dt;
		} else {
			dth = wr * dt;
		}
		theta = observer(s, c, in, dth);
		s->rotor_phase = norm_angle(s->rotor_phase + dth);
	}
	s->wr = wr;

	const float imr = sqrtf(s->imr_a * s->imr_a + s->imr_b * s->imr_b);
	s->imr = imr;
	out->phase = s->theta;

	if (s->state == ACIM_STATE_FAULT) {
		return;
	}

	// 2. NaN / magnitude plausibility. i_mr is a low-pass of the measured current, so it
	// can only exceed the largest current the loop may drive through a numerical fault.
	if (!(imr == imr) || imr > 1.5f * c->current_max) {
		acim_core_reset(s);
		set_fault(s, out, ACIM_FAULT_FLUX);
		return;
	}

	// 3. State machine
	const float build_t = c->flux_build_time > 0.0f ? c->flux_build_time : 3.0f * c->tau_r;
	const bool request = fabsf(in->iq_request) > in->iq_request_min;

	if (!in->driven) {
		s->state = ACIM_STATE_OFF;
		s->t_state = 0.0f;
		s->hold_left = 0.0f;
		s->if_active = true;
		s->if_f = 0.0f;
		s->if_th = theta;
		s->theta = theta;
		s->id = s->iq = s->id_ref = s->iq_ref = 0.0f;
		s->w_s = wr;
		s->slip_hz = 0.0f;
		out->phase = theta;
		// Keep the voltage model seeded from the current model so a restart onto a
		// spinning rotor (encoder mode) has a consistent plausibility check.
		const float lm = c->lm;
		s->ps_a = lm / c->lr_lm * s->imr_a;
		s->ps_b = lm / c->lr_lm * s->imr_b;
		s->ci_a = s->ci_b = 0.0f;
		s->flux_err_t = s->slip_err_t = s->slip_plaus_t = 0.0f;
		s->vm_w_gate = 0.0f;
		return;
	}

	if (s->state == ACIM_STATE_OFF) {
		s->state = ACIM_STATE_FLUXING;
		s->t_state = 0.0f;
		s->t_handover = 0.0f;
	}
	s->t_state += dt;

	if (s->state == ACIM_STATE_FLUXING && imr >= 0.9f * c->id_mag && s->t_state >= build_t) {
		s->state = ACIM_STATE_RUN;
	}

	if (request || s->state == ACIM_STATE_FLUXING) {
		s->hold_left = c->flux_hold_time;
	} else if (s->hold_left > 0.0f) {
		s->hold_left -= dt;
	}

	if (s->state == ACIM_STATE_RUN || s->state == ACIM_STATE_HOLD) {
		s->state = request ? ACIM_STATE_RUN : ACIM_STATE_HOLD;
	}
	run_state = s->state == ACIM_STATE_RUN || s->state == ACIM_STATE_HOLD;

	// 4. Sensorless low speed: I/f instead of the observer angle
	if (c->sensorless) {
		const float ws_obs = s->vm_w;
		if (s->if_active) {
			if (run_state && request) {
				s->if_f += (in->iq_request > 0.0f ? 1.0f : -1.0f) * c->sl_if_ramp * dt;
			} else if (run_state) {
				// No torque requested: bring the field to a stop instead of dragging
				// the rotor along at the last I/f frequency
				float step = c->sl_if_ramp * dt;
				if (fabsf(s->if_f) <= step) {
					s->if_f = 0.0f;
				} else {
					s->if_f -= s->if_f > 0.0f ? step : -step;
				}
			}
			s->if_th = norm_angle(s->if_th + ACIM_TWO_PI * s->if_f * dt);

			if (fabsf(s->if_f) >= 1.2f * c->sl_min_hz && fabsf(ws_obs) > ACIM_TWO_PI * c->sl_min_hz) {
				// Hand over to the observer and re-seed the current model from it
				s->if_active = false;
				s->imr_a = s->psr_vm_a / c->lm;
				s->imr_b = s->psr_vm_b / c->lm;
				s->t_handover = 0.0f;
			}
		} else if (fabsf(ws_obs) < ACIM_TWO_PI * 0.8f * c->sl_min_hz) {
			s->if_active = true;
			s->if_th = theta;
			s->if_f = ws_obs / ACIM_TWO_PI;
		}

		if (s->if_active) {
			theta = s->if_th;
		}
	}

	// The same angle is used for the Park transform of the sampled currents and the
	// inverse Park of the output voltage. It must be the angle at the sample instant:
	// advancing it (as the PM observer path does) biases the measured id/iq and
	// therefore the flux. The output delay is absorbed by the PI integrators.
	const float phase = theta;

	// 5. Current references
	float id_ref = c->id_mag;
	float iq_ref = 0.0f;
	s->iq_unclamped = 0.0f;
	if (run_state) {
		const float imr_eff = (c->sensorless && s->if_active) ? c->id_mag : imr;
		const float iq_slip_max = ACIM_TWO_PI * c->slip_max_hz * c->tau_r * imr_eff;
		s->iq_unclamped = in->iq_request;
		iq_ref = clampf(in->iq_request, iq_slip_max);

		// Flux-collapse detector: torque wanted but flux far too low for it
		if (fabsf(in->iq_request) > c->fault_slip_fac * iq_slip_max &&
				fabsf(in->iq_request) > 0.2f * c->id_mag) {
			s->slip_err_t += dt;
		} else {
			s->slip_err_t = 0.0f;
		}

		if (s->slip_err_t > ACIM_SLIP_ERR_TIME) {
			set_fault(s, out, ACIM_FAULT_SLIP);
			return;
		}
	}

	if (c->sensorless && s->if_active && run_state) {
		// I/f: one rotating current vector along the I/f angle. The cage follows it like
		// an induction motor on a rotating field, which is self-damping.
		id_ref = sqrtf(c->id_mag * c->id_mag + in->iq_request * in->iq_request);
		iq_ref = 0.0f;
	}

	const float imax = c->current_max;
	if (id_ref > imax) {
		id_ref = imax;
	}
	iq_ref = clampf(iq_ref, sqrtf(fmaxf(imax * imax - id_ref * id_ref, 0.0f)));

	// 6. Measured currents in the flux frame, stator frequency for decoupling
	float sn, cs;
	acim_sincos(phase, &sn, &cs);
	const float i_d = cs * ia + sn * ib;
	const float i_q = cs * ib - sn * ia;

	float w_s;
	if (c->sensorless && s->if_active) {
		w_s = ACIM_TWO_PI * s->if_f;
	} else {
		w_s = wr + (imr > 1.0f ? i_q / (c->tau_r * imr) : 0.0f);
	}
	s->w_s = w_s;

	// 7. Flux plausibility: voltage model vs current model, once the stator frequency is
	// high enough for the voltage model to carry information (both modes). The gate uses the
	// voltage model's own speed because it must not depend on the encoder it checks (a lost
	// encoder signal drops w_s too). It is low-passed: at low speed vm_w is noise with brief
	// spikes past the threshold, which otherwise run these checks on a meaningless model.
	s->t_handover += dt;
	s->vm_w_gate += (fabsf(s->vm_w) - s->vm_w_gate) * fminf(1.0f, dt / ACIM_VM_GATE_TAU);
	const bool vm_valid = s->vm_w_gate > ACIM_TWO_PI * c->sl_min_hz &&
			s->t_handover > ACIM_HANDOVER_BLANK && !(c->sensorless && s->if_active);

	if (vm_valid && run_state) {
		const float pcm_a = c->lm * s->imr_a;
		const float pcm_b = c->lm * s->imr_b;
		const float m = sqrtf(pcm_a * pcm_a + pcm_b * pcm_b);
		const float ea = s->psr_vm_a - pcm_a;
		const float eb = s->psr_vm_b - pcm_b;
		const float err = sqrtf(ea * ea + eb * eb);

		if (m > 1e-6f && err / m > c->fault_flux_err) {
			s->flux_err_t += dt;
		} else {
			s->flux_err_t = 0.0f;
		}

		if (s->flux_err_t > ACIM_FLUX_ERR_TIME) {
			set_fault(s, out, ACIM_FAULT_FLUX);
			return;
		}

		// Encoder mode: the slip seen by the voltage model (stator frequency - encoder
		// speed) must match the slip the current model commands. A stuck, slipping or
		// wrongly scaled encoder breaks this long before the flux error grows.
		if (!c->sensorless) {
			const float slip_seen = s->vm_w - wr;
			const float slip_cmd = w_s - wr;
			const float tol = ACIM_TWO_PI * fmaxf(c->slip_max_hz, 0.25f * fabsf(s->vm_w) / ACIM_TWO_PI);
			if (fabsf(slip_seen - slip_cmd) > tol) {
				s->slip_plaus_t += dt;
			} else {
				s->slip_plaus_t = 0.0f;
			}

			if (s->slip_plaus_t > ACIM_PLAUS_TIME) {
				set_fault(s, out, ACIM_FAULT_SLIP);
				return;
			}
		}
	}

	s->theta = theta;
	s->id = i_d;
	s->iq = i_q;
	s->id_ref = id_ref;
	s->iq_ref = iq_ref;
	s->slip_hz = (w_s - wr) / ACIM_TWO_PI;

	out->phase = phase;
	out->id_ref = id_ref;
	out->iq_ref = iq_ref;
}
