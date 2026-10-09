/*
 * Host build of motor/acim_core.c for check_c_core.py. Flat functions so ctypes does not
 * need to mirror the structs.
 */

#define ACIM_HOST_TEST
#include "../../../motor/acim_core.c"

static acim_core_t S;
static acim_core_conf_t C;
static float I_AVAIL = 0.0f;

void h_set_i_avail(float a) {
	I_AVAIL = a;
}

void h_reset(void) {
	acim_core_reset(&S);
}

void h_conf(int sensorless, float id_mag, float tau_r, float lm, float lr_lm, float slip_max_hz,
		float flux_build_time, float flux_hold_time, float current_max, float sl_min_hz,
		float sl_if_ramp, float obs_bw, float fault_flux_err, float fault_slip_fac) {
	C.sensorless = sensorless != 0;
	C.id_mag = id_mag;
	C.tau_r = tau_r;
	C.lm = lm;
	C.lr_lm = lr_lm;
	C.slip_max_hz = slip_max_hz;
	C.flux_build_time = flux_build_time;
	C.flux_hold_time = flux_hold_time;
	C.current_max = current_max;
	C.sl_min_hz = sl_min_hz;
	C.sl_if_ramp = sl_if_ramp;
	C.obs_bw = obs_bw;
	C.fault_flux_err = fault_flux_err;
	C.fault_slip_fac = fault_slip_fac;
}

// res: phase, id_ref, iq_ref, fault, state, imr, w_s, wr, theta, id, iq, slip_hz, if_active, imr_a, imr_b
void h_update(float dt, int driven, float ia, float ib, float va_prev, float vb_prev,
		float enc_phase, int enc_valid, float iq_request, float iq_request_min,
		float rs, float sigma_ls, float *res) {
	acim_core_in_t in;
	in.dt = dt;
	in.driven = driven != 0;
	in.i_alpha = ia;
	in.i_beta = ib;
	in.v_alpha_prev = va_prev;
	in.v_beta_prev = vb_prev;
	in.enc_phase = enc_phase;
	in.enc_valid = enc_valid != 0;
	in.iq_request = iq_request;
	in.iq_request_min = iq_request_min;
	in.i_avail = I_AVAIL;  // 0 = not limited (the default)
	in.rs = rs;
	in.sigma_ls = sigma_ls;

	acim_core_out_t out;
	acim_core_update(&S, &C, &in, &out);

	res[0] = out.phase;
	res[1] = out.id_ref;
	res[2] = out.iq_ref;
	res[3] = (float)out.fault;
	res[4] = (float)S.state;
	res[5] = S.imr;
	res[6] = S.w_s;
	res[7] = S.wr;
	res[8] = S.theta;
	res[9] = S.id;
	res[10] = S.iq;
	res[11] = S.slip_hz;
	res[12] = S.if_active ? 1.0f : 0.0f;
	res[13] = S.imr_a;
	res[14] = S.imr_b;
}
