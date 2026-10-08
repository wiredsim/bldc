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
 * Firmware glue for the ACIM mode: configuration page and storage, ISR wrappers around
 * acim_core, fault reporting, terminal commands, plotting and LispBM extensions.
 */

#include "acim.h"
#include "acim_core.h"
#include "acim_confgen.h"

#include "ch.h"
#include "hal.h"
#include "mc_interface.h"
#include "conf_general.h"
#include "conf_custom.h"
#include "commands.h"
#include "terminal.h"
#include "timeout.h"
#include "encoder.h"
#include "utils_math.h"
#include "utils_sys.h"

#ifdef USE_LISPBM
#include "lispif.h"
#include "lispbm.h"
#endif

#include <string.h>
#include <stdio.h>
#include <math.h>

// |iq request| below this counts as no torque request [A]
#define ACIM_IQ_REQUEST_MIN		1.0

// Hardware EEPROM variables at the top of the range, away from the few hwconfs that use the
// low addresses: signature first, then the config struct
#define ACIM_CONF_WORDS			((sizeof(acim_config) + 3) / 4)
#define ACIM_EEPROM_BASE		(EEPROM_VARS_HW - 1 - ACIM_CONF_WORDS)
_Static_assert(ACIM_CONF_WORDS + 1 <= EEPROM_VARS_HW, "ACIM config does not fit the HW EEPROM variables");

// Private variables
static acim_config m_conf;
static acim_core_conf_t m_core_conf;
static acim_core_t m_core;
static volatile bool m_active = false;
static volatile bool m_fault_sent = false;
static volatile bool m_plot_en = false;
static float m_va_now = 0.0, m_vb_now = 0.0;
static bool m_init_done = false;

static THD_WORKING_AREA(acim_thread_wa, 1024);
static THD_FUNCTION(acim_thread, arg);

// Private functions
static void conf_apply(const acim_config *conf);
static void terminal_status(int argc, const char **argv);
static void terminal_plot(int argc, const char **argv);
static void terminal_enc_check(int argc, const char **argv);
#ifdef USE_LISPBM
static void load_extensions(bool main_found);
#endif

static const char *state_to_string(acim_state_t s) {
	switch (s) {
	case ACIM_STATE_OFF: return "OFF";
	case ACIM_STATE_FLUXING: return "FLUXING";
	case ACIM_STATE_RUN: return "RUN";
	case ACIM_STATE_HOLD: return "HOLD";
	case ACIM_STATE_FAULT: return "FAULT";
	}
	return "?";
}

static const char *fault_to_string(acim_fault_t f) {
	switch (f) {
	case ACIM_FAULT_NONE: return "none";
	case ACIM_FAULT_FLUX: return "ACIM_FLUX";
	case ACIM_FAULT_SLIP: return "ACIM_SLIP";
	case ACIM_FAULT_ENCODER: return "ENCODER";
	}
	return "?";
}

// Custom config page

static int get_cfg(uint8_t *buffer, bool is_default) {
	acim_config c = m_conf;
	if (is_default) {
		acim_confgen_set_defaults(&c);
	}
	return acim_confgen_serialize(buffer, &c);
}

static bool set_cfg(uint8_t *buffer) {
	acim_config c = m_conf;
	if (!acim_confgen_deserialize(buffer, &c)) {
		return false;
	}

	// Same as writing the motor configuration: the motor is released before anything changes
	mc_interface_ignore_input_both(5000);
	mc_interface_release_motor_override_both();
	if (!mc_interface_wait_for_motor_release_both(3.0)) {
		mc_interface_ignore_input_both(100);
		commands_printf("ACIM: motor did not stop, config not applied");
		return false;
	}

	conf_apply(&c);

	uint32_t words[ACIM_CONF_WORDS];
	memset(words, 0, sizeof(words));
	memcpy(words, &m_conf, sizeof(acim_config));

	bool ok = true;
	for (unsigned int i = 0;i < ACIM_CONF_WORDS;i++) {
		eeprom_var v;
		v.as_u32 = words[i];
		if (!conf_general_store_eeprom_var_hw(&v, ACIM_EEPROM_BASE + 1 + i)) {
			ok = false;
			break;
		}
	}

	if (ok) {
		eeprom_var v;
		v.as_u32 = ACIM_CONF_SIGNATURE;
		ok = conf_general_store_eeprom_var_hw(&v, ACIM_EEPROM_BASE);
	}

	if (!ok) {
		commands_printf("ACIM: storing the config failed, it is active until reboot");
	}

	mc_interface_ignore_input_both(100);
	return true;
}

static int get_cfg_xml(uint8_t **buffer) {
	*buffer = (uint8_t*)acim_conf_xml;
	return ACIM_CONF_XML_SIZE;
}

static void conf_load(void) {
	acim_config c;
	acim_confgen_set_defaults(&c);

	eeprom_var v;
	if (conf_general_read_eeprom_var_hw(&v, ACIM_EEPROM_BASE) && v.as_u32 == ACIM_CONF_SIGNATURE) {
		uint32_t words[ACIM_CONF_WORDS];
		bool ok = true;
		for (unsigned int i = 0;i < ACIM_CONF_WORDS;i++) {
			if (!conf_general_read_eeprom_var_hw(&v, ACIM_EEPROM_BASE + 1 + i)) {
				ok = false;
				break;
			}
			words[i] = v.as_u32;
		}

		if (ok) {
			memcpy(&c, words, sizeof(acim_config));
		}
	}

	conf_apply(&c);
}

static float clamp_range(float x, float min, float max) {
	if (!(x == x)) {
		return min;
	}
	utils_truncate_number(&x, min, max);
	return x;
}

static void conf_apply(const acim_config *conf) {
	acim_config c = *conf;

	// Keep the estimator out of divisions by zero whatever was written
	c.id_mag = clamp_range(c.id_mag, 0.0, 400.0);
	c.tau_r = clamp_range(c.tau_r, 0.001, 5.0);
	c.lm = clamp_range(c.lm, 1e-6, 0.1);
	c.lr_lm = clamp_range(c.lr_lm, 1.0, 2.0);
	c.current_max = clamp_range(c.current_max, 0.0, 500.0);
	c.slip_max = clamp_range(c.slip_max, 0.1, 50.0);
	c.flux_build_time = clamp_range(c.flux_build_time, 0.0, 5.0);
	c.flux_hold_time = clamp_range(c.flux_hold_time, 0.0, 60.0);
	c.sl_min_hz = clamp_range(c.sl_min_hz, 1.0, 100.0);
	c.sl_if_ramp = clamp_range(c.sl_if_ramp, 0.1, 500.0);
	c.obs_bw = clamp_range(c.obs_bw, 0.1, 50.0);
	c.fault_flux_err = clamp_range(c.fault_flux_err, 0.05, 5.0);
	c.fault_slip_fac = clamp_range(c.fault_slip_fac, 1.0, 100.0);
	if (c.speed_src > ACIM_SPEED_SRC_SENSORLESS) {
		c.speed_src = ACIM_SPEED_SRC_ENCODER;
	}

	acim_core_conf_t cc;
	cc.sensorless = c.speed_src == ACIM_SPEED_SRC_SENSORLESS;
	cc.id_mag = c.id_mag;
	cc.tau_r = c.tau_r;
	cc.lm = c.lm;
	cc.lr_lm = c.lr_lm;
	cc.slip_max_hz = c.slip_max;
	cc.flux_build_time = c.flux_build_time;
	cc.flux_hold_time = c.flux_hold_time;
	cc.current_max = c.current_max;
	cc.sl_min_hz = c.sl_min_hz;
	cc.sl_if_ramp = c.sl_if_ramp;
	cc.obs_bw = c.obs_bw;
	cc.fault_flux_err = c.fault_flux_err;
	cc.fault_slip_fac = c.fault_slip_fac;

	utils_sys_lock_cnt();
	m_conf = c;
	m_core_conf = cc;
	acim_core_reset(&m_core);
	utils_sys_unlock_cnt();
}

// Public functions

void acim_init(void) {
	// mcpwm_foc_init runs again on every motor config write; only do this once
	if (m_init_done) {
		return;
	}

	acim_core_reset(&m_core);
	conf_load();

	conf_custom_add_config(get_cfg, set_cfg, get_cfg_xml);

	terminal_register_command_callback(
			"acim_status",
			"Print the ACIM mode state, rotor flux, slip and speeds",
			0,
			terminal_status);

	terminal_register_command_callback(
			"acim_plot",
			"Stream ACIM flux, slip and speeds to the experiment plot",
			"[0/1]",
			terminal_plot);

	terminal_register_command_callback(
			"acim_enc_check",
			"Spin the field open-loop both ways and check encoder direction and ratio. The motor will turn.",
			"[current] [erpm]",
			terminal_enc_check);

#ifdef USE_LISPBM
	lispif_add_ext_load_callback(load_extensions);
#endif

	chThdCreateStatic(acim_thread_wa, sizeof(acim_thread_wa), LOWPRIO, acim_thread, NULL);

	m_init_done = true;
}

bool acim_active(bool is_second_motor) {
	return m_active && !is_second_motor;
}

bool acim_isr_undriven(bool is_second_motor, const mc_configuration *conf, float dt,
		float enc_phase, bool enc_valid, float rs, float *phase) {
	if (is_second_motor) {
		return false;
	}

	bool active = m_conf.enable && conf->motor_type == MOTOR_TYPE_FOC &&
			conf->foc_sensor_mode == FOC_SENSOR_MODE_SENSORLESS;

	if (active && !m_active) {
		acim_core_reset(&m_core);
	}

	m_active = active;
	m_fault_sent = false;
	m_va_now = 0.0;
	m_vb_now = 0.0;

	if (!active) {
		return false;
	}

	acim_core_in_t in;
	memset(&in, 0, sizeof(in));
	in.dt = dt;
	in.driven = false;
	in.enc_phase = enc_phase;
	in.enc_valid = enc_valid;
	in.iq_request_min = ACIM_IQ_REQUEST_MIN;
	in.rs = rs;
	in.sigma_ls = conf->foc_motor_l;

	acim_core_out_t out;
	acim_core_update(&m_core, &m_core_conf, &in, &out);
	*phase = out.phase;

	return true;
}

void acim_isr_driven(bool is_second_motor, const mc_configuration *conf, float dt, bool takeover,
		float i_alpha, float i_beta, float v_alpha, float v_beta,
		float enc_phase, bool enc_valid, float rs, float iq_request,
		float *phase, float *id_set, float *iq_set) {
	if (is_second_motor || !m_active) {
		return;
	}

	// v_alpha/v_beta is what the previous ISR computed and is being applied now. The
	// period that just ended used the value from the ISR before that.
	acim_core_in_t in;
	in.dt = dt;
	in.driven = takeover;
	in.i_alpha = i_alpha;
	in.i_beta = i_beta;
	in.v_alpha_prev = m_va_now;
	in.v_beta_prev = m_vb_now;
	in.enc_phase = enc_phase;
	in.enc_valid = enc_valid;
	in.iq_request = iq_request;
	in.iq_request_min = ACIM_IQ_REQUEST_MIN;
	in.rs = rs;
	in.sigma_ls = conf->foc_motor_l;
	m_va_now = v_alpha;
	m_vb_now = v_beta;

	acim_core_out_t out;
	acim_core_update(&m_core, &m_core_conf, &in, &out);

	if (out.fault != ACIM_FAULT_NONE && !m_fault_sent) {
		m_fault_sent = true;
		mc_fault_code code = FAULT_CODE_ENCODER_FAULT;
		if (out.fault == ACIM_FAULT_FLUX) {
			code = FAULT_CODE_ACIM_FLUX;
		} else if (out.fault == ACIM_FAULT_SLIP) {
			code = FAULT_CODE_ACIM_SLIP;
		}
		mc_interface_fault_stop(code, is_second_motor, true);
	}

	if (takeover) {
		*phase = out.phase;
		*id_set = out.id_ref;
		*iq_set = out.iq_ref;
	}
}

float acim_rotor_phase(bool is_second_motor) {
	(void)is_second_motor;
	return m_core.rotor_phase;
}

void acim_decoupling(bool is_second_motor, const mc_configuration *conf, float id, float iq,
		float *dec_vd, float *dec_vq, float *dec_bemf) {
	(void)is_second_motor;
	const float w_s = m_core.w_s;
	const float l = conf->foc_motor_l;
	const float bemf = w_s * m_conf.lm / m_conf.lr_lm * m_core.imr;

	switch (conf->foc_cc_decoupling) {
	case FOC_CC_DECOUPLING_CROSS:
		*dec_vd = iq * w_s * l;
		*dec_vq = id * w_s * l;
		break;

	case FOC_CC_DECOUPLING_BEMF:
		*dec_bemf = bemf;
		break;

	case FOC_CC_DECOUPLING_CROSS_BEMF:
		*dec_vd = iq * w_s * l;
		*dec_vq = id * w_s * l;
		*dec_bemf = bemf;
		break;

	default:
		break;
	}
}

float acim_bemf_q(bool is_second_motor, const mc_configuration *conf) {
	(void)is_second_motor;
	if (conf->foc_cc_decoupling == FOC_CC_DECOUPLING_BEMF ||
			conf->foc_cc_decoupling == FOC_CC_DECOUPLING_CROSS_BEMF) {
		return m_core.w_s * m_conf.lm / m_conf.lr_lm * m_core.imr;
	}
	return 0.0;
}

float acim_hold_time_left(bool is_second_motor) {
	(void)is_second_motor;
	acim_state_t s = m_core.state;
	if (s == ACIM_STATE_FLUXING || s == ACIM_STATE_RUN || s == ACIM_STATE_HOLD) {
		return m_core.hold_left;
	}
	return 0.0;
}

// Thread: keeps the config page registered and streams the plot

static THD_FUNCTION(acim_thread, arg) {
	(void)arg;
	chRegSetThreadName("ACIM");

	bool plot_started = false;
	float plot_t = 0.0;
	int div = 0;

	for (;;) {
		// A LispBM package may replace the single custom config page and clear it when
		// it stops. Put the ACIM page back whenever no page is registered.
		if (++div >= 100) {
			div = 0;
			if (conf_custom_cfg_num() == 0) {
				conf_custom_add_config(get_cfg, set_cfg, get_cfg_xml);
			}
		}

		if (m_plot_en) {
			if (!plot_started) {
				plot_started = true;
				plot_t = 0.0;
				commands_init_plot("Time (s)", "Value");
				commands_plot_add_graph("i_mr (A)");
				commands_plot_add_graph("Slip (Hz)");
				commands_plot_add_graph("Rotor (Hz el)");
				commands_plot_add_graph("Stator (Hz el)");
				commands_plot_add_graph("Id (A)");
				commands_plot_add_graph("Iq (A)");
				commands_plot_add_graph("State");
			}

			const float vals[7] = {
					m_core.imr, m_core.slip_hz, m_core.wr / (2.0 * M_PI),
					m_core.w_s / (2.0 * M_PI), m_core.id, m_core.iq, (float)m_core.state
			};

			for (int i = 0;i < 7;i++) {
				commands_plot_set_graph(i);
				commands_send_plot_points(plot_t, vals[i]);
			}
			plot_t += 0.02;
			chThdSleepMilliseconds(20);
		} else {
			plot_started = false;
			chThdSleepMilliseconds(10);
		}
	}
}

// Terminal commands

static void terminal_status(int argc, const char **argv) {
	(void)argc; (void)argv;

	const volatile mc_configuration *mc = mc_interface_get_configuration();
	acim_core_t s = m_core;
	const float pp = m_conf.speed_src == ACIM_SPEED_SRC_ENCODER ?
			mc->foc_encoder_ratio : (float)mc->si_motor_poles / 2.0;

	commands_printf("ACIM mode: %s, %s",
			m_conf.enable ? "enabled" : "disabled", m_active ? "active" : "not active");
	if (m_conf.enable && !m_active) {
		if (mc->motor_type != MOTOR_TYPE_FOC || mc->foc_sensor_mode != FOC_SENSOR_MODE_SENSORLESS) {
			commands_printf("  Needs Motor Type FOC and Sensor Mode Sensorless in the motor config");
		} else {
			commands_printf("  Takes effect when the motor is stopped");
		}
	}
	commands_printf("Speed source: %s",
			m_conf.speed_src == ACIM_SPEED_SRC_ENCODER ? "encoder" : "sensorless");
	commands_printf("State: %s, last fault: %s", state_to_string(s.state), fault_to_string(s.fault));
	commands_printf("i_mr: %.2f A (target %.2f A), psi_r: %.3f mWb",
			(double)s.imr, (double)m_conf.id_mag, (double)(s.imr * m_conf.lm * 1e3));
	commands_printf("Slip: %.2f Hz, rotor: %.2f Hz el (%.0f rpm at %.0f pole pairs), stator: %.2f Hz el",
			(double)s.slip_hz, (double)(s.wr / (2.0 * M_PI)),
			(double)(pp > 0.0 ? s.wr / (2.0 * M_PI) * 60.0 / pp : 0.0), (double)pp,
			(double)(s.w_s / (2.0 * M_PI)));
	commands_printf("Id: %.2f A (ref %.2f), Iq: %.2f A (ref %.2f, requested %.2f)",
			(double)s.id, (double)s.id_ref, (double)s.iq, (double)s.iq_ref, (double)s.iq_unclamped);
	commands_printf("Voltage model flux: %.3f mWb, stator freq %.2f Hz",
			(double)(sqrtf(SQ(s.psr_vm_a) + SQ(s.psr_vm_b)) * 1e3), (double)(s.vm_w / (2.0 * M_PI)));
	if (m_conf.speed_src == ACIM_SPEED_SRC_SENSORLESS) {
		commands_printf("I/f: %s, %.2f Hz", s.if_active ? "active" : "off", (double)s.if_f);
	}
	commands_printf("Config: Id_mag %.1f A, tau_r %.1f ms, Lm %.1f uH, Lr/Lm %.3f, I_max %.1f A, slip_max %.1f Hz",
			(double)m_conf.id_mag, (double)(m_conf.tau_r * 1e3), (double)(m_conf.lm * 1e6),
			(double)m_conf.lr_lm, (double)m_conf.current_max, (double)m_conf.slip_max);
	commands_printf(" ");
}

static void terminal_plot(int argc, const char **argv) {
	if (argc == 2) {
		int en = -1;
		sscanf(argv[1], "%d", &en);
		if (en == 0 || en == 1) {
			m_plot_en = en == 1;
			commands_printf(m_plot_en ? "ACIM plot on" : "ACIM plot off");
			return;
		}
	}
	commands_printf("Usage: acim_plot [0/1]");
}

static void terminal_enc_check(int argc, const char **argv) {
	const volatile mc_configuration *mc = mc_interface_get_configuration();
	float current = fminf(m_conf.id_mag, m_conf.current_max);
	float erpm = 120.0;

	if (argc >= 2) {
		sscanf(argv[1], "%f", &current);
	}
	if (argc >= 3) {
		sscanf(argv[2], "%f", &erpm);
	}

	if (!encoder_is_configured()) {
		commands_printf("No encoder configured on the sensor port\n");
		return;
	}

	if (!(current > 0.0 && current <= mc->l_current_max && erpm > 0.0 && erpm <= 3000.0)) {
		commands_printf("Current must be in (0, %.1f] A and ERPM in (0, 3000]\n", (double)mc->l_current_max);
		return;
	}

	const float field_turns = 3.0;
	const float settle_turns = 1.0;
	const float t_turn = 60.0 / erpm;

	commands_printf("ACIM encoder check: %.1f A, %.0f ERPM field. The motor will turn both ways.",
			(double)current, (double)erpm);

	float ratio[2] = {0.0, 0.0};
	float mech[2] = {0.0, 0.0};

	for (int d = 0;d < 2;d++) {
		const float dir = d == 0 ? 1.0 : -1.0;
		float acc = 0.0;
		float prev = encoder_read_deg();
		const int ms_settle = (int)(settle_turns * t_turn * 1000.0);
		const int ms_total = (int)((settle_turns + field_turns) * t_turn * 1000.0);

		for (int ms = 0;ms < ms_total;ms++) {
			timeout_reset();
			mc_interface_set_openloop_current(current, dir * erpm);
			chThdSleepMilliseconds(1);

			float now = encoder_read_deg();
			float diff = now - prev;
			prev = now;
			while (diff > 180.0) {
				diff -= 360.0;
			}
			while (diff < -180.0) {
				diff += 360.0;
			}
			if (ms >= ms_settle) {
				acc += diff;
			}

			if (mc_interface_get_fault() != FAULT_CODE_NONE) {
				mc_interface_release_motor();
				commands_printf("Aborted: %s\n", mc_interface_fault_to_string(mc_interface_get_fault()));
				return;
			}
		}

		mc_interface_release_motor();
		chThdSleepMilliseconds(500);

		float mech_turns = acc / 360.0;
		if (mc->foc_encoder_inverted) {
			mech_turns = -mech_turns;
		}
		mech[d] = mech_turns;
		ratio[d] = mech_turns * mc->foc_encoder_ratio / (dir * field_turns);
	}

	bool ok = true;
	for (int d = 0;d < 2;d++) {
		const char *name = d == 0 ? "Forward" : "Reverse";
		if (fabsf(mech[d]) < 0.02) {
			commands_printf("%s: rotor did not follow the field. Try more current.", name);
			ok = false;
			continue;
		}

		const float pp_implied = (d == 0 ? field_turns : -field_turns) / mech[d];
		commands_printf("%s: rotor/field %.3f (expect 0.9-1.0 at no load), implied pole pairs %.2f (configured %.0f)",
				name, (double)ratio[d], (double)pp_implied, (double)mc->foc_encoder_ratio);

		if (ratio[d] < 0.0) {
			commands_printf("  Encoder direction is reversed: toggle Encoder Inverted");
			ok = false;
		} else if (ratio[d] > 1.05) {
			commands_printf("  Rotor faster than the field: Encoder Ratio (pole pairs) is too high");
			ok = false;
		} else if (ratio[d] < 0.8) {
			commands_printf("  Large slip or Encoder Ratio too low: check pole pairs, or reduce load");
			ok = false;
		}
	}

	commands_printf(ok ? "Encoder check passed\n" : "Encoder check FAILED\n");
}

// LispBM extensions

#ifdef USE_LISPBM
static lbm_value ext_acim_flux(lbm_value *args, lbm_uint argn) {
	(void)args; (void)argn;
	return lbm_enc_float(m_core.imr);
}

static lbm_value ext_acim_slip(lbm_value *args, lbm_uint argn) {
	(void)args; (void)argn;
	return lbm_enc_float(m_core.slip_hz);
}

static lbm_value ext_acim_state(lbm_value *args, lbm_uint argn) {
	(void)args; (void)argn;
	return lbm_enc_i((lbm_int)m_core.state);
}

static lbm_value ext_acim_wr(lbm_value *args, lbm_uint argn) {
	(void)args; (void)argn;
	return lbm_enc_float(m_core.wr / (2.0 * M_PI));
}

static void load_extensions(bool main_found) {
	if (!main_found) {
		lbm_add_extension("acim-flux", ext_acim_flux);
		lbm_add_extension("acim-slip", ext_acim_slip);
		lbm_add_extension("acim-state", ext_acim_state);
		lbm_add_extension("acim-wr", ext_acim_wr);
	}
}
#endif
