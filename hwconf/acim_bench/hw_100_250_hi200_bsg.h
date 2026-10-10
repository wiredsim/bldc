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

#ifndef HW_100_250_HI200_BSG_H_
#define HW_100_250_HI200_BSG_H_

/*
 * MakerX HI200-C V2.2b running the Trampa 100_250 hardware config (as shipped by the vendor),
 * driving the Continental 48 V eTorque belt starter-generator on the ACIM bench.
 *
 * Identical to 100_250 except for the motor config defaults below. The VESC bootloader erases
 * flash sectors 0-7 on every update, including the emulated EEPROM in sectors 1-2, so every
 * reflash resets the configuration to these defaults. Stock 100_250 defaults are not safe on
 * this bench (400 A abs current, -200 A regen, no ERPM limit, phase filters on), so they are
 * the measured bench values here. See documentation/acim/BENCH_TEST_PLAN.md "Bench status".
 * The ACIM page resets to its own defaults (ACIM disabled); restore it with
 * documentation/acim/bench/host/restore_bench_config.sh.
 */

// Limits: 14S Li-ion pack, bench values
#define MCCONF_L_CURRENT_MAX			160.0
#define MCCONF_L_CURRENT_MIN			-20.0
#define MCCONF_L_IN_CURRENT_MAX			60.0
#define MCCONF_L_IN_CURRENT_MIN			-20.0
#define MCCONF_L_MAX_ABS_CURRENT		240.0
#define MCCONF_L_RPM_MAX				6000.0
#define MCCONF_L_RPM_MIN				-6000.0
#define MCCONF_L_MAX_VOLTAGE			60.0
#define MCCONF_L_BATTERY_CUT_START		44.8
#define MCCONF_L_BATTERY_CUT_END		42.0

// The HI200 has no switchable phase filters: with them enabled the phase voltage is sampled
// at the PWM midpoint and reads about 0 V while driving
#define MCCONF_FOC_PHASE_FILTER_ENABLE	false
// Fitted with measure_res at 5 to 30 A (hardware dead time 660 ns in hw_100_250.h)
#define MCCONF_FOC_DT_US				0.53

// Motor: Measure R and L after the two fixes above; gains are L and R x 1000 rad/s
#define MCCONF_FOC_MOTOR_R				0.0078
#define MCCONF_FOC_MOTOR_L				12.07e-6
#define MCCONF_FOC_MOTOR_LD_LQ_DIFF		0.589e-6
#define MCCONF_FOC_CURRENT_KP			0.0121
#define MCCONF_FOC_CURRENT_KI			7.8
#define MCCONF_SI_MOTOR_POLES			8

// AS5047P on the shaft, ABI, 4000 counts per turn; ratio = pole pairs
#define MCCONF_M_SENSOR_PORT_MODE		SENSOR_PORT_MODE_ABI
#define MCCONF_M_ENCODER_COUNTS			4000
#define MCCONF_FOC_ENCODER_RATIO		4.0

#include "../trampa/100_250/hw_100_250.h"

#endif /* HW_100_250_HI200_BSG_H_ */
