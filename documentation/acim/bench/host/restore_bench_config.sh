#!/bin/sh
# Restore the HI200 + Continental BSG bench configuration (2026-10-09) after a reflash, which
# resets the motor config and the ACIM page to firmware defaults. Needs VESC Tool connected to
# the board with its TCP server on (port 65102). Every value is written, read back and checked.
set -e
cd "$(dirname "$0")"
python3 vesc_mcconf.py --set \
  l_current_max=30 l_current_min=-5 l_in_current_max=20 l_in_current_min=-5 l_abs_current_max=60 \
  l_max_erpm=3000 l_min_erpm=-3000 l_max_vin=60 l_battery_cut_start=44.8 l_battery_cut_end=42 \
  foc_phase_filter_enable=0 foc_dt_us=0.53 foc_motor_r=0.0078 foc_motor_l=0.00001207 \
  foc_motor_ld_lq_diff=0.000000589 foc_current_kp=0.0121 foc_current_ki=7.8 \
  m_sensor_port_mode=1 m_encoder_counts=4000 foc_encoder_ratio=4 si_motor_poles=8
python3 vesc_acim_conf.py enable=1 speed_src=0 id_mag=20 tau_r=0.045 lm=0.0001 lr_lm=1.06 \
  current_max=40 sl_min_hz=50 sl_if_ramp=40
python3 vesc_term.py acim_status
