# ACIM support for VESC: design

Target: MakerX HI200-C V2.2b running the `100_250` hwconf (see `PHASE1_BUILD.md`),
driving a Continental 48 V eTorque belt starter-generator (3-phase squirrel cage,
pole pairs, tau_r, Lm and rated magnetizing current unknown).
Base: upstream `release_7_00` @ `20cbb36`. All line numbers below refer to that commit.

## 1. Why stock FOC fails on this motor

Two things in `motor/mcpwm_foc.c` make stock FOC unusable on an induction motor:

1. **Id is forced to zero.** Every branch of the sensor-mode switch in the ADC ISR ends
   with `id_set_tmp = 0.0` (lines 3471, 3509, 3517, 3534, 3553, 3581). A PM motor gets its
   flux from magnets; an induction motor only has rotor flux if the controller supplies a
   d-axis magnetizing current. With Id = 0 the rotor flux decays with tau_r.
2. **The angle comes from a PM flux observer.** `foc_observer_update()` (`foc_math.c:26`)
   estimates a constant-magnitude flux linkage `foc_motor_flux_linkage`. An induction
   motor's flux magnitude follows Id with a tau_r lag, and its angle slips ahead of the
   rotor. Any residual flux at start gives a brief torque, then the observer loses lock.

The fix is to own both the angle and Id while ACIM mode is active, and leave the PI current
loop, SVM, limits, protections and comms unchanged.

## 2. Control structure

Indirect field-oriented control (IFOC) with a rotor-flux "current model" in stator
coordinates, as in the brief:

    d(psi_a)/dt = (Lm*i_alpha - psi_a)/tau_r - w_r*psi_b
    d(psi_b)/dt = (Lm*i_beta  - psi_b)/tau_r + w_r*psi_a
    theta       = atan2(psi_b, psi_a)

`w_r` is rotor electrical speed (pole pairs x mechanical rad/s).

Implementation choices:

- **Flux in amps.** Integrate `i_mr = psi_r / Lm` instead of psi (divide the equations by
  Lm). The angle then depends only on tau_r. Lm is only needed for telemetry (flux in
  V*s), BEMF feed-forward and the sensorless voltage model. This is the same convention
  ODrive uses (`acim_estimator.cpp`). It matters because Lm is the parameter we know least.
- **Measured currents.** Feed the model with measured i_alpha/i_beta, not the setpoints.
  ODrive feeds setpoints, which goes wrong when the current loop saturates at high speed.
- **Discretization.** Exact rotation plus exponential decay per step, not forward Euler:
  rotate (i_mr_a, i_mr_b) by the rotor's electrical angle increment plus slip, then
  `i_mr += (1 - exp(-dt/tau_r)) * (i - i_mr)`. In encoder mode the increment is the raw
  encoder angle change since the last sample, not PLL speed x dt. The Phase 3 simulation
  showed that the PLL lags under hard acceleration and gave a 39 deg flux angle error, where
  the raw increment gives 0.3 deg.
  `exp(-dt/tau_r)` is precomputed whenever the config changes. It stays stable for any tau_r
  and speed and costs one sincos per sample, which the ISR already does.
- **Accumulated state, no increments.** The state is the flux vector itself. Unlike the
  2020 Split thesis, nothing is reconstructed from per-step deltas, and there is no
  hard-coded Lr/Lm.
- **No angle advance.** The PM path advances the observer angle by half a period
  (`mcpwm_foc.c:3451`). ACIM returns the flux angle at the sample instant and the PI
  integrators absorb the output delay. The advance biased id/iq and cost 3.5% flux in the
  simulation.
- **Torque/slip relations** (rotor-flux frame, used for limits and telemetry):
  `w_slip = iq / (tau_r * i_mr)`, `T = 1.5 * pp * (Lm^2/Lr) * i_mr * iq`.

### Speed sources

| Mode | w_r source | Use |
|------|-----------|-----|
| `ENCODER` | AS5047P on ABI (sensor port, `SENSOR_PORT_MODE_ABI`), electrical angle = mech * pole pairs. The current model uses raw angle increments; the speed estimate is `pll_w + kp*err` (PLL integrator plus proportional term, so it does not lag) | Primary. The current model is exact apart from parameter error. |
| `SENSORLESS` | Estimated: w_r = w_psi - w_slip, where w_psi comes from a voltage-model flux estimate | First tests, until the encoder arrives. |

Voltage-model observer (validated in the Phase 3 simulation, `SIMULATION.md`). It runs in
both modes: in `SENSORLESS` it provides the angle, in `ENCODER` it is only a plausibility
check for the faults in section 5. It is a closed-loop hybrid, Jansen-Lorenz style:

    psi_s  = integral( v - Rs*i + u_corr ) dt         (voltage model, stator flux)
    psi_r_vm = (Lr/Lm) * (psi_s - sigmaLs*i)           (rotor flux from voltage model)
    psi_r_cm = current model above, driven by w_r_est
    u_corr = Kp*(psi_r_cm - psi_r_vm) + Ki*integral(...)   (pulls VM toward CM at low speed,
                                                       removes integrator drift)
    w_psi  = d/dt angle(psi_r_vm)  (PLL)
    w_r_est = w_psi - iq/(tau_r*i_mr)

At low speed the corrector makes the current model dominate. At speed, the voltage model
dominates and tau_r errors matter less. The crossover is set by `acim_obs_bw`
(Kp = 2 wc, Ki = wc^2, wc = 2 pi obs_bw; default 3 Hz). The voltage reaching the motor is
taken from the duty cycles of the previous period, matching when it was actually applied.

Below a minimum stator frequency an induction motor is not observable sensorlessly, so
`SENSORLESS` starts with **I/f**:

- Build flux at standstill, then rotate a **single current vector** of magnitude
  sqrt(Id_mag^2 + Iq_req^2) open-loop, ramping its frequency at `acim_sl_if_ramp` in the
  direction of the request. A first version commanded Id/Iq against the I/f angle and rocked
  the rotor back and forth.
- With no torque request the I/f frequency ramps back to zero instead of dragging the rotor.
- Hand over to the observer above 1.2 x `acim_sl_min_hz` (12 Hz by default), and back to I/f
  below 0.8 x (8 Hz). At handover the current model is re-seeded from the voltage model, and
  the plausibility faults are blanked for 250 ms.
- In closed loop the sensorless current model is rotated by the voltage-model angle
  increment minus the commanded slip.
- Rs is the sensitive parameter: at low speed the IR drop on this motor exceeds the
  back-EMF. That is why `acim_sl_min_hz` defaults to 10 Hz and `foc_temp_comp` should be on.

`sigmaLs` is the inductance VESC already measures (`foc_motor_l`, the high-frequency
transient inductance). `Rs` is `foc_motor_r` (temperature-compensated `m_res_temp_comp`).

### Flux management (state machine)

    OFF --(run request)--> FLUXING --(i_mr >= 0.9*Id_mag && t >= flux_build_time)--> RUN
     ^                       |  Iq forced 0, Id = Id_mag                              |
     |                       v                                                         |
     +--- (PWM stops) <-- HOLD (Iq=0, Id=Id_mag for flux_hold_time after release) <--+
    any state --(implausible flux/slip, NaN, encoder loss)--> FAULT (mc_interface_fault_stop)

- The model keeps running while PWM is off, with i = 0. Rotor flux then decays at tau_r
  exactly as it does physically, so a quick re-start only waits for the remaining flux to
  build. Starting onto a spinning rotor (encoder mode) works the same way.
- `flux_build_time` defaults to 3 x tau_r.
- Open question for Phase 4: in sensorless mode the HOLD state has nothing to hold the angle
  against at standstill, so it may be better to stop PWM at once when the request drops to
  zero, and re-flux on the next start.
- In RUN, Iq is clamped to the slip limit: `|iq| <= slip_max * tau_r * i_mr`. This also stops
  Iq from running away while i_mr is still low.
- Id has priority in the current limit. Iq is limited to `sqrt(Imax^2 - Id^2)`, which is
  what the existing limit code at lines 3657-3659 already does.

## 3. Where ACIM hooks into the firmware

New files (all ACIM logic lives here):

- `motor/acim.c`, `motor/acim.h`: config, estimator, state machine, faults, terminal
  commands, telemetry getters. Added to `motor/motor.mk`.
- `motor/acim_conf.c`, `motor/acim_conf.h`: custom-config XML blob, serializer and EEPROM
  storage (section 4).

Changes to existing files. Each is a guarded branch: `if (acim_active(motor)) { ... }` or
a one-line call. Everything is a no-op when `acim_enable` is false.

| # | File:line | Existing code | ACIM change |
|---|-----------|---------------|-------------|
| 1 | `mcpwm_foc.c:356` `mcpwm_foc_init` | init | `acim_init()` registers the custom config and terminal commands |
| 2 | `mcpwm_foc.c:3259-3278` | encoder angle -> `m_phase_now_encoder` (applies `foc_encoder_offset`) | none. ACIM reads `encoder_read_deg()` itself, because the PM offset is meaningless for an induction rotor and only ratio/direction matter |
| 3 | `mcpwm_foc.c:3309-3310` | `id_set_tmp`, `iq_set_tmp` from setpoints | save the requested `iq_set_tmp` before the sensor switch (the sensorless branch adds `foc_sl_openloop_boost_q`) |
| 4 | `mcpwm_foc.c:3443-3607` | PM observer + sensor-mode switch picks `state_now->phase`, forces `id_set_tmp = 0` | after the switch: `acim_update(motor, i_alpha, i_beta, v_alpha, v_beta, dt, &phase, &id_set_tmp, &iq_set_tmp)` overrides phase, Id (Id_mag) and Iq (flux gating + slip clamp). Skipped in open-loop / phase-override / handbrake modes, so those stay usable for bench tests. The PM observer call at 3445 is skipped in ACIM mode to save ISR time |
| 5 | `mcpwm_foc.c:3611-3624` | MTPA (uses ld_lq_diff and flux linkage) | skip when ACIM is active |
| 6 | `mcpwm_foc.c:3630-3639` | PM field weakening (negative Id) | skip when ACIM is active. ACIM flux weakening (Id_mag ~ 1/w above base speed) is a later option |
| 7 | `mcpwm_foc.c:3669-3743` | undriven branch: PM observer tracks BEMF, picks phase | `acim_update_undriven()`: run the model with i = 0 so flux decays correctly, keep the phase coherent for restart |
| 8 | `mcpwm_foc.c:3783-3789` | preset `vq_int` with `w * flux_linkage` before re-drive | in ACIM mode preset with `w_s * Lm * i_mr` (rotor-flux BEMF) instead |
| 9 | `mcpwm_foc.c:3807-3818` | speed PLL runs on `state->phase` (= flux angle in ACIM, i.e. stator frequency) | in ACIM mode feed the **rotor** electrical angle, so ERPM, tachometer, speed PID and odometry report rotor speed rather than synchronous speed |
| 10 | `mcpwm_foc.c:4642-4662` `control_current` | decoupling uses `m_speed_est_fast` and `foc_motor_flux_linkage` | in ACIM mode use `w_s` from the estimator: cross terms with sigmaLs (`p_ld`/`p_lq` = `foc_motor_l`), BEMF term `w_s * Lm/Lr * psi_r` |
| 11 | `mcpwm_foc.c:3937-3961` `timer_update` | stops PWM when the Iq/Id setpoints drop below `cc_min_current` | while ACIM is in HOLD, keep `m_current_off_delay` above zero so the field is held for `flux_hold_time` |
| 12 | `mcpwm_foc.c:4026-4120` `timer_update` | sensorless open-loop start (`m_phase_observer_override`) | ignored in ACIM mode. ACIM has its own I/f start |
| 13 | `datatypes.h:178` `mc_fault_code` | last code `FAULT_CODE_ABS_OVERSPEED` | append `FAULT_CODE_ACIM_FLUX`, `FAULT_CODE_ACIM_SLIP`. This does not change the config signature. Stock VESC Tool shows them as unknown numbers, and the terminal prints the name |

Untouched, and reused as is: current sampling/Clarke, PI current controller, SVM,
dead-time compensation, all current, voltage and temperature limits and faults in
`mc_interface.c`, CAN and comms, and R/L measurement (`mcpwm_foc_measure_res_ind`, which
gives Rs and sigmaLs on an induction motor too).

Not applicable in ACIM mode: VESC Tool's flux-linkage detection, encoder offset detection
(`mcpwm_foc_encoder_detect`, which aligns to magnets), hall detection and HFI. The
recommended mcconf for ACIM is documented below so nobody has to rediscover it.

ISR budget: one atan2, one sincos, and about 30 multiply-adds per sample. That is well
under 2 us at 168 MHz against a 66 us period (30 kHz switching, low-side shunts sample on V0
only, so `p_fs` = 15 kHz). The PM observer that ACIM skips costs more than that.

Flash budget: about 24 KB free on `100_250`. Target under 10 KB for the code plus the
compressed config XML.

## 4. Configuration parameters

### Why not add them to `mc_configuration`

Adding fields to `mc_configuration` changes `MCCONF_SIGNATURE` (`confgenerator.h`).
Stock VESC Tool then refuses to read or write the motor config, so it would need a
custom VESC Tool build. That is a large, conflict-prone change for every rebase.

Instead, ACIM uses the firmware's existing **custom config** channel (`conf_custom.c`,
`COMM_GET/SET_CUSTOM_CONFIG[_XML]`). Stock VESC Tool fetches the XML from the firmware and
shows it as an extra settings page with read/write/default buttons. `mc_configuration`, its
signature and `confgenerator.c` stay untouched. Values are stored in the 32 hardware EEPROM
variables (`conf_general_store_eeprom_var_hw`, `EEPROM_VARS_HW = 32`), which the `100_250`
hwconf does not use, under a signature word. This avoids colliding with LispBM, which uses
the custom EEPROM area.

Caveat: there is a single custom-config slot. A LispBM package that registers its own
config (for example Refloat) would replace the ACIM page, and stopping that package clears
it. ACIM therefore re-registers its page when LispBM stops, and its settings keep working
from EEPROM even when the page is hidden.

### Parameters (page "ACIM")

| Name | Unit | Default | Meaning |
|------|------|---------|---------|
| `acim_enable` | bool | **false** | Master switch. Read only while the motor is stopped. When false, the firmware is bit-for-bit stock in behavior |
| `acim_speed_src` | enum | Encoder | `Encoder` / `Sensorless` |
| `acim_id_mag` | A | 10 | Magnetizing (d-axis) current. Deliberately low until measured |
| `acim_tau_r` | s | 0.10 | Rotor time constant Lr/Rr |
| `acim_lm` | uH | 100 | Magnetizing inductance. Used only for flux telemetry, BEMF feed-forward and the sensorless voltage model |
| `acim_lr_lm` | ratio | 1.03 | Lr/Lm (1 + rotor leakage ratio). Sensorless and torque estimate only |
| `acim_slip_max` | Hz (elec) | 8 | Slip clamp. Iq is limited so the commanded slip never exceeds this |
| `acim_flux_build_time` | s | 0 (= 3 x tau_r) | Minimum time holding Id before Iq is allowed |
| `acim_flux_hold_time` | s | 0.5 | Keep magnetized after torque goes to zero, to avoid re-flux delays |
| `acim_current_max` | A | 40 | ACIM-mode cap on \|I\| (applied on top of mcconf limits), so first runs are gentle |
| `acim_sl_min_hz` | Hz (elec) | 10 | Sensorless: hand over to the observer above 1.2 x this, back to I/f below 0.8 x |
| `acim_sl_if_ramp` | Hz/s | 20 | Sensorless I/f frequency ramp |
| `acim_obs_bw` | Hz | 3 | Sensorless: VM/CM crossover bandwidth (sets Kp, Ki) |
| `acim_fault_flux_err` | % | 50 | Fault when voltage-model and current-model flux disagree by more than this for 100 ms |
| `acim_fault_slip_fac` | x | 3 | Fault when the unclamped slip demand exceeds this x slip_max for 200 ms (flux collapse) |

Pole pairs come from the existing `foc_encoder_ratio` (VESC already treats it as electrical
per mechanical revolution), and `si_motor_poles` is used for display. Encoder direction uses
`foc_encoder_inverted`. `foc_encoder_offset` is ignored in ACIM mode.

### Recommended mcconf (stock fields) for ACIM

- Motor type FOC. Run "Measure R and L" only, not the full FOC detection.
- `foc_sensor_mode` = Sensorless (ACIM picks its own speed source).
- `m_sensor_port_mode` = ABI, `m_encoder_counts` = 4000 (AS5047P default ABI resolution),
  when the encoder is fitted.
- `foc_cc_decoupling` = Cross (ACIM supplies its own BEMF term when Cross+BEMF is selected).
- `foc_fw_current_max` = 0, `foc_mtpa_mode` = Off. Both are skipped anyway.
- `foc_motor_flux_linkage`: leave as measured. It is unused in ACIM mode.
- Conservative `l_current_max` and `l_in_current_max` for first spins.

## 5. Faults

All faults go through `mc_interface_fault_stop(code, is_second_motor, true)` from the ISR,
so the existing fault log, LED and VESC Tool fault display apply.

| Fault | Condition |
|-------|-----------|
| `FAULT_CODE_ACIM_FLUX` | NaN or Inf in the estimator state. i_mr > 1.5 x `acim_current_max` (the current model low-passes the measured current, so only a numerical fault gets there). Any mode, once the stator frequency is above `acim_sl_min_hz` and outside the 250 ms handover blanking: \|psi_vm - psi_cm\| / \|psi_cm\| > `acim_fault_flux_err` for 100 ms. In simulation this catches pole pairs set wrong within 150-165 ms of torque |
| `FAULT_CODE_ACIM_SLIP` | Unclamped slip demand > `acim_fault_slip_fac` x slip_max for 200 ms, meaning flux collapsed while torque was requested. Encoder mode, same enable conditions as above: the slip seen by the voltage model differs from the commanded slip beyond tolerance for 50 ms. In simulation this catches a lost encoder signal in 51 ms |
| `FAULT_CODE_ENCODER_FAULT` (existing) | Encoder mode with no encoder configured, or the existing `encoder_check_faults()` |

A backwards encoder is **not** caught at runtime: the motor stalls near standstill, where the
voltage model has no information, and draws current without faulting. The simulation
confirmed this. It is caught at commissioning instead, by `acim_enc_check` (section 6).

## 6. Telemetry

Stock VESC Tool's realtime page has a fixed packet layout (`COMM_GET_VALUES`), so ACIM
telemetry uses channels that need no VESC Tool changes:

- **Terminal `acim_status`**: state, i_mr (A), psi_r (mWb = Lm x i_mr), slip (Hz), rotor and
  stator electrical frequency, Id/Iq target vs measured, and fault counters.
- **Terminal `acim_enc_check`**: commissioning check. Rotates the field open-loop at low
  current through a few electrical turns in each direction and reports encoder direction and
  counts per electrical turn against `foc_encoder_inverted` and `foc_encoder_ratio`. Run it
  once after fitting the encoder and after any wiring change.
- **Terminal `acim_plot [on|off]`**: streams to VESC Tool's Experiment plot (the same mechanism
  as `terminal_plot_hfi`, `mcpwm_foc.c:5384`). It streams i_mr, slip Hz, w_r, w_s, Id and Iq,
  plus psi_vm vs psi_cm in sensorless mode.
- **LispBM extensions** `(acim-flux)`, `(acim-slip)`, `(acim-state)` and `(acim-wr)`. These
  allow logging and scripting from VESC Tool's LispBM REPL and `plot-send-points`, for
  example during the Phase 5 tau_r sweep.
- `mc_interface_get_rpm()` reports rotor ERPM (hook 9). Iq and Id appear on the normal
  realtime page as usual.

## 7. What ODrive does, and what is taken from it

ODrive fw-v0.5.6 (MIT, Copyright (c) 2016-2018 ODrive Robotics), `acim_estimator.cpp`:

- Rotor-flux magnitude in amps: `di_mr/dt = (Id - i_mr)/tau_r`, slip = `Iq/(tau_r*i_mr)`,
  with slip integrated into a phase offset added to the encoder angle. **Adopted:** the
  flux-in-amps convention and the slip relation, which are textbook IFOC.
- Feeds Id/Iq setpoints, has no pre-magnetization, can't set a fixed Id (Id only comes from
  "autoflux"), uses forward Euler, and silently zeroes slip above 0.1/dt. **Not adopted.**
- Its velocity-loop gains scale by 1/max(flux, gain_min_flux). **Consider for Phase 4**, where
  VESC's speed PID is used with ACIM.

No ODrive code is copied verbatim. If any routine ends up closely following theirs, the
file will carry the MIT notice and attribution above.

## 8. Rebase strategy

- All logic is in the four new files. Existing files get about 15 short guarded hunks in
  `mcpwm_foc.c`, one enum addition in `datatypes.h` and one line in `motor.mk`.
- Every hook is wrapped as `if (acim_active(motor))`, which reads one bool that is false
  unless enabled, so upstream diffs around the hooks stay readable.
- No changes to `mc_configuration`, `confgenerator.*`, VESC Tool XML or the hwconf.

## 9. Open questions (answered by Phase 3 simulation or Phase 5 bench tests)

1. Pole pairs: open-loop rotating field at low current, counting shaft turns per electrical
   revolution.
2. tau_r and Lm: no-load test (Lm from V/(w*I) at synchronous speed), then a tau_r sweep for
   peak torque per amp. Until then, the defaults are guesses.
3. Sensorless observer gains and the lowest usable speed. Answered by Phase 3: obs_bw 3 Hz,
   handover at 12 Hz electrical, usable but Rs-sensitive. See `SIMULATION.md`.
5. tau_r sweep direction. Phase 3 showed that over-estimating tau_r over-fluxes the motor
   and runs out of voltage at high speed, so the Phase 5 sweep starts from a low guess and
   works upward at moderate speed.
4. Whether sampling on V0 only (low-side shunts) is adequate at the BSG's top electrical
   frequency. This board cannot sample in V0+V7 (no phase shunts).
