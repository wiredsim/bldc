# ACIM Phase 5: bench test plan

Target: MakerX HI200-C V2.2b (`100_250` hwconf), Continental 48 V eTorque belt
starter-generator, bench supply at about 20.5 V. The pole pairs, tau_r, Lm and Id_mag are
unknown at the start; this plan measures them in that order. The AS5047P encoder comes
later: steps 1 to 7 need no encoder.

Every step says what to expect and what to do if something else happens. **If anything
looks wrong, stop and send the output before going on.**

Scripts are in `documentation/acim/bench/`. Paste one into VESC Tool > LispBM Scripting
and press Run. The stop button in that tab stops it, and the motor stops with it, because
each script refreshes the command timeout itself. All four were checked on the host LispBM
REPL (same LispBM version and build flags as the firmware) against a fake motor. They have
not run on real hardware.

| Script | Step | ACIM mode |
|---|---|---|
| `pole_count.lisp` | 4: pole pairs | off |
| `no_load.lisp` | 5: magnetizing curve (Lm, Id_mag) and tau_r | off |
| `first_spin.lisp` | 6, 7, 9: short ACIM runs with a printout | on |
| `accel_test.lisp` | 10: rotor time constant sweep | on, encoder |

## Bench status, 2026-10-09

**ACIM runs in encoder mode** (steps 1 to 11 done, see each section): from standstill to about 680 ERPM in 3 s at 5 A torque, both
directions, flux held at 20 A, clean FLUXING > RUN > HOLD > OFF, no faults. Sensorless mode
starts (I/f) but cannot hand over to the observer at bench speeds; see below.

Board settings that differ from stock 7.00 and why:

| Setting | Value | Why |
|---|---|---|
| FOC phase filters | **off** | The HI200 runs the 100_250 hwconf but has no switchable phase filters. With them on, VESC reads the phase voltage at the PWM midpoint, about 0 V, so vd/vq, Measure R and L and the ACIM voltage model all saw a fraction of the real voltage |
| Dead-time compensation | **0.53 us** (stock 0.12) | Fitted from `measure_res` at 5 to 30 A: V = R*I + V0, with V0 driven to about 0 at working currents |
| Motor R / L / Ld-Lq | 7.8 mOhm / 12.1 uH / 0.59 uH | Re-measured after the two fixes above (earlier 36 mOhm / 18.1 uH were wrong) |
| Current Kp / Ki | 0.0121 / 7.8 | L and R times 1000 rad/s |
| Motor Poles | 8 | 4 pole pairs (step 4 note) |
| Encoder Ratio | 4 | Pole pairs |
| ACIM page | Id_mag 20 A, tau_r 30 ms (step 10), Lm 100 uH, Lr/Lm 1.06, Speed Source Encoder, sl_min_hz 50, sl_if_ramp 40 | Step 5 and 10 results; sl_min_hz 50 since df41aff2 |

Motor: Rs 7.8 mOhm, sigma*L 12.1 uH, Lm about 100 uH, Lr/Lm 1.06, tau_r about 45 ms cold,
knee between 30 and 40 A, 4 pole pairs. The encoder hand check read 10.07 turns for 10, and
`acim_enc_check` passed (rotor/field 0.88 forward and reverse at 120 ERPM, friction slip).

Known issues, not yet fixed in firmware:

1. **The voltage model is too weak at low speed on this motor.** At 20 A the stator voltage is
   under 1 V below about 60 Hz electrical, and the dead-time error left after compensation is
   0.05 to 0.3 V. The voltage-model flux read 1.6 mWb at 20 Hz falling to 0.4 mWb at 50 Hz
   (true value about 2 mWb), so the sensorless handover fails and falls back to I/f.
2. **The voltage-model checks are gated by the voltage model's own speed** (`vm_valid` uses
   `vm_w`). When that estimate spikes, the ACIM_FLUX and encoder slip-plausibility checks run
   on garbage and trip falsely; in encoder mode this gave an ACIM_SLIP at 554 ERPM. The gate
   should use the trusted stator frequency (`w_s`, encoder speed plus commanded slip). Until
   then, **sl_min_hz = 100** keeps the checks off below 6000 ERPM, which also means the step 9
   encoder fault checks will not trip at bench speeds.
3. **With ACIM enabled but not driving (open loop, measurements), VESC's speed estimate reads
   0**, because `phase_for_speed_est` is the ACIM rotor phase, which only moves in RUN. The
   speed-dependent voltage filter then sits at its minimum (filter_const 0.01), so vd/vq lag
   and shrink with frequency. Fall back to the normal phase when ACIM is not in RUN.

## 0. Safety and the bench

- **Clamp the motor**, take the belt off and guard the pulley. Everything here runs at no
  load. Keep a hand on the supply switch.
- **Power: the 14S Li-ion pack (58.8 V full, 42 V empty) is the bench baseline.** A bench
  supply cannot absorb regen; the battery can, within its BMS charge limit. An induction
  motor does not generate once it is unpowered (no magnets, and the flux dies within a few
  tau_r), so cutting the power is always safe. Regen only happens while the controller is
  braking. Baseline settings (Motor > General):
  - Current: **Battery Current Max = 20 A, Battery Current Min (regen) = -5 A**.
  - Voltage: **Maximum Input Voltage = 60 V** (just above full charge), **Battery Voltage
    Cutoff Start / End = 44.8 / 42.0 V** (3.2 / 3.0 V per cell).
  - If you ever go back to a bench supply: regen -1 A and Maximum Input Voltage a few volts
    above the supply, since an overvoltage fault is the only thing protecting it.
- **Speed limit**: Motor > General > RPM: **Max ERPM = 3000, Min ERPM = -3000** for steps 6
  to 9. ERPM is electrical: 3000 ERPM is 750 rpm at the motor's 4 pole pairs. VESC tapers the
  current from 80% of Max ERPM to zero at 100%, so a test that runs at a fixed ERPM needs
  the cap at least 25% above it.
- **Current**: Motor Current Max 60 A and Motor Current Max Brake -20 A for steps 4 and 5.
  Leave Absolute Maximum Current at the default. The ACIM page has its own current limit
  on top of these (ACIM Current Limit, default 40 A).

## 1. Back up, then flash

1. In the VESC Tool you use now (5.03), save the motor config and the app config to XML
   files (File > Save Motor Configuration XML / Save App Configuration XML). Write down
   the hwconf name and the FW version from the Welcome page.
2. Flashing is done by you, in **VESC Tool 7.x** (the firmware is based on release 7.00).
   Connect, open Firmware > Custom File, and choose
   `acim/phase4/acim_100_250_0195f25.bin`
   (sha256 `56d42153bae1ff2c0d06c19a3a0491538b8e7dc0612aed65800a40ecd68a299e`).
   If VESC Tool asks to update the bootloader, do that first. Wait for the board to reboot.
3. The 7.x firmware starts with default configs. Do not load the 5.03 XML wholesale;
   copy the values over by hand where needed.

**Rollback**, if anything is off:
- Stock 7.00: Firmware > Included Files in VESC Tool 7.x, pick 100_250.
- Back to 5.03: Firmware > Custom File with `acim/phase5/stock_5.03_100_250.bin`
  (the official 5.03 release build, sha256
  `da1bd61a991fc6b6316ebb80394c2691bf73520705084319d5b9d28f214cc384`). Use VESC Tool 5.03 to
  talk to it afterwards.
- If the board will not boot at all: ST-Link on the SWD header.

## 2. Stock checks with ACIM off

1. Welcome page shows FW 7.00, hardware 100_250. Realtime Data shows about 20.5 V input.
2. Motor Settings > General: Motor Type **FOC**. FOC > General: Sensor Mode
   **Sensorless**, Current Controller Decoupling **Cross**.
3. Run **Measure R and L** only. This gives Rs and the leakage inductance (sigma*Ls), which
   is exactly what ACIM needs for foc_motor_l. **Do not run Measure Flux Linkage or the
   setup wizard**: both assume a permanent-magnet motor.
4. Terminal: `last_adc_duration` and note the "Latest sample ADC duration" (ISR time,
   ACIM off). Budget at 15 kHz is 66 us.

Record:

| Item | Value |
|---|---|
| Rs (mOhm) | |
| L (uH), Ld-Lq difference | |
| ISR time, ACIM off (us) | |

## 3. ACIM page

1. Click the ACIM page (it appears in the left column next to the App pages). Expect the
   General, Motor, Limits, Sensorless and Faults groups with the defaults
   (Magnetizing Current 10 A, Rotor Time Constant 100 ms, ACIM Current Limit 40 A).
   If there is no page: Terminal `acim_status`; if that command exists, reconnect VESC
   Tool once (the page is fetched on connect).
2. Change Magnetizing Current to 11 A, write, read back, power-cycle the board, read
   again: it must stay 11 A. Set it back to 10 A.
3. Terminal `acim_status` with Enable off: "ACIM mode: disabled, not active".

## 4. Pole pairs (no encoder)

Put a tape flag on the shaft or pulley. Run `pole_count.lisp`. It ramps a 20 A rotating
current up to 300 ERPM (5 Hz electrical), prints COUNT START, holds for 60 s, prints
COUNT STOP. Count the shaft turns between the two (film it if that is easier).

pole pairs = 300 / shaft turns, **rounded down** (slip makes the rotor slightly slower than
the field, so the raw ratio is a little high). 4 pole pairs gives 75 turns, 6 gives 50,
8 gives 37.5.

If the shaft stalls or jerks, raise `cur` by 10 A at a time. At low current this motor
makes little torque until it is magnetized.

Rounding down only works when slip is small, so the rotor must be properly magnetized. On
the bench, 10 A gave 24 turns in 30 s (150 field turns), which rounds down to 6 pole pairs.
The real answer was 4: the rotor was slipping 36%. The no-load test settled it, with the rotor
running at 747 to 752 rpm against a 3000 ERPM field (750 rpm at 4 pole pairs) from 20 A up.
Run this at the script's default 20 A or more, and if two candidate pole counts are both
plausible, check with the no-load rotor speed.

| Run | Current (A) | Shaft turns | 300 / turns | Pole pairs |
|---|---|---|---|---|
| 1 | | | | |

Enter the result as Motor Poles (= 2 x pole pairs) under Motor > Additional Info.

## 5. Magnetizing curve and tau_r (no encoder)

**Use `release_curve.lisp` on the 100_250 hardware, not part 1 of `no_load.lisp`.** On the
bench, part 1 read about 0.1 V where the real stator voltage was 2 to 3 V: while driving,
`get-vd`/`get-vq` are reported after dead-time compensation, which at 5% duty is as large as
the signal. So every L came out 0. The release voltage is measured directly and works.

`release_curve.lisp` ramps to 6000 ERPM at 30 A, then at each current holds the field,
releases, and dumps 400 ms of |V|^2 samples. `release_fit.py <log>` averages |V|^2 over each
electrical cycle and subtracts the at-rest value. That removes the 0.33 V measurement offset
exactly, because the cross term between a rotating vector and a fixed one averages to zero
over a cycle. It then fits the decay for tau_r and gets Lm^2/Lr from the voltage at release
(V0 = w_rotor * Lm^2/Lr * I). Max ERPM must be 8000 for this run. Only points where the rotor
holds near synchronous speed are valid; below about 20 A at 6000 ERPM it falls behind.

Bench result, 2026-10-09, battery at 52.6 V:

| I (A) | rotor (rpm) | slip | V0 (V) | Lm^2/Lr (uH) | psi_r (mWb) | tau_r (ms) |
|---|---|---|---|---|---|---|
| 40 | 1496 | 0.3% | 2.198 | 87.7 | 3.51 | 51.1 |
| 30 | 1496 | 0.2% | 1.852 | 98.5 | 2.95 | 44.2 |
| 20 | 1478 | 1.4% | 1.172 | 94.6 | 1.89 | 44.3 |
| 15 | 1010 | 33% | not valid, rotor fell behind | | | |
| 10 | 371 | 75% | not valid | | | |

Correction, 2026-10-09 (later): the 11% drop at 40 A was a fitting artefact, not a knee.
`release_fit.py` used the whole decay, and the tail sits on what is left of the offset,
which bends the fit and pulls V0 down. It now fits only down to 25% of the starting
voltage. Refitted, and extended to 100 A with ACIM **disabled** (with ACIM enabled the
release is not a release: ACIM holds the flux at its own Id_mag for flux_hold_time and
trips ACIM_FLUX at speed):

| I (A) | 20 | 30 | 40 | 60 | 80 | 100 |
|---|---|---|---|---|---|---|
| Lm^2/Lr (uH) | 96 to 106 | 104 | 106 to 112 | 106 | 96 | 101 |
| psi_r (mWb) | 1.9 to 2.1 | 3.1 | 4.3 to 4.5 | 6.3 | 7.7 | 10.1 |
| tau_r from the decay (ms) | 38 to 41 | 41 | 40 to 42 | 40 | 37 | 39 |

Extended to 120 A: Lm^2/Lr 94 uH, psi_r 11.3 mWb (-6%); 150 A: 87 uH, 13.0 mWb (-13%).
Battery current peaked at 8.4 A and the FETs reached 37 C.

**Saturation starts between 120 and 150 A**: below 100 A Lm^2/Lr is flat at about 100 uH. By
the rule above (L down 10 to 15%), **Id_mag is about 150 A**, and ACIM runs should start at
about half, 75 A. Lm is about 106 uH unsaturated, Lr/Lm about 1.06. The decay tau_r is about 40 ms at every current; the step 10
spin-up sweep gives 30 ms, which is the value to run with.

The original `no_load.lisp` procedure follows for reference.

Run `no_load.lisp`. It spins the motor open-loop at 6000 ERPM (100 Hz electrical, about
1000 rpm at 6 pole pairs), steps the current from 60 A down to 10 A, and then lets go and
times the voltage decay. Expected output, one line per current, then tau_r:

Max ERPM must be at least `erpm` / 0.8 for this step (7500 for the default 6000 ERPM).
VESC starts cutting current at 80% of Max ERPM (`l_erpm_start`) and reaches zero at 100%,
so with the 3000 ERPM cap from the safety settings the field gets no current and every
point reads 0. Put the cap back to 3000 before step 6.

```
I  60.0 A   V  3.344 V   L    88.3 uH   psi  5.300 mWb
...
tau_r 119.0 ms   (60% at 67.6 ms, 20% at 198.3 ms)
V at release 2.223 V -> Lm*Lm/Lr 117.9 uH (multiply by Lr/Lm for Lm)
```

(These numbers are from the fake motor, not a prediction.)

- **Id_mag**: L stays flat at low current and falls as the iron saturates. Pick Id_mag
  where L has dropped 10 to 15% from its low-current value. Start ACIM runs at about
  half of that, then raise it.
- **Lm**: L at the chosen Id_mag, minus half the Measure R and L inductance (the stator
  leakage). The "V at release" line is a second estimate that avoids the 660 ns dead-time
  error of part 1. Use it if the two disagree by more than 10%.
- **Lr/Lm**: 1 + (Measure R and L inductance / 2) / Lm.
- **tau_r**: re-run with `decay-cur` set to the chosen Id_mag, because tau_r changes with
  saturation. Run it again at `erpm` 3000; if the two agree within about 5%, the rotor
  slowing down during the decay did not bias it. Rr rises as the rotor warms, so tau_r
  measured cold is the high end.

If a line shows an L that jumps around at the lowest currents, the rotor was slipping
there: ignore those points. If the decay is "not captured", raise `decay-cur` or `erpm`.

| Current (A) | V (V) | L (uH) | psi (mWb) |
|---|---|---|---|
| 60 | | | |
| 50 | | | |
| 40 | | | |
| 30 | | | |
| 20 | | | |
| 10 | | | |

| Result | Value |
|---|---|
| Knee / Id_mag chosen (A) | |
| Lm (uH), from L / from V at release | |
| Lr/Lm | |
| tau_r at Id_mag, 6000 / 3000 ERPM (ms) | |

## 6. First ACIM spin, sensorless, low current

Settings on the ACIM page: Speed Source **Sensorless**, Magnetizing Current = half the
chosen Id_mag, Rotor Time Constant and Magnetizing Inductance and Lr/Lm from step 5, ACIM
Current Limit 40 A, others default. Then **Enable ACIM Mode**, write. Terminal
`acim_status` must say "enabled, active". If it says "not active", it prints why.

Terminal `acim_plot 1` and open Realtime Data > Experiment to watch i_mr, slip, speeds,
Id, Iq and state.

Run `first_spin.lisp` (5 A for 3 s). Expected, in order:
1. FLUXING for about 3 x tau_r, i_mr rising to the target, no rotation.
2. RUN with the I/f field ramping at 20 Hz/s; the rotor follows it.
3. Around 12 Hz electrical (720 ERPM) the I/f hands over to the observer; the speed
   then keeps rising on torque alone up to the Max ERPM rollback.
4. After 3 s, HOLD (magnetized, no torque) for 0.5 s, then OFF and coasting.

Repeat with `iq` = -5 to check reverse. Then `last_adc_duration` while it runs to get the
ISR time with ACIM on.

If it faults, Terminal `faults` shows the code: ACIM_FLUX means the voltage and current
models disagree (check Lm, tau_r and the pole pairs). ACIM_SLIP means the flux collapsed
while torque was requested (raise Magnetizing Current or lower iq).

| Check | Result |
|---|---|
| FLUXING to RUN time (ms) | |
| Handover seen, at what ERPM | |
| Faults | |
| ISR time, ACIM on (us) | |

## 7. Sensorless, more current

Raise `iq` in steps (10, 20, 30 A) and Magnetizing Current up to the chosen Id_mag. In
`acim_status` at steady speed, the "Voltage model flux" should match psi_r within about
10%. If the gap grows with speed, the voltage timing assumption in DESIGN.md section 3 is
off: report it with the numbers.

## 8. Encoder fitted

1. Motor > General: Sensor Port Mode **ABI Encoder**, ABI Encoder Counts **4000** (the
   AS5047P default is 1000 pulses, 4000 counts). FOC Sensor Mode stays **Sensorless**:
   the ACIM page picks the encoder, not the FOC sensor mode.
2. FOC > Encoder: Encoder Ratio = pole pairs from step 4.
3. **Check the counts**: mark the shaft, turn it 10 turns by hand, and watch the rotor position
   display in Realtime Data (it shows the encoder angle once one is configured). It must come back to the same angle. A 4096-count encoder set as 4000
   drifts 86 degrees in 10 turns, and a 2% speed error is more than the whole slip at speed.
4. ACIM page: Speed Source **Encoder**, write. Terminal `acim_enc_check` (the motor turns
   one way, then the other). Expect "rotor/field 0.9 to 1.0", "implied pole pairs" near
   the configured ratio, and "Encoder check passed". It tells you if the direction is
   reversed (toggle Encoder Inverted) or the ratio is wrong.
5. `first_spin.lisp` again at 5 A, then 20 A. There is no I/f in encoder mode, so the
   rotor accelerates straight out of FLUXING.

## 9. Fault checks at low current (encoder mode)

| Test | Expected |
|---|---|
| Encoder Ratio set 2 too high, 5 A spin | ACIM_FLUX within about 0.5 s of passing 10 Hz electrical |
| Encoder unplugged (with the power off), 5 A spin | ACIM_SLIP or ACIM_FLUX, no runaway |
| Encoder Inverted toggled, 5 A spin | stalls or crawls, no fault (known gap; `acim_enc_check` catches it) |

Restore the settings after each test.

The checks only run once the voltage-model speed (filtered) passes `sl_min_hz`, 50 Hz
electrical (3000 ERPM) on this motor, so a 3 s spin at 5 A (about 700 ERPM) never reaches
them. Run each fault test a second time at 10 A up to a speed past the threshold, with Max
ERPM raised for that run, and run the same high-speed spin with the correct settings first.

Bench result, 2026-10-09, firmware df41aff2, battery 52.6 V:

| Test | Result |
|---|---|
| Baseline, correct settings, 10 A to 3510 ERPM | No fault. Checks active above 3000 ERPM for about 1.5 s |
| Encoder Ratio 6 (true 4), 5 A, 3 s | No fault, no runaway; 2313 ERPM reported (about 1540 real). Below the check threshold |
| Encoder Ratio 6, 10 A, up to reported 6000 ERPM | **ACIM_FLUX at 2905 ERPM reported**, about 0.7 s after the driven stator frequency reached 50 Hz. The rotor could not get past about 1930 real ERPM |
| Encoder Inverted, 5 A, 3 s | Crawled backwards at about 20 ERPM, no fault (the known gap) |
| Encoder unplugged (power off, unplug, power on), 5 A, 3 s | No fault, no runaway. Speed reads 0, so the field only turns at the commanded slip, about 1 Hz electrical; the shaft crept about 120 degrees in 3 s. **Not detected**: below `sl_min_hz` a missing encoder looks like a stalled rotor, and ABI gives no signal-present indication. Run `acim_enc_check` after any work on the encoder wiring |

## 10. Rotor time constant sweep (encoder mode)

`accel_test.lisp` times the spin-up from 600 to 2400 ERPM at 15 A, three runs per setting.
With only the rotor's own inertia, the shortest time means the most torque per amp.

1. Start at half the tau_r from step 5. Run the script, note the three times.
2. Raise Rotor Time Constant in steps of about 1.4x (for example 60, 85, 120, 170 ms)
   and repeat. Write each change on the ACIM page; it applies at the next start.
3. The fastest setting is tau_r. If the times are flat on the high side, pick the lower
   value: overestimating tau_r over-fluxes the motor at speed.

If "max input" gets close to Battery Current Max, the supply is limiting the torque and the
times mean nothing: lower `erpm-b` or `iq`.

| tau_r (ms) | Run 1 (s) | Run 2 (s) | Run 3 (s) | i_mr (A) | Slip (Hz) |
|---|---|---|---|---|---|
| 22 | 1.296 | 1.318 | 1.306 | 20.0 | 3.8 to 4.2 |
| 26 | 1.229 | 1.291 | 1.266 | 20.0 | 3.8 to 5.4 |
| **30** | **1.212** | **1.227** | **1.214** | 20.0 | 3.1 to 4.0 |
| 32 | 1.228 | 1.249 | 1.215 | 20.0 | 3.3 to 4.6 |
| 35 | 1.247 | 1.257 | 1.232 | 20.0 | 2.8 to 4.2 |
| 45 | 1.367 | 1.385 | 1.374 | 20.0 | 2.0 to 3.4 |
| 63 | 1.760 | 1.788 | 1.737 | 20.0 | 1.7 to 2.4 |
| 88 | 2.617 | 2.657 | 2.585 | 20.0 | 1.3 to 1.7 |

Bench result, 2026-10-09, firmware df41aff2, Id_mag 20 A, Iq 15 A, Max ERPM 3500 for the
sweep: **tau_r = 30 ms**, two thirds of the 45 ms from the step 5 decay. The optimum is
shallow within about 15% and steep above it (88 ms takes twice as long). Input current
stayed at 0.5 A, so the battery was not limiting.

**Correction (later the same day): sweep with Iq = Id.** Fastest spin-up at a fixed Iq finds
the right tau_r only when Iq is about equal to Id. With Iq < Id, a tau_r error turns the
controller's frame and part of the large Id becomes torque current, so the misaligned
settings win until the current angle reaches 45 degrees (max torque per amp). At Id 75 A,
Iq 5 A the time fell in proportion to tau_r all the way down to 8 ms (0.455 s) with no
minimum. Repeated with Id = Iq = 20 A, 600 to 2400 ERPM:

| tau_r (ms) | 25 | 32 | **40** | 50 | 63 |
|---|---|---|---|---|---|
| Mean time (s) | 1.024 | 0.925 | **0.897** | 0.917 | 1.017 |

**tau_r = 40 ms**, the same as the release-decay value at every current from 20 to 150 A.
The 30 ms above (Iq 15 A, Id 20 A) was pulled low by the same effect. Run the sweep with
`iq` equal to Id_mag.

## 11. Ramping up

Only after steps 6 to 10 pass. In this order, one at a time: raise Max ERPM, ACIM Current
Limit and Motor Current Max; then move from the bench supply to the 48 V battery, set the
voltage limits and battery cutoffs back to 48 V values, and put Battery Current Min back
to a real regen limit. Re-run `first_spin.lisp` after each change.

Bench result, 2026-10-09, firmware df41aff2, 14S battery at 52.6 V, no load on the shaft:

| Change | Test | Result |
|---|---|---|
| Max ERPM 3000 to 6000 | 5 A, 3 s; 10 A to 4500 ERPM | 1194 ERPM; 4497 ERPM in 5.7 s. No faults |
| Motor Current Max 60 A, Absolute Max 90 A, Battery Current Max 60 A, ACIM Current Limit 60 A | 5 A, 3 s; 30 A to 4500 ERPM | 0 to 4607 ERPM in 1.8 s at Iq 30 A. Slip sat at the 8 Hz `slip_max` clamp (30 A / (2 pi x 30 ms x 20 A) = 8 Hz): above about 30 A of Iq, raise Id_mag towards the knee (30 to 35 A) or slip_max |
| Battery Current Min -20 A, Motor Current Max Brake -20 A | +20 A to 4000 ERPM, then -20 A to 500 ERPM | Braked in 1.16 s, battery current -0.4 A peak (about 24 W of braking power at 1000 rpm, as expected with only the rotor inertia). No faults |

Id_mag 75 A (half the ~150 A saturation point, see step 5): 5 A Iq reaches 5832 ERPM in
3 s (1194 at Id_mag 20 A); 20 A Iq gives about 240 rad/s^2, about 1.1 Nm with the 0.0046
kg m^2 rotor, against 0.9 Nm predicted. No faults, FETs under 37 C.

Battery current check against the ANT BMS (12 s open-loop hold, 120 A at 1500 rpm, no load):
BMS 6.0 to 6.1 A at 51.15 to 51.19 V (307 to 312 W); VESC estimate 4.57 A at 51.7 V (236 W).
**The VESC battery current reads about 25% low here**: the HI200 has no input current sensor,
and the estimate (modulation x phase current) leaves out the inverter's own losses, about
75 W at 120 A phase current. The VESC voltage reads about 0.4 to 0.5 V (1%) high, also at rest.
Use the BMS for power and efficiency.

The pack sagged 0.86 V at 6 A (cells 3.716 to 3.653 V average): about **0.14 ohm**. At 60 A
that is about 8 V, which puts the pack at the 44.8 V start of the low-voltage cutback, so the
battery's internal resistance, not its current rating, limits bench power.

Second 14S pack (no BMS), same hold: rest 53.83 V before and 53.88 V after, 53.48 V under
load (VESC readings, offset cancels), so 0.38 V of sag at about 5.8 A (same ~310 W operating
point; VESC estimate again 4.60 A). About **0.066 ohm**, half the first pack: about 47 V at
100 A, above the cutback. Use this pack for high-current work. Longer pulls sag more as the
cells polarise.

Iq steps at Id_mag 75 A, tau_r 40 ms, second pack at 53.8 V, two runs each, 1000 to 4000 ERPM:

| Iq (A) | Time (s) | Accel (rad/s^2) | vs 20 A | Slip (Hz) | Battery, VESC est. (A) |
|---|---|---|---|---|---|
| 10 | 0.862 | 91 | 0.46 | 0.3 | 3.3 |
| 20 | 0.396 | 198 | 1.00 | 0.9 | 4.2 |
| 40 | 0.201 | 391 | 1.97 | 1.9 | 6.8 |
| 60 | 0.136 | 577 | 2.91 | 2.9 | 10.4 |

Torque is linear in Iq to 60 A (3% roll-off at the top; the 10 A shortfall fits a few
hundredths of a Nm of friction). The model gives about 0.045 Nm per A of Iq at 75 A flux,
2.7 Nm at 60 A, but J came from the same model, so an absolute torque check (torque arm and
scale) is still needed. No faults, FETs under 30 C.

First load test, a friction pad on the pulley (Id_mag 75 A, second pack):
- Torque control is the wrong mode for dry friction (nearly constant torque): below the
  friction torque the rotor stalls, above it it accelerates away. Reverse breakaway was at
  Iq -24 A (about 1.1 Nm on the model); coast-down from 4400 ERPM gave about 0.4 Nm on a
  lighter setting.
- **VESC speed mode (set-rpm) produced no Iq with ACIM enabled**: Iq stayed 0 for 5 s at a
  -4000 ERPM setpoint with the rotor stopped. Open issue, cause not found yet. Also, ACIM
  clamps Id_mag to Motor Current Max, so lowering that limit lowers the flux.
- A LispBM PI speed loop (`speed_pi.lisp`: PI on encoder speed into set-current, Iq clamped
  to 30 A) held -4000 ERPM (1020 rpm) at Iq about -27.7 A, about 1.25 Nm and 133 W
  mechanical on the model. Battery about 270 W (VESC estimate x 1.25), so about 50%
  efficiency: 75 A of flux costs about 65 W of copper loss at this light load. The pad then
  heated and gripped harder until the rotor stalled at the 30 A clamp.
- A friction pad is fine for spot checks, not for steady power: a controllable load is
  needed, and flux reduction at light load is worth adding.

Firmware 8f0739f1 (fixes 244af82d and 8f0739f1), 2026-10-09:
- The "speed mode gives no Iq" issue above was ACIM stuck in FLUXING: the test had lowered
  Motor Current Max to 40 A, below 1.1 x Id_mag, so i_mr never reached 0.9 x Id_mag. Fixed by
  capping the flux target at 90% of the available current. Same test now: i_mr 36 A, RUN at
  0.16 s, Iq tracks the speed loop's -8 to -12 A. VESC speed mode itself was fine.
- Light-load flux reduction, `(acim-flux-opt 0.3)`, LispBM speed loop at -2000 ERPM under the
  friction pad: off, i_mr 75.0 A / Iq -21.9 A (78 A total); on, i_mr 43.0 A / Iq -43.4 A
  (61 A total, Id = Iq as designed) for 14% more torque as the pad tightened. About 39% less
  copper loss. The VESC battery estimate (2.30 vs 2.15 A) is too coarse to show it, and the
  third leg (off again) stalled at the 30 A clamp as the friction kept rising. Needs the BMS
  pack and a steady load for a real efficiency number.
- Reflashing reset both configs again; restore_bench_config.sh put everything back.

Not done yet: steady load, absolute torque (torque arm), efficiency map.

## Reporting back

Send the filled tables and the full printouts of `no_load.lisp` and `acim_enc_check`.
`acim_status` output after any surprise. Screenshots of the Experiment plot help for
anything in steps 6 to 10.
