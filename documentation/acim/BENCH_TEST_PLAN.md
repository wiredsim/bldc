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

## 0. Safety and the bench

- **Clamp the motor**, take the belt off and guard the pulley. Everything here runs at no
  load. Keep a hand on the supply switch.
- **The bench supply cannot absorb regen.** An induction motor does not generate once it
  is unpowered (no magnets, and the flux dies within a few tau_r), so cutting the power is
  always safe. Regen only happens while the controller is braking. Keep it small:
  - Motor > General > Current: **Battery Current Min (regen) = -1 A**. Battery Current Max
    at or below what the supply delivers.
  - Motor > General > Voltage: **Maximum Input Voltage = 26 V**. An overvoltage fault cuts
    the PWM, which is safe on this motor.
  - In sensorless mode the I/f start slows the field to zero when the request ends, which
    drags the rotor down. The input current limit does not cover that, so keep those
    speeds low (the scripts do). A 48 V battery instead of the supply removes the problem.
- **Speed limit**: Motor > General > RPM: **Max ERPM = 3000, Min ERPM = -3000** for steps 6
  to 9. ERPM is electrical: 3000 ERPM is 500 rpm at 6 pole pairs.
- **Current**: Motor Current Max 60 A and Motor Current Max Brake -20 A for steps 4 and 5.
  Leave Absolute Maximum Current at the default. The ACIM page has its own current limit
  on top of these (ACIM Current Limit, default 40 A).
- Check that the battery voltage cutoffs (Motor > General > Voltage) are below 20.5 V;
  a 48 V setup has them far above it, and they would cut the current.

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

| Run | Current (A) | Shaft turns | 300 / turns | Pole pairs |
|---|---|---|---|---|
| 1 | | | | |

Enter the result as Motor Poles (= 2 x pole pairs) under Motor > Additional Info.

## 5. Magnetizing curve and tau_r (no encoder)

Run `no_load.lisp`. It spins the motor open-loop at 6000 ERPM (100 Hz electrical, about
1000 rpm at 6 pole pairs), steps the current from 60 A down to 10 A, and then lets go and
times the voltage decay. Expected output, one line per current, then tau_r:

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
| | | | | | |
| | | | | | |
| | | | | | |
| | | | | | |

## 11. Ramping up

Only after steps 6 to 10 pass. In this order, one at a time: raise Max ERPM, ACIM Current
Limit and Motor Current Max; then move from the bench supply to the 48 V battery, set the
voltage limits and battery cutoffs back to 48 V values, and put Battery Current Min back
to a real regen limit. Re-run `first_spin.lisp` after each change.

## Reporting back

Send the filled tables and the full printouts of `no_load.lisp` and `acim_enc_check`.
`acim_status` output after any surprise. Screenshots of the Experiment plot help for
anything in steps 6 to 10.
