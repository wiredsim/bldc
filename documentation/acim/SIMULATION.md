# ACIM Phase 3: simulation results

Code: `sim/acim_sim.py` (motor model + controller) and `sim/run_scenarios.py` (scenarios, plots,
table). Run `python3 run_scenarios.py` (numpy not needed, matplotlib for plots, about 2 minutes).
Plots are in `sim/out/`, the full table in `sim/out/results.md`.

## What is simulated

**Motor.** A dynamic induction-motor model in stator coordinates (stator and rotor flux
states, RK4, 4 substeps per control period). The true parameters are a guess for the
eTorque BSG, since the real ones are unknown until Phase 5: 6 pole pairs, Rs 8 mOhm,
Rr 3 mOhm, Lm 120 uH, leakage 6 uH + 6 uH (tau_r = 42 ms, sigma*Ls = 11.7 uH),
J = 0.01 kg m^2. Runs are repeated with a **saturating** magnetizing curve (knee at 130 A),
because whether over-fluxing helps or hurts depends entirely on saturation. PWM off is
modelled as all switches open (no stator current, rotor flux decays at tau_r), not as a
short.

**Controller.** Written per-sample with plain floats so it ports 1:1 to C. It matches
the 100_250 hardware: 30 kHz switching with low-side shunts, so one control update per
period at 15 kHz. The voltage computed at a sample is applied one full period later
(slightly worse than the real board). Measured currents carry 0.5 A rms noise, and the
encoder is quantized to 4000 counts/rev. It uses the stock VESC PI current loop
(kp = L/tc, ki = R/tc, tc = 500 us), cross + BEMF decoupling, the voltage-circle limit with
d priority, and the ACIM estimator, state machine, slip clamp and faults from DESIGN.md.
Id_mag is 100 A.

## Results

### Encoder IFOC converges with correct parameters

Scenario A (`A_encoder_cycle_*.png`): flux build, then Iq 150 A, 250 A (slip-clamped to
211 A), -150 A regen, then 0, against a fan load.

| | Torque / ideal | Flux / target | Flux angle error |
|---|---|---|---|
| Linear iron, tau_r correct | 0.94-1.00 | 0.99-1.00 | < 0.3 deg |
| Saturating iron, tau_r correct | 0.93-0.95 | 0.96-0.97 | -2.5 deg |

With saturating iron the shortfall is the iron itself: "ideal" assumes unsaturated Lm,
and the effective tau_r drops as Lm saturates. The 0.94 at 250 A (linear iron) is measured
while the motor is still accelerating hard against the fan load. There are no oscillations
or faults in any case.

### tau_r off by ±30% degrades gracefully

Scenario B (`B_tau_r_sweep.png`): the speed is held by a dyno at 300, 1500 and 3000 rpm,
with Iq 150 A and tau_r swept from 70% to 130% of true.

| controller tau_r | torque vs correct tau_r (sat iron, 300-1500 rpm) | flux | angle error |
|---|---|---|---|
| 70% | 0.86 | 0.76 | +8 deg |
| 85% | 0.96 | 0.87 | +3 deg |
| 100% | 1.00 | 0.96 | -2 deg |
| 115% | 1.00 | 1.03 | -7 deg |
| 130% | 0.97 | 1.09 | -12 deg |

- Errors in either direction never caused instability, a current-limit violation or a
  false fault. The error just costs a predictable amount of torque per amp.
- **Over-estimating tau_r is the risky side at high speed.** It raises the flux, so at
  3000 rpm (300 Hz electrical, near the 48 V voltage limit) the current loop runs out of
  voltage. Torque drops to 0.76 at +30% with saturating iron, and to 0.48 with linear iron.
  **Recommendation: start the tau_r sweep from a low guess and work upward**, and do the
  sweep at moderate speed.
- With linear iron, a higher tau_r always looks better at low speed because the extra flux
  costs nothing. Real iron saturates, so the sweep on the bench will show a flat peak
  around the true value, as in the dashed curves.

### Flux build-up

Scenario C (`C_flux_buildup.png`): with 150 A requested from t = 0, Iq is held at 0 for
126 ms (3·tau_r) while the estimated flux follows the true flux. Then Iq is released
cleanly.

### Sensorless works above about 10 Hz electrical, with caveats

Scenario D (`D_sensorless.png`, saturating iron):
1. Flux build, then I/f start at 20 Hz/s.
2. Handover to the observer at 12 Hz.
3. A 6 Nm load step.
4. Braking at -60 A back down into I/f.
5. Zero request.

| Case | Steady torque / ideal | Flux angle error rms (closed loop) | Speed estimate error rms |
|---|---|---|---|
| nominal | 0.92 (iron) | 2.0 deg | 1.7 Hz |
| tau_r -30% | 0.92 | 1.9 deg | 3.1 Hz |
| tau_r +30% | 0.92 | 2.0 deg | 1.7 Hz |
| Rs +30% | 0.92 | 1.8 deg | 1.6 Hz |
| Rs -30% | 0.92 | 10.3 deg | 6.2 Hz |

- **tau_r matters much less sensorless**, because once the voltage model dominates the
  orientation comes from it. It is no longer a current-model output.
- **Rs is the weak point.** On this low-voltage, high-current motor, the IR drop
  (8 mOhm × 150 A ≈ 1.2 V) is larger than the back-EMF below roughly 15 Hz. With Rs
  under-estimated by 30%, the observer rings for about 0.5 s after handover and settles only
  as speed rises.
- The first version handed over at 6 Hz and failed badly with Rs -30%, which is why the
  default is now 10 Hz (handover at 12 Hz). VESC's temperature compensation of Rs
  (`foc_temp_comp`) should be on.
- Braking back toward standstill hands control back to I/f below 8 Hz (0.8 × `acim_sl_min_hz`).
  The first version used 0.6 ×, and with Rs +30% the observer then held on too long and let
  the rotor creep at zero request. At 0.8 × all five cases return to I/f cleanly and stop.
  Low-speed braking is still the weakest part of sensorless mode.

Verdict: sensorless is good enough for first spins at moderate speed. The encoder is
needed for low-speed torque and for the tau_r sweep.

### Faults

Scenario E:

| Case | Result |
|---|---|
| Pole pairs set to 4 or 8 (true 6) | `ACIM_FLUX` 150-165 ms after torque is requested |
| Encoder signal lost at full speed | `ACIM_SLIP` after 51 ms |
| Encoder wired backwards | **not detected**: the motor stalls at under 120 rpm and draws current. Safe, but silent |
| Correct setup, tau_r ±30%, all scenarios | no false faults |

The backwards-encoder case cannot be caught while running near standstill, because the
voltage model has no information there. It needs a commissioning check instead (below).

## What the simulation changed in the design

These are folded into DESIGN.md.

1. **No angle advance on the Park transform.** The PM-observer path advances the angle by
   half a period. Doing the same in ACIM mode biased the measured id/iq and cut the flux by
   about 3.5% at 1000 rpm. ACIM uses the angle at the sample instant, and the PI
   integrators absorb the output delay.
2. **Rotate the current model by encoder angle increments, not speed × dt.** The first
   version used the PLL speed, which lags by 2a/wn during acceleration. That produced a
   **39 deg** flux angle error and +70% flux at full acceleration. Using the raw encoder
   angle increment brings this down to 0.3 deg. For the same reason the speed estimate is
   `pll_w + kp·err`, not the PLL integrator alone.
3. **The voltage-model observer runs in encoder mode too**, purely as a plausibility
   check. It gives two faults:
   - `ACIM_FLUX`: the voltage-model and current-model flux disagree by more than 50% for
     100 ms.
   - `ACIM_SLIP`: the slip seen by the voltage model disagrees with the commanded slip for
     50 ms.

   Both are enabled once the stator frequency exceeds `acim_sl_min_hz`, with a 250 ms
   blanking time after handover. The magnitude sanity limit is now 1.5 × `acim_current_max`.
   The current-model flux is a low-pass of the measured current, so it can only exceed that
   through a numerical fault.
4. **Sensorless I/f uses a single rotating current vector** (magnitude √(Id² + Iq²) along the
   I/f angle), not id/iq against an angle that isn't the flux angle. The split version
   rocked the rotor back and forth. With no torque request, the I/f frequency ramps to zero
   instead of dragging the rotor along.
5. **`acim_sl_min_hz` default 5 → 10 Hz**, because of the Rs sensitivity above. Handover to
   the observer at 1.2 × (12 Hz), hand-back to I/f at 0.8 × (8 Hz).
6. **New terminal command `acim_enc_check`** for commissioning. It rotates the field
   open-loop at low current through a few electrical turns and compares the encoder
   direction and counts per turn with `foc_encoder_inverted` and `foc_encoder_ratio`. This
   catches the backwards-encoder case the runtime faults can't.

## Limits of this simulation

- The motor parameters are guesses. The structure of the results carries over, the exact
  numbers don't.
- Dead time, PWM ripple, ADC timing and the board's current-sense filters are not modelled.
  The one-period voltage delay is conservative.
- There is no thermal model, so Rs and Rr are constant. A real rotor heating up changes tau_r
  by about 30% from cold to hot. That is exactly the ±30% case above, so it is covered as a
  steady-state error.
- Flux weakening above base speed is not implemented. It is the next thing the voltage limit
  in scenario B asks for.
