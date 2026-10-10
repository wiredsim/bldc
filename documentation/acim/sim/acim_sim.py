#!/usr/bin/env python3
"""
ACIM (squirrel-cage induction motor) + VESC-style controller simulation.

Phase 3 of the ACIM work: check, before touching firmware, that the control proposed in
../DESIGN.md converges with correct parameters and degrades gracefully when tau_r is off.

Plant: linear dynamic induction-motor model in the stationary (alpha/beta) frame,
state = stator flux, rotor flux, mechanical speed and angle; integrated with RK4 at
SUBSTEPS per control period.

Controller: mirrors what the firmware will do, at the VESC sample rate:
  - one control update per sample (100_250: low-side shunts -> fs = f_zv / 2)
  - measured currents with noise, encoder angle quantized to the ABI count
  - voltage computed at sample k is applied over the next whole period (1 period delay + ZOH)
  - VESC-style PI current loops (kp = L/tc, ki = R/tc), cross + BEMF decoupling,
    voltage-circle limit with d-axis priority and integrator clamping
  - ACIM estimator, flux state machine, slip clamp and fault checks as in DESIGN.md
  - optional sensorless hybrid voltage/current-model observer with I/f start

All controller code is written with plain floats and per-sample updates so it maps 1:1 to C.
"""

import math
import random

TWO_PI = 2.0 * math.pi


def norm_angle(a):
    while a > math.pi:
        a -= TWO_PI
    while a < -math.pi:
        a += TWO_PI
    return a


# ---------------------------------------------------------------------------------------
# Plant
# ---------------------------------------------------------------------------------------

class MotorParams:
    """True motor parameters. Defaults are a plausible guess for a 48 V belt
    starter-generator (about 15 kW peak); the real values are unknown until Phase 5."""

    def __init__(self, **kw):
        self.pp = 6            # pole pairs
        self.Rs = 0.008        # stator resistance [ohm]
        self.Rr = 0.003        # rotor resistance referred to stator [ohm]
        self.Lm = 120e-6       # magnetizing inductance [H]
        self.Lls = 6e-6        # stator leakage [H]
        self.Llr = 6e-6        # rotor leakage [H]
        self.J = 0.01          # inertia incl. pulley [kg m^2]
        self.B = 0.0005        # viscous friction [Nm s/rad]
        self.i_sat = 0.0       # magnetizing-current saturation knee [A]; 0 = linear iron
        for k, v in kw.items():
            setattr(self, k, v)

    @property
    def Ls(self):
        return self.Lm + self.Lls

    @property
    def Lr(self):
        return self.Lm + self.Llr

    @property
    def tau_r(self):
        return self.Lr / self.Rr

    @property
    def sigma_Ls(self):
        return self.Ls - self.Lm * self.Lm / self.Lr


class Plant:
    def __init__(self, mp, load=None, fixed_speed_rpm=None):
        self.mp = mp
        self.load = load or (lambda wm, t: 0.0)
        self.fixed_wm = None if fixed_speed_rpm is None else fixed_speed_rpm * TWO_PI / 60.0
        self.x = [0.0] * 6  # psa, psb, pra, prb, wm, thm
        if self.fixed_wm is not None:
            self.x[4] = self.fixed_wm
        self.t = 0.0
        mp_ = mp
        self.D = mp_.Ls * mp_.Lr - mp_.Lm * mp_.Lm

    def psi_m(self, im):
        """Magnetizing flux vs magnetizing current magnitude (soft saturation)."""
        mp = self.mp
        if mp.i_sat <= 0.0:
            return mp.Lm * im
        return mp.Lm * im / (1.0 + (im / mp.i_sat) ** 4) ** 0.25

    def currents(self, x=None):
        x = x or self.x
        mp = self.mp
        if mp.i_sat <= 0.0:
            D = self.D
            isa = (mp.Lr * x[0] - mp.Lm * x[2]) / D
            isb = (mp.Lr * x[1] - mp.Lm * x[3]) / D
            ira = (mp.Ls * x[2] - mp.Lm * x[0]) / D
            irb = (mp.Ls * x[3] - mp.Lm * x[1]) / D
            return isa, isb, ira, irb
        # psi_s = Lls*is + psi_m, psi_r = Llr*ir + psi_m, psi_m = f(|im|) along im = is + ir.
        # With A = psi_s/Lls + psi_r/Llr and G = 1/Lls + 1/Llr: |A| = |im| + G*f(|im|).
        G = 1.0 / mp.Lls + 1.0 / mp.Llr
        Aa = x[0] / mp.Lls + x[2] / mp.Llr
        Ab = x[1] / mp.Lls + x[3] / mp.Llr
        A = math.hypot(Aa, Ab)
        if A < 1e-9:
            pma = pmb = 0.0
        else:
            im = A / (1.0 + G * mp.Lm)   # linear guess, then Newton
            for _ in range(20):
                h = 1e-3 * max(im, 1.0)
                g = im + G * self.psi_m(im) - A
                dg = 1.0 + G * (self.psi_m(im + h) - self.psi_m(im - h)) / (2 * h)
                step = g / dg
                im -= step
                if abs(step) < 1e-6:
                    break
            pm = self.psi_m(im)
            pma = pm * Aa / A
            pmb = pm * Ab / A
        isa = (x[0] - pma) / mp.Lls
        isb = (x[1] - pmb) / mp.Lls
        ira = (x[2] - pma) / mp.Llr
        irb = (x[3] - pmb) / mp.Llr
        return isa, isb, ira, irb

    def torque(self, x=None):
        x = x or self.x
        isa, isb, _, _ = self.currents(x)
        return 1.5 * self.mp.pp * (x[0] * isb - x[1] * isa)

    def deriv(self, x, va, vb, t):
        mp = self.mp
        isa, isb, ira, irb = self.currents(x)
        wr = mp.pp * x[4]
        dpsa = va - mp.Rs * isa
        dpsb = vb - mp.Rs * isb
        dpra = -mp.Rr * ira - wr * x[3]
        dprb = -mp.Rr * irb + wr * x[2]
        if self.fixed_wm is not None:
            dwm = 0.0
        else:
            te = 1.5 * mp.pp * (x[0] * isb - x[1] * isa)
            dwm = (te - self.load(x[4], t) - mp.B * x[4]) / mp.J
        return [dpsa, dpsb, dpra, dprb, dwm, x[4]]

    def step_open(self, dt, substeps):
        """All switches off: no stator current (diodes not conducting while the BEMF is
        below the bus voltage). Rotor flux decays with the open-circuit rotor time constant."""
        mp = self.mp
        h = dt / substeps
        x = list(self.x)
        for _ in range(substeps):
            wr = mp.pp * x[4]
            dpra = -(mp.Rr / mp.Lr) * x[2] - wr * x[3]
            dprb = -(mp.Rr / mp.Lr) * x[3] + wr * x[2]
            x[2] += h * dpra
            x[3] += h * dprb
            if self.fixed_wm is None:
                x[4] += h * (-self.load(x[4], self.t) - mp.B * x[4]) / mp.J
            x[5] += h * x[4]
        x[0] = mp.Lm / mp.Lr * x[2]
        x[1] = mp.Lm / mp.Lr * x[3]
        self.x = x
        self.t += dt

    def step(self, va, vb, dt, substeps):
        h = dt / substeps
        x = self.x
        t = self.t
        for _ in range(substeps):
            k1 = self.deriv(x, va, vb, t)
            x2 = [x[i] + 0.5 * h * k1[i] for i in range(6)]
            k2 = self.deriv(x2, va, vb, t + 0.5 * h)
            x3 = [x[i] + 0.5 * h * k2[i] for i in range(6)]
            k3 = self.deriv(x3, va, vb, t + 0.5 * h)
            x4 = [x[i] + h * k3[i] for i in range(6)]
            k4 = self.deriv(x4, va, vb, t + h)
            x = [x[i] + h / 6.0 * (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]) for i in range(6)]
            t += h
        self.x = x
        self.t = t

    @property
    def rotor_flux_angle(self):
        return math.atan2(self.x[3], self.x[2])

    @property
    def rotor_flux_mag(self):
        return math.hypot(self.x[2], self.x[3])

    @property
    def rpm(self):
        return self.x[4] * 60.0 / TWO_PI


# ---------------------------------------------------------------------------------------
# Controller (what goes into motor/acim.c)
# ---------------------------------------------------------------------------------------

ST_OFF, ST_FLUXING, ST_RUN, ST_FAULT = 0, 1, 2, 3
ST_NAMES = {ST_OFF: "OFF", ST_FLUXING: "FLUXING", ST_RUN: "RUN", ST_FAULT: "FAULT"}
SRC_ENCODER, SRC_SENSORLESS = 0, 1


class AcimConf:
    """Mirrors the 'ACIM' custom-config page in DESIGN.md section 4."""

    def __init__(self, **kw):
        self.speed_src = SRC_ENCODER
        self.id_mag = 100.0          # A
        self.tau_r = 0.042           # s (controller's belief)
        self.lm = 120e-6             # H (controller's belief)
        self.lr_lm = 1.05            # Lr/Lm (controller's belief)
        self.slip_max_hz = 8.0       # electrical Hz
        self.flux_build_time = 0.0   # 0 -> 3*tau_r
        self.current_max = 300.0     # A, |I| cap in ACIM mode
        self.sl_min_hz = 10.0
        self.sl_if_ramp = 20.0       # Hz/s
        self.obs_bw = 3.0            # Hz
        self.fault_flux_err = 0.5
        self.fault_slip_fac = 3.0
        for k, v in kw.items():
            setattr(self, k, v)


class VescConf:
    """The stock mcconf values the ACIM mode relies on."""

    def __init__(self, mp, **kw):
        self.f_zv = 30000.0
        self.fs = self.f_zv / 2.0          # low-side shunts: sample in V0 only
        self.foc_motor_r = mp.Rs           # 'Measure R and L' result
        self.foc_motor_l = mp.sigma_Ls     # VESC measures the transient inductance
        self.tc = 500e-6                   # current loop time constant
        self.v_bus = 48.0
        self.max_duty = 0.95
        self.enc_counts = 4000             # AS5047P ABI default
        self.pp = mp.pp                    # foc_encoder_ratio
        self.enc_dir = 1                   # foc_encoder_inverted -> -1
        self.curr_noise = 0.5              # A rms on each phase sample
        for k, v in kw.items():
            setattr(self, k, v)

    @property
    def kp(self):
        return self.foc_motor_l / self.tc

    @property
    def ki(self):
        return self.foc_motor_r / self.tc


class AcimController:
    def __init__(self, vc, ac):
        self.vc = vc
        self.ac = ac
        self.dt = 1.0 / vc.fs
        self.reset()

    def reset(self):
        # estimator (flux in amps, i_mr = psi_r / Lm)
        self.imr_a = 0.0
        self.imr_b = 0.0
        self.theta = 0.0
        # encoder speed PLL
        self.pll_th = 0.0
        self.pll_w = 0.0
        self.enc_init = False
        # sensorless observer
        self.ps_a = 0.0
        self.ps_b = 0.0
        self.ci_a = 0.0
        self.ci_b = 0.0
        self.vm_th = 0.0
        self.vm_w = 0.0
        self.vm_w_int = 0.0
        self.vm_th_raw = 0.0
        self.vm_th_raw_prev = 0.0
        self.w_sl_obs = 0.0
        self.wr_hat = 0.0
        self.if_active = True
        self.if_th = 0.0
        self.if_f = 0.0
        self.psr_vm_a = 0.0
        self.psr_vm_b = 0.0
        # current loop
        self.vd_int = 0.0
        self.vq_int = 0.0
        self.va_prev = 0.0   # voltage applied during the period that just ended
        self.vb_prev = 0.0
        self.va_now = 0.0    # voltage being applied during the current period
        self.vb_now = 0.0
        self.va_next = 0.0   # voltage computed this sample, applied next period
        self.vb_next = 0.0
        # state machine
        self.state = ST_OFF
        self.t_state = 0.0
        self.fault = None
        self.flux_err_t = 0.0
        self.slip_err_t = 0.0
        self.slip_plaus_t = 0.0
        self.t_handover = 0.0
        # logging
        self.id = self.iq = 0.0
        self.id_ref = self.iq_ref = 0.0
        self.w_s = 0.0
        self.slip = 0.0
        self.wr = 0.0

    # --- helpers ----------------------------------------------------------------------
    def _encoder_speed(self, th_e):
        """Returns (angle increment since last sample, speed estimate)."""
        if not self.enc_init:
            self.pll_th = th_e
            self.th_e_prev = th_e
            self.enc_init = True
        dth = norm_angle(th_e - self.th_e_prev)
        self.th_e_prev = th_e
        # same structure as foc_pll_run(); bandwidth ~ 300 rad/s
        kp, ki = 2.0 * 300.0, 300.0 ** 2
        err = norm_angle(th_e - self.pll_th)
        self.pll_th = norm_angle(self.pll_th + (self.pll_w + kp * err) * self.dt)
        self.pll_w += ki * err * self.dt
        # pll_w alone lags by 2*a/wn under acceleration; pll_w + kp*err does not.
        return dth, self.pll_w + kp * err

    def _current_model(self, ia, ib, dth):
        """i_mr update: exact rotation by the rotor's electrical angle increment dth, then
        exact first-order decay toward i.

        Rotating by the measured encoder angle increment (not by a filtered speed * dt) is
        important: any speed-filter lag during acceleration becomes a slip error directly,
        while summed angle increments are exact up to one count of quantization."""
        ac = self.ac
        s = math.sin(dth)
        c = math.cos(dth)
        a = c * self.imr_a - s * self.imr_b
        b = s * self.imr_a + c * self.imr_b
        k = 1.0 - math.exp(-self.dt / ac.tau_r)   # precomputed in C
        self.imr_a = a + k * (ia - a)
        self.imr_b = b + k * (ib - b)

    def _observer(self, ia, ib, dth_cm):
        """Hybrid flux observer: voltage model corrected toward the current model.

        The current model (i_mr) is rotated by dth_cm: the encoder angle increment in encoder
        mode, the estimated speed * dt in sensorless mode. Returns the voltage-model rotor-flux angle.
        In encoder mode the orientation still comes from the current model; the voltage
        model is only used as an independent plausibility check (wrong encoder direction,
        wrong pole pairs, lost encoder)."""
        ac, vc, dt = self.ac, self.vc, self.dt
        lm = ac.lm
        lr = ac.lm * ac.lr_lm
        sls = vc.foc_motor_l
        wc = TWO_PI * ac.obs_bw
        kp, ki = 2.0 * wc, wc * wc

        self._current_model(ia, ib, dth_cm)
        ps_cm_a = (lm / lr) * lm * self.imr_a + sls * ia
        ps_cm_b = (lm / lr) * lm * self.imr_b + sls * ib

        ea = ps_cm_a - self.ps_a
        eb = ps_cm_b - self.ps_b
        self.ci_a += ki * ea * dt
        self.ci_b += ki * eb * dt
        ua = kp * ea + self.ci_a
        ub = kp * eb + self.ci_b
        rs = vc.foc_motor_r
        self.ps_a += (self.va_prev - rs * ia + ua) * dt
        self.ps_b += (self.vb_prev - rs * ib + ub) * dt

        pra = (lr / lm) * (self.ps_a - sls * ia)
        prb = (lr / lm) * (self.ps_b - sls * ib)
        self.psr_vm_a, self.psr_vm_b = pra, prb
        th = math.atan2(prb, pra)
        self.vm_th_raw_prev = self.vm_th_raw
        self.vm_th_raw = th

        # PLL on the voltage-model flux angle -> stator (synchronous) frequency
        kpp, kip = 2.0 * 400.0, 400.0 ** 2
        err = norm_angle(th - self.vm_th)
        self.vm_th = norm_angle(self.vm_th + (self.vm_w_int + kpp * err) * dt)
        self.vm_w_int += kip * err * dt
        self.vm_w = self.vm_w_int + kpp * err

        mag2 = pra * pra + prb * prb
        if mag2 > 1e-12:
            w_sl = lm * (pra * ib - prb * ia) / (ac.tau_r * mag2)
        else:
            w_sl = 0.0
        self.w_sl_obs = w_sl
        w_r = self.vm_w - w_sl
        # light low-pass on the speed estimate (only used once the observer is in charge)
        if ac.speed_src == SRC_SENSORLESS and self.state == ST_RUN and not self.if_active:
            self.wr_hat += (w_r - self.wr_hat) * min(1.0, dt * TWO_PI * 50.0)
        return th

    def _fault(self, name):
        self.state = ST_FAULT
        self.fault = name
        self.va_next = self.vb_next = 0.0

    # --- main per-sample update -------------------------------------------------------
    def update(self, ia, ib, enc_th_e, run_request, iq_request):
        """Returns (v_alpha, v_beta) to be applied over the next period."""
        ac, vc, dt = self.ac, self.vc, self.dt
        # Voltage pipeline: computed at sample k, applied over [t(k+1), t(k+2)].
        # The period that just ended therefore used the value computed two samples ago.
        self.va_prev, self.vb_prev = self.va_now, self.vb_now
        self.va_now, self.vb_now = self.va_next, self.vb_next

        # 1. Rotor speed and flux angle
        if ac.speed_src == SRC_ENCODER:
            dth, wr = self._encoder_speed(enc_th_e)
            self._observer(ia, ib, dth)
            theta = math.atan2(self.imr_b, self.imr_a)
        else:
            if self.state != ST_RUN:
                # standstill (flux build) or off: nothing to observe, assume no rotation
                self.wr_hat = 0.0
            elif self.if_active:
                # I/f: the rotor is assumed to follow the field at the commanded slip
                self.wr_hat = TWO_PI * self.if_f
            wr = self.wr_hat
            if self.state == ST_RUN and not self.if_active:
                # rotate the current model by the rotor angle increment implied by the
                # observer (flux angle increment minus slip), not by a filtered speed
                dth = norm_angle(self.vm_th_raw - self.vm_th_raw_prev) - self.w_sl_obs * dt
            else:
                dth = wr * dt
            theta = self._observer(ia, ib, dth)
        self.wr = wr

        imr = math.hypot(self.imr_a, self.imr_b)
        if self.state == ST_FAULT:
            self.va_next = self.vb_next = 0.0
            return 0.0, 0.0

        # 2. NaN / magnitude plausibility
        # i_mr is a low-pass of the measured current, so it can only exceed the largest
        # current the loop is allowed to drive through a numerical fault.
        if not math.isfinite(imr) or imr > 1.5 * ac.current_max:
            self._fault("ACIM_FLUX")
            return 0.0, 0.0

        # 3. State machine
        build_t = ac.flux_build_time if ac.flux_build_time > 0 else 3.0 * ac.tau_r
        if not run_request:
            self.state = ST_OFF
            self.t_state = 0.0
        elif self.state == ST_OFF:
            self.state = ST_FLUXING
            self.t_state = 0.0
        self.t_state += dt
        if self.state == ST_FLUXING and imr >= 0.9 * ac.id_mag and self.t_state >= build_t:
            self.state = ST_RUN

        if self.state == ST_OFF:
            # PWM off: currents are zero, model just decays; keep integrators preset
            self.vd_int = self.vq_int = 0.0
            self.va_next = self.vb_next = 0.0
            self.theta = theta
            self.id = self.iq = 0.0
            return 0.0, 0.0

        # 4. Sensorless low-speed: I/f instead of the observer angle
        if ac.speed_src == SRC_SENSORLESS:
            ws_obs = self.vm_w
            if self.if_active:
                if self.state == ST_RUN and abs(iq_request) > 1.0:
                    sgn = 1.0 if iq_request > 0 else -1.0
                    self.if_f += sgn * ac.sl_if_ramp * dt
                elif self.state == ST_RUN:
                    # no torque requested: bring the field to a stop instead of dragging
                    # the rotor along at the last I/f frequency
                    step = ac.sl_if_ramp * dt
                    self.if_f = 0.0 if abs(self.if_f) <= step else self.if_f - math.copysign(step, self.if_f)
                self.if_th = norm_angle(self.if_th + TWO_PI * self.if_f * dt)
                if abs(self.if_f) >= 1.2 * ac.sl_min_hz and abs(ws_obs) > TWO_PI * ac.sl_min_hz:
                    self.if_active = False
                    # hand over: start the observer-driven speed estimate from the I/f
                    # frequency and re-seed the current model from the voltage model
                    self.imr_a = self.psr_vm_a / ac.lm
                    self.imr_b = self.psr_vm_b / ac.lm
                    self.t_handover = 0.0
            else:
                if abs(ws_obs) < TWO_PI * 0.8 * ac.sl_min_hz:
                    self.if_active = True
                    self.if_th = theta
                    self.if_f = ws_obs / TWO_PI
            if self.if_active:
                theta = self.if_th

        # The same angle is used for the Park transform of the sampled currents and the
        # inverse Park of the output voltage (one state->phase in mcpwm_foc.c). It must be
        # the angle at the sample instant: advancing it (as the PM observer path does with
        # foc_observer_offset) biases the measured id/iq and therefore the flux by
        # ~iq*w_s*dt/2. The 1.5-period voltage delay is absorbed by the PI integrators.
        phase = theta
        s, c = math.sin(phase), math.cos(phase)

        # 5. Current references
        id_ref = ac.id_mag
        iq_ref = 0.0
        if self.state == ST_RUN:
            iq_ref = iq_request
            imr_eff = imr if not (ac.speed_src == SRC_SENSORLESS and self.if_active) else ac.id_mag
            iq_slip_max = TWO_PI * ac.slip_max_hz * ac.tau_r * imr_eff
            iq_unclamped = iq_ref
            iq_ref = max(-iq_slip_max, min(iq_slip_max, iq_ref))
            # flux-collapse detector: torque wanted but flux far too low for it
            if abs(iq_unclamped) > ac.fault_slip_fac * iq_slip_max and abs(iq_unclamped) > 0.2 * ac.id_mag:
                self.slip_err_t += dt
            else:
                self.slip_err_t = 0.0
            if self.slip_err_t > 0.2:
                self._fault("ACIM_SLIP")
                return 0.0, 0.0
        if ac.speed_src == SRC_SENSORLESS and self.if_active and self.state == ST_RUN:
            # I/f: one rotating current vector along the I/f angle. The cage follows it
            # like a classic induction motor on a rotating field, which is self-damping;
            # splitting it into id/iq against an angle that is not the real flux angle is not.
            id_ref = math.sqrt(ac.id_mag * ac.id_mag + iq_request * iq_request)
            iq_ref = 0.0
        imax = ac.current_max
        id_ref = min(id_ref, imax)
        iq_lim = math.sqrt(max(imax * imax - id_ref * id_ref, 0.0))
        iq_ref = max(-iq_lim, min(iq_lim, iq_ref))

        # 6. Park transform (measured currents)
        i_d = c * ia + s * ib
        i_q = c * ib - s * ia

        # Stator frequency for decoupling: rotor speed + slip from measured iq
        if ac.speed_src == SRC_SENSORLESS and self.if_active:
            w_s = TWO_PI * self.if_f
        else:
            w_s = wr + (i_q / (ac.tau_r * imr) if imr > 1.0 else 0.0)
        self.w_s = w_s

        # 7. PI current control (VESC structure)
        ed = id_ref - i_d
        eq = iq_ref - i_q
        self.vd_int += ed * vc.ki * dt
        self.vq_int += eq * vc.ki * dt
        vd = self.vd_int + ed * vc.kp
        vq = self.vq_int + eq * vc.kp
        # decoupling: cross terms with sigma*Ls, BEMF from the rotor flux
        sls = vc.foc_motor_l
        vd -= w_s * sls * i_q
        vq += w_s * sls * i_d + w_s * (ac.lm / ac.lr_lm) * imr
        vmax = vc.max_duty * vc.v_bus / math.sqrt(3.0)
        vd = max(-vmax, min(vmax, vd))
        self.vd_int = max(-vmax, min(vmax, self.vd_int))
        vq_max = math.sqrt(max(vmax * vmax - vd * vd, 0.0))
        vq = max(-vq_max, min(vq_max, vq))
        self.vq_int = max(-vq_max, min(vq_max, self.vq_int))

        va = c * vd - s * vq
        vb = s * vd + c * vq
        self.va_next, self.vb_next = va, vb

        # 8. Flux plausibility: voltage model vs current model, once the stator frequency
        #    is high enough for the voltage model to carry information (both modes).
        self.t_handover += dt
        vm_valid = abs(self.vm_w) > TWO_PI * ac.sl_min_hz and self.t_handover > 0.25 and not (
            ac.speed_src == SRC_SENSORLESS and self.if_active)
        if vm_valid and self.state == ST_RUN:
            pcm_a = ac.lm * self.imr_a
            pcm_b = ac.lm * self.imr_b
            m = math.hypot(pcm_a, pcm_b)
            err = math.hypot(self.psr_vm_a - pcm_a, self.psr_vm_b - pcm_b)
            if m > 1e-6 and err / m > ac.fault_flux_err:
                self.flux_err_t += dt
            else:
                self.flux_err_t = 0.0
            if self.flux_err_t > 0.1:
                self._fault("ACIM_FLUX")
                return 0.0, 0.0
            # Encoder mode: slip seen by the voltage model (stator frequency - encoder speed)
            # must match the slip the current model commands. A stuck, slipping or
            # wrongly scaled encoder breaks this long before the flux error grows.
            if ac.speed_src == SRC_ENCODER:
                slip_seen = self.vm_w - wr
                slip_cmd = w_s - wr
                tol = TWO_PI * max(ac.slip_max_hz, 0.25 * abs(self.vm_w) / TWO_PI)
                if abs(slip_seen - slip_cmd) > tol:
                    self.slip_plaus_t += dt
                else:
                    self.slip_plaus_t = 0.0
                if self.slip_plaus_t > 0.05:
                    self._fault("ACIM_SLIP")
                    return 0.0, 0.0

        # logging
        self.theta = theta
        self.id, self.iq = i_d, i_q
        self.id_ref, self.iq_ref = id_ref, iq_ref
        self.slip = (w_s - wr) / TWO_PI
        return va, vb


# ---------------------------------------------------------------------------------------
# Simulation driver
# ---------------------------------------------------------------------------------------

def simulate(mp, vc, ac, t_end, iq_profile, run_profile=None, load=None,
             fixed_speed_rpm=None, substeps=4, seed=1, log_every=10, enc_dir_true=1):
    """
    iq_profile(t) -> requested Iq [A] (what mc_interface_set_current would ask for).
    run_profile(t) -> bool, motor enabled. Defaults to always on.
    enc_dir_true: -1 simulates an encoder wired backwards relative to the config.
    """
    rnd = random.Random(seed)
    plant = Plant(mp, load=load, fixed_speed_rpm=fixed_speed_rpm)
    ctl = AcimController(vc, ac)
    dt = ctl.dt
    va = vb = 0.0
    pwm_on = False
    log = {k: [] for k in ("t", "rpm", "te", "te_ideal", "psr", "psr_ref", "imr", "ang_err",
                           "id", "iq", "id_ref", "iq_ref", "slip", "state", "wr_true", "wr_est",
                           "i_abs", "vmag", "if_active")}
    n = int(round(t_end / dt))
    run_profile = run_profile or (lambda t: True)
    for k in range(n):
        t = k * dt
        isa, isb, _, _ = plant.currents()
        ia = isa + rnd.gauss(0.0, vc.curr_noise)
        ib = isb + rnd.gauss(0.0, vc.curr_noise)
        th_m = plant.x[5]
        cnt = math.floor((enc_dir_true * th_m) / TWO_PI * vc.enc_counts)
        th_e = norm_angle(vc.enc_dir * cnt / vc.enc_counts * TWO_PI * vc.pp)
        run = run_profile(t)
        psr_true = plant.rotor_flux_mag
        ang_true = plant.rotor_flux_angle
        te_now = plant.torque()
        rpm_now = plant.rpm
        va_new, vb_new = ctl.update(ia, ib, th_e, run, iq_profile(t))
        # One-period delay: the voltage computed now is applied over the next period.
        if pwm_on:
            plant.step(va, vb, dt, substeps)
        else:
            plant.step_open(dt, substeps)
        pwm_on = run and ctl.state not in (ST_FAULT, ST_OFF)
        va, vb = (va_new, vb_new) if pwm_on else (0.0, 0.0)
        if k % log_every == 0:
            log["t"].append(t)
            log["rpm"].append(rpm_now)
            log["te"].append(te_now)
            # ideal torque for the *commanded* id/iq with perfect orientation
            log["te_ideal"].append(1.5 * mp.pp * mp.Lm ** 2 / mp.Lr * ctl.id_ref * ctl.iq_ref
                                   if ctl.state == ST_RUN else 0.0)
            log["psr"].append(psr_true)
            log["psr_ref"].append(mp.Lm * ac.id_mag)
            log["imr"].append(math.hypot(ctl.imr_a, ctl.imr_b))
            log["ang_err"].append(math.degrees(norm_angle(ctl.theta - ang_true)) if psr_true > 1e-4 else 0.0)
            log["id"].append(ctl.id)
            log["iq"].append(ctl.iq)
            log["id_ref"].append(ctl.id_ref)
            log["iq_ref"].append(ctl.iq_ref)
            log["slip"].append(ctl.slip)
            log["state"].append(ctl.state)
            log["wr_true"].append(mp.pp * rpm_now / 60.0)
            log["wr_est"].append(ctl.wr / TWO_PI)
            log["i_abs"].append(math.hypot(isa, isb))
            log["vmag"].append(math.hypot(va, vb))
            log["if_active"].append(1 if (ac.speed_src == SRC_SENSORLESS and ctl.if_active) else 0)
    return log, ctl, plant


def steady(log, key, t0, t1):
    vals = [v for t, v in zip(log["t"], log[key]) if t0 <= t <= t1]
    return sum(vals) / len(vals) if vals else float("nan")
