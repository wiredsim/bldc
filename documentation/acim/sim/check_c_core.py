#!/usr/bin/env python3
"""Port check: runs motor/acim_core.c (the firmware code, compiled for the host) against
the Python motor model and compares it with the Python controller from Phase 3.

    python3 check_c_core.py

Needs gcc. Writes out/c_core_check.md.
"""

import ctypes
import math
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import acim_sim as S          # noqa: E402
import run_scenarios as R     # noqa: E402

OUT = os.path.join(HERE, "out")
LIB = os.path.join(OUT, "libacimcore.so")


def build():
    os.makedirs(OUT, exist_ok=True)
    cmd = ["gcc", "-O2", "-shared", "-fPIC", "-std=gnu99", "-Wall", "-Wextra",
           "-fsingle-precision-constant", "-Wdouble-promotion",
           os.path.join(HERE, "acim_core_host.c"), "-o", LIB, "-lm"]
    subprocess.check_call(cmd)
    lib = ctypes.CDLL(LIB)
    f = ctypes.c_float
    lib.h_conf.argtypes = [ctypes.c_int] + [f] * 13
    lib.h_set_i_avail.argtypes = [f]
    lib.h_update.argtypes = [f, ctypes.c_int, f, f, f, f, f, ctypes.c_int, f, f, f, f, ctypes.POINTER(f)]
    return lib


LIBC = None
STATE_MAP = {0: S.ST_OFF, 1: S.ST_FLUXING, 2: S.ST_RUN, 3: S.ST_RUN, 4: S.ST_FAULT}
FAULT_MAP = {1: "ACIM_FLUX", 2: "ACIM_SLIP", 3: "ENCODER"}


class CCoreController:
    """Same interface as acim_sim.AcimController, with the estimator, state machine, faults
    and references from the C core. The PI current loop around it is the Python copy of
    VESC's control_current(), identical to the one in AcimController."""

    def __init__(self, vc, ac):
        self.vc, self.ac = vc, ac
        self.i_avail = getattr(ac, "i_avail", 0.0)
        LIBC.h_set_i_avail(self.i_avail) if LIBC is not None else None
        self.dt = 1.0 / vc.fs
        LIBC.h_reset()
        LIBC.h_conf(1 if ac.speed_src == S.SRC_SENSORLESS else 0, ac.id_mag, ac.tau_r, ac.lm, ac.lr_lm,
                    ac.slip_max_hz, ac.flux_build_time, 0.5, ac.current_max, ac.sl_min_hz,
                    ac.sl_if_ramp, ac.obs_bw, ac.fault_flux_err, ac.fault_slip_fac)
        self.res = (ctypes.c_float * 15)()
        self.vd_int = self.vq_int = 0.0
        self.va_now = self.vb_now = self.va_next = self.vb_next = 0.0
        self.state = S.ST_OFF
        self.fault = None
        self.id = self.iq = self.id_ref = self.iq_ref = 0.0
        self.imr_a = self.imr_b = 0.0
        self.theta = 0.0
        self.slip = 0.0
        self.wr = 0.0
        self.if_active = True

    def update(self, ia, ib, enc_th_e, run_request, iq_request):
        vc, ac, dt = self.vc, self.ac, self.dt
        va_prev, vb_prev = self.va_now, self.vb_now
        self.va_now, self.vb_now = self.va_next, self.vb_next

        r = self.res
        LIBC.h_update(dt, 1 if run_request else 0, ia, ib, va_prev, vb_prev, enc_th_e, 1,
                      iq_request, 1.0, vc.foc_motor_r, vc.foc_motor_l, r)
        phase, id_ref, iq_ref, fault, state = r[0], r[1], r[2], int(r[3]), int(r[4])
        if self.i_avail > 0.0:
            # mcpwm_foc.c truncates id_set to the current limit and iq_set to what is left
            id_ref = max(-self.i_avail, min(self.i_avail, id_ref))
            iq_lim = math.sqrt(max(self.i_avail ** 2 - id_ref ** 2, 0.0))
            iq_ref = max(-iq_lim, min(iq_lim, iq_ref))
        imr, w_s, self.wr = r[5], r[6], r[7]
        self.imr_a, self.imr_b = r[13], r[14]
        self.if_active = r[12] > 0.5
        self.state = STATE_MAP[state]
        if fault and self.fault is None:
            self.fault = FAULT_MAP[fault]

        if self.state == S.ST_FAULT or not run_request:
            self.vd_int = self.vq_int = 0.0
            self.va_next = self.vb_next = 0.0
            self.theta = r[8]
            self.id = self.iq = 0.0
            return 0.0, 0.0

        s, c = math.sin(phase), math.cos(phase)
        i_d = c * ia + s * ib
        i_q = c * ib - s * ia
        ed = id_ref - i_d
        eq = iq_ref - i_q
        self.vd_int += ed * vc.ki * dt
        self.vq_int += eq * vc.ki * dt
        vd = self.vd_int + ed * vc.kp
        vq = self.vq_int + eq * vc.kp
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

        self.theta = phase
        self.id, self.iq = i_d, i_q
        self.id_ref, self.iq_ref = id_ref, iq_ref
        self.slip = r[11]
        return va, vb


PY_CTL = S.AcimController


def run(use_c, fn):
    S.AcimController = CCoreController if use_c else PY_CTL
    try:
        return fn()
    finally:
        S.AcimController = PY_CTL


def case_a(f):
    def fn():
        mp = S.MotorParams(i_sat=130.0)
        vc = S.VescConf(mp)
        ac = S.AcimConf(tau_r=mp.tau_r * f)
        log, ctl, _ = S.simulate(mp, vc, ac, 2.4, R.iq_cycle, load=R.fan())
        out = []
        for t0, t1 in ((0.6, 0.8), (1.2, 1.4), (1.8, 2.0)):
            out.append("%.2f Nm / %.1f deg" % (S.steady(log, "te", t0, t1), S.steady(log, "ang_err", t0, t1)))
        return ", ".join(out) + ", fault %s" % (ctl.fault or "none")
    return fn


def case_c():
    mp = S.MotorParams()
    vc = S.VescConf(mp)
    ac = S.AcimConf()
    log, ctl, _ = S.simulate(mp, vc, ac, 0.35, lambda t: 150.0, load=R.fan(), log_every=2)
    t_run = next(t for t, st in zip(log["t"], log["state"]) if st == S.ST_RUN)
    return "RUN after %.0f ms, fault %s" % (t_run * 1e3, ctl.fault or "none")


def case_d(f, rs):
    def fn():
        mp = S.MotorParams(i_sat=130.0)
        vc = S.VescConf(mp, foc_motor_r=mp.Rs * rs)
        ac = S.AcimConf(speed_src=S.SRC_SENSORLESS, tau_r=mp.tau_r * f)
        log, ctl, _ = S.simulate(mp, vc, ac, 2.6, R.iq_sl, load=R.load_sl)
        closed = [i for i, t in enumerate(log["t"]) if 0.8 <= t <= 1.6 and not log["if_active"][i]]
        ang = R.rms([log["ang_err"][i] for i in closed])
        spd = R.rms([log["wr_est"][i] - log["wr_true"][i] for i in closed])
        end_rpm = log["rpm"][-1]
        return "%.2f Nm, angle rms %.1f deg, speed err rms %.2f Hz, end %.0f rpm, fault %s" % (
            S.steady(log, "te", 1.3, 1.6), ang, spd, end_rpm, ctl.fault or "none")
    return fn


def case_e(name, kw_vc, kw_sim):
    def fn():
        mp = S.MotorParams()
        vc = S.VescConf(mp, **kw_vc)
        ac = S.AcimConf()
        log, ctl, _ = S.simulate(mp, vc, ac, 1.5, lambda t: 150.0 if t > 0.2 else 0.0, load=R.fan(), **kw_sim)
        ft = R.first_fault_t(log)
        return "fault %s at %s, max %.0f rpm" % (ctl.fault or "none", "%.3f s" % ft if ft else "-", max(log["rpm"]))
    return fn


def case_enc_lost(use_c):
    mp = S.MotorParams()
    vc = S.VescConf(mp)
    ac = S.AcimConf()
    plant = S.Plant(mp, load=R.fan())
    ctl = (CCoreController if use_c else PY_CTL)(vc, ac)
    dt = ctl.dt
    va = vb = 0.0
    fault_t = None
    stuck = None
    for k in range(int(1.6 / dt)):
        t = k * dt
        isa, isb, _, _ = plant.currents()
        cnt = math.floor(plant.x[5] / S.TWO_PI * vc.enc_counts)
        if t > 1.0:
            stuck = cnt if stuck is None else stuck
            cnt = stuck
        th = S.norm_angle(cnt / vc.enc_counts * S.TWO_PI * vc.pp)
        vn = ctl.update(isa, isb, th, True, 150.0 if t > 0.2 else 0.0)
        plant.step(va, vb, dt, 4)
        va, vb = vn
        if ctl.state == S.ST_FAULT and fault_t is None:
            fault_t = t
    return "fault %s at %s" % (ctl.fault or "none", "%.3f s" % fault_t if fault_t else "-")



def case_ilimit():
    """Motor current limit below Id_mag (bench, 2026-10-09): with a 40 A limit and Id_mag 75 A
    the motor never left FLUXING and made no torque. C core only (the Python controller has no
    current limit)."""
    mp = S.MotorParams()
    vc = S.VescConf(mp)
    ac = S.AcimConf()
    ac.i_avail = 0.5 * ac.id_mag
    log, ctl, _ = S.simulate(mp, vc, ac, 1.5, lambda t: 40.0 if t > 0.2 else 0.0, load=R.fan(), log_every=2)
    t_run = next((t for t, st in zip(log["t"], log["state"]) if st == S.ST_RUN), None)
    return "limit %.0f A (Id_mag %.0f A): %s, %.2f Nm at 1.0 to 1.5 s, max %.0f rpm, fault %s" % (
        ac.i_avail, ac.id_mag, "RUN after %.0f ms" % (t_run * 1e3) if t_run else "never left FLUXING",
        S.steady(log, "te", 1.0, 1.5), max(log["rpm"]), ctl.fault or "none")


def main():
    global LIBC
    LIBC = build()

    cases = [
        ("A encoder, sat iron, tau_r 70%", case_a(0.7)),
        ("A encoder, sat iron, tau_r 100%", case_a(1.0)),
        ("A encoder, sat iron, tau_r 130%", case_a(1.3)),
        ("C flux build-up", case_c),
        ("D sensorless nominal", case_d(1.0, 1.0)),
        ("D sensorless tau_r -30%", case_d(0.7, 1.0)),
        ("D sensorless Rs -30%", case_d(1.0, 0.7)),
        ("D sensorless Rs +30%", case_d(1.0, 1.3)),
        ("E encoder wired backwards", case_e("rev", {}, dict(enc_dir_true=-1))),
        ("E pole pairs 4 (true 6)", case_e("pp4", dict(pp=4), {})),
        ("E pole pairs 8 (true 6)", case_e("pp8", dict(pp=8), {})),
    ]

    lines = ["| Case | Python controller (Phase 3) | C core (motor/acim_core.c) |", "|---|---|---|"]
    for name, fn in cases:
        py = run(False, fn)
        cc = run(True, fn)
        lines.append("| %s | %s | %s |" % (name, py, cc))
        print(lines[-1], flush=True)
    lines.append("| E encoder signal lost at 1.0 s | %s | %s |" % (case_enc_lost(False), case_enc_lost(True)))
    lines.append("| F current limit below Id_mag | - | %s |" % run(True, case_ilimit))
    print(lines[-1])
    lines.append("")
    lines.append("Case A columns: torque / flux angle error at the 150 A, 250 A and -150 A steps.")

    with open(os.path.join(OUT, "c_core_check.md"), "w") as fh:
        fh.write("\n".join(lines) + "\n")


if __name__ == "__main__":
    main()
