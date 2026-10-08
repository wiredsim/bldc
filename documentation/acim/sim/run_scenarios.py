#!/usr/bin/env python3
"""Runs the Phase 3 scenarios, writes plots to out/ and a results table to out/results.md.

    python3 run_scenarios.py          # all scenarios (a few minutes)
"""

import math
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import acim_sim as S

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "out")
os.makedirs(OUT, exist_ok=True)

COLORS = {0.7: "#d1495b", 0.85: "#edae49", 1.0: "#2e4057", 1.15: "#66a182", 1.3: "#00798c"}
results = []


def fan(k=2e-4):
    return lambda wm, t: k * wm * abs(wm)


def first_fault_t(log):
    for t, st in zip(log["t"], log["state"]):
        if st == S.ST_FAULT:
            return t
    return None


def rms(xs):
    return math.sqrt(sum(x * x for x in xs) / len(xs)) if xs else float("nan")


# -----------------------------------------------------------------------------------------
# A: encoder drive cycle with tau_r error, linear and saturating iron
# -----------------------------------------------------------------------------------------
def iq_cycle(t):
    if t < 0.2:
        return 0.0
    if t < 0.8:
        return 150.0
    if t < 1.4:
        return 250.0
    if t < 2.0:
        return -150.0
    return 0.0


def scenario_a():
    for sat in (0.0, 130.0):
        mp = S.MotorParams(i_sat=sat)
        tag = "sat" if sat else "linear"
        fig, ax = plt.subplots(5, 1, figsize=(9, 12), sharex=True)
        for f in (0.7, 1.0, 1.3):
            vc = S.VescConf(mp)
            ac = S.AcimConf(tau_r=mp.tau_r * f)
            log, ctl, _ = S.simulate(mp, vc, ac, 2.4, iq_cycle, load=fan())
            c = COLORS[f]
            lab = "tau_r est = %.0f %%" % (f * 100)
            ax[0].plot(log["t"], log["rpm"], color=c, label=lab)
            ax[1].plot(log["t"], log["te"], color=c, label=lab)
            if f == 1.0:
                ax[1].plot(log["t"], log["te_ideal"], "k--", lw=0.8, label="ideal (perfect orientation)")
            ax[2].plot(log["t"], [p * 1e3 for p in log["psr"]], color=c)
            ax[3].plot(log["t"], log["ang_err"], color=c)
            ax[4].plot(log["t"], log["iq"], color=c)
            ax[4].plot(log["t"], log["id"], color=c, ls=":")
            # steady-state numbers in each phase of the cycle
            for name, t0, t1 in (("150A", 0.6, 0.8), ("250A", 1.2, 1.4), ("-150A", 1.8, 2.0)):
                te = S.steady(log, "te", t0, t1)
                ideal = S.steady(log, "te_ideal", t0, t1)
                results.append(dict(scn="A encoder cycle (%s)" % tag, case="tau_r %.0f%%, %s" % (f * 100, name),
                                    te=te, te_ideal=ideal,
                                    psr=S.steady(log, "psr", t0, t1) / (mp.Lm * ac.id_mag),
                                    ang=S.steady(log, "ang_err", t0, t1),
                                    imax=max(log["i_abs"]), fault=ctl.fault))
        ax[0].set_ylabel("rpm (mech)")
        ax[1].set_ylabel("torque [Nm]")
        ax[2].set_ylabel("|psi_r| true [mWb]")
        ax[2].axhline(mp.Lm * 100.0 * 1e3, color="k", lw=0.6, ls="--")
        ax[3].set_ylabel("flux angle err [deg]")
        ax[4].set_ylabel("iq (solid), id (dotted) [A]")
        ax[4].set_xlabel("time [s]")
        ax[0].legend(loc="upper left", fontsize=8)
        ax[1].legend(loc="lower left", fontsize=8)
        fig.suptitle("A: encoder IFOC drive cycle, %s iron. Iq request 0 -> 150 -> 250 -> -150 -> 0 A, fan load"
                     % tag, fontsize=10)
        for a in ax:
            a.grid(alpha=0.3)
        fig.tight_layout()
        fig.savefig(os.path.join(OUT, "A_encoder_cycle_%s.png" % tag), dpi=110)
        plt.close(fig)


# -----------------------------------------------------------------------------------------
# B: tau_r sweep at fixed speed (dyno), linear and saturating iron
# -----------------------------------------------------------------------------------------
def scenario_b():
    facs = [0.7, 0.75, 0.8, 0.85, 0.9, 0.95, 1.0, 1.05, 1.1, 1.15, 1.2, 1.25, 1.3]
    fig, ax = plt.subplots(1, 3, figsize=(13, 4))
    for sat, ls in ((0.0, "-"), (130.0, "--")):
        mp = S.MotorParams(i_sat=sat)
        for rpm, col in ((300, "#2e4057"), (1500, "#66a182"), (3000, "#d1495b")):
            ratios, fluxes, angs = [], [], []
            nominal = None
            rows = []
            for f in facs:
                vc = S.VescConf(mp)
                ac = S.AcimConf(tau_r=mp.tau_r * f)
                log, ctl, _ = S.simulate(mp, vc, ac, 0.9, lambda t: 150.0 if t > 0.3 else 0.0,
                                         fixed_speed_rpm=rpm)
                te = S.steady(log, "te", 0.7, 0.9)
                rows.append((f, te, S.steady(log, "psr", 0.7, 0.9) / (mp.Lm * 100.0),
                             S.steady(log, "ang_err", 0.7, 0.9), ctl.fault,
                             S.steady(log, "vmag", 0.7, 0.9)))
                if f == 1.0:
                    nominal = te
            for f, te, fl, ang, fault, vm in rows:
                ratios.append(te / nominal)
                fluxes.append(fl)
                angs.append(ang)
                if f in (0.7, 0.85, 1.0, 1.15, 1.3):
                    results.append(dict(scn="B tau_r sweep %s, %d rpm, Iq 150 A" % ("sat" if sat else "linear", rpm),
                                        case="tau_r %.0f%%" % (f * 100), te=te, te_ideal=nominal,
                                        psr=fl, ang=ang, imax=float("nan"), fault=fault))
            lab = "%d rpm, %s" % (rpm, "saturating" if sat else "linear")
            ax[0].plot([f * 100 for f in facs], ratios, ls, color=col, marker="o", ms=3, label=lab)
            ax[1].plot([f * 100 for f in facs], fluxes, ls, color=col, marker="o", ms=3)
            ax[2].plot([f * 100 for f in facs], angs, ls, color=col, marker="o", ms=3)
    ax[0].set_ylabel("torque / torque at correct tau_r")
    ax[1].set_ylabel("|psi_r| / (Lm * Id_mag)")
    ax[2].set_ylabel("flux angle error [deg]")
    for a in ax:
        a.set_xlabel("controller tau_r [% of true]")
        a.grid(alpha=0.3)
        a.axvline(100, color="k", lw=0.6)
    ax[0].legend(fontsize=7)
    fig.suptitle("B: steady state vs tau_r error. Id 100 A, Iq 150 A, speed held by a dyno", fontsize=10)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "B_tau_r_sweep.png"), dpi=110)
    plt.close(fig)


# -----------------------------------------------------------------------------------------
# C: flux build-up detail
# -----------------------------------------------------------------------------------------
def scenario_c():
    mp = S.MotorParams()
    vc = S.VescConf(mp)
    ac = S.AcimConf()
    log, ctl, _ = S.simulate(mp, vc, ac, 0.35, lambda t: 150.0, load=fan(), log_every=2)
    fig, ax = plt.subplots(3, 1, figsize=(9, 7), sharex=True)
    ax[0].plot(log["t"], log["imr"], label="estimated i_mr [A]")
    ax[0].plot(log["t"], [p / mp.Lm for p in log["psr"]], "--", label="true |psi_r| / Lm [A]")
    ax[0].axhline(90, color="k", lw=0.6, ls=":")
    ax[0].legend(fontsize=8)
    ax[1].plot(log["t"], log["id"], label="id")
    ax[1].plot(log["t"], log["iq"], label="iq (request is 150 A from t=0)")
    ax[1].legend(fontsize=8)
    ax[2].step(log["t"], log["state"], where="post")
    ax[2].set_yticks([0, 1, 2])
    ax[2].set_yticklabels(["OFF", "FLUXING", "RUN"])
    ax[2].set_xlabel("time [s]")
    for a in ax:
        a.grid(alpha=0.3)
    fig.suptitle("C: flux build-up. Iq is held at 0 until i_mr >= 90 %% of Id_mag and t >= 3*tau_r (%.0f ms)"
                 % (3e3 * ac.tau_r), fontsize=10)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "C_flux_buildup.png"), dpi=110)
    plt.close(fig)
    t_run = next(t for t, st in zip(log["t"], log["state"]) if st == S.ST_RUN)
    results.append(dict(scn="C flux build-up", case="time to RUN %.0f ms" % (t_run * 1e3),
                        te=float("nan"), te_ideal=float("nan"), psr=float("nan"), ang=float("nan"),
                        imax=max(log["i_abs"]), fault=ctl.fault))


# -----------------------------------------------------------------------------------------
# D: sensorless start, run, load step, decel back into I/f
# -----------------------------------------------------------------------------------------
def iq_sl(t):
    if t < 0.2:
        return 0.0
    if t < 1.6:
        return 120.0
    if t < 2.4:
        return -60.0
    return 0.0


def load_sl(wm, t):
    return 2e-4 * wm * abs(wm) + (6.0 if 1.0 <= t < 1.6 else 0.0) * (1.0 if wm >= 0 else -1.0)


def scenario_d():
    cases = [("nominal", 1.0, 1.0), ("tau_r -30%", 0.7, 1.0), ("tau_r +30%", 1.3, 1.0),
             ("Rs -30%", 1.0, 0.7), ("Rs +30%", 1.0, 1.3)]
    cols = ["#2e4057", "#d1495b", "#00798c", "#edae49", "#66a182"]
    fig, ax = plt.subplots(5, 1, figsize=(9, 12), sharex=True)
    mp = S.MotorParams(i_sat=130.0)
    for (name, f, rs), col in zip(cases, cols):
        vc = S.VescConf(mp, foc_motor_r=mp.Rs * rs)
        ac = S.AcimConf(speed_src=S.SRC_SENSORLESS, tau_r=mp.tau_r * f)
        log, ctl, _ = S.simulate(mp, vc, ac, 2.6, iq_sl, load=load_sl)
        ax[0].plot(log["t"], log["rpm"], color=col, label=name)
        ax[1].plot(log["t"], [a - b for a, b in zip(log["wr_est"], log["wr_true"])], color=col)
        ax[2].plot(log["t"], log["ang_err"], color=col)
        ax[3].plot(log["t"], log["te"], color=col)
        ax[4].plot(log["t"], log["if_active"], color=col)
        closed = [i for i, t in enumerate(log["t"]) if 0.8 <= t <= 1.6 and not log["if_active"][i]]
        results.append(dict(scn="D sensorless (saturating iron)", case=name,
                            te=S.steady(log, "te", 1.3, 1.6), te_ideal=S.steady(log, "te_ideal", 1.3, 1.6),
                            psr=S.steady(log, "psr", 1.3, 1.6) / (mp.Lm * ac.id_mag),
                            ang=rms([log["ang_err"][i] for i in closed]),
                            imax=max(log["i_abs"]), fault=ctl.fault,
                            extra="speed est err rms %.2f Hz" % rms([log["wr_est"][i] - log["wr_true"][i]
                                                                     for i in closed])))
    ax[0].set_ylabel("rpm (mech)")
    ax[0].legend(fontsize=8)
    ax[1].set_ylabel("speed est error [Hz elec]")
    ax[1].set_ylim(-10, 10)
    ax[2].set_ylabel("flux angle err [deg]")
    ax[2].set_ylim(-60, 60)
    ax[3].set_ylabel("torque [Nm]")
    ax[4].set_ylabel("I/f active")
    ax[4].set_xlabel("time [s]")
    for a in ax:
        a.grid(alpha=0.3)
    fig.suptitle("D: sensorless. Flux build, I/f start, observer above 12 Hz (handover), 6 Nm load step at 1.0-1.6 s,\n"
                 "braking at -60 A back into the I/f region below 6 Hz, then zero request", fontsize=10)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "D_sensorless.png"), dpi=110)
    plt.close(fig)


# -----------------------------------------------------------------------------------------
# E: fault handling
# -----------------------------------------------------------------------------------------
def scenario_e():
    mp = S.MotorParams()
    cases = []
    cases.append(("encoder wired backwards", S.VescConf(mp), S.AcimConf(), dict(enc_dir_true=-1)))
    cases.append(("pole pairs set to 4 (true 6)", S.VescConf(mp, pp=4), S.AcimConf(), {}))
    cases.append(("pole pairs set to 8 (true 6)", S.VescConf(mp, pp=8), S.AcimConf(), {}))
    for name, vc, ac, kw in cases:
        log, ctl, _ = S.simulate(mp, vc, ac, 1.5, lambda t: 150.0 if t > 0.2 else 0.0, load=fan(), **kw)
        results.append(dict(scn="E fault handling", case=name, te=float("nan"), te_ideal=float("nan"),
                            psr=float("nan"), ang=float("nan"), imax=max(log["i_abs"]), fault=ctl.fault,
                            extra="fault at %s s, max %.0f rpm" % (
                                ("%.3f" % first_fault_t(log)) if first_fault_t(log) else "-", max(log["rpm"]))))

    # encoder signal lost (stuck count) at full speed
    class Stuck(S.Plant):
        pass
    vc = S.VescConf(mp)
    ac = S.AcimConf()
    plant = S.Plant(mp, load=fan())
    ctl = S.AcimController(vc, ac)
    dt = ctl.dt
    va = vb = 0.0
    fault_t = None
    stuck_cnt = None
    for k in range(int(1.6 / dt)):
        t = k * dt
        isa, isb, _, _ = plant.currents()
        cnt = math.floor(plant.x[5] / S.TWO_PI * vc.enc_counts)
        if t > 1.0:
            stuck_cnt = cnt if stuck_cnt is None else stuck_cnt
            cnt = stuck_cnt
        th = S.norm_angle(cnt / vc.enc_counts * S.TWO_PI * vc.pp)
        vn = ctl.update(isa, isb, th, True, 150.0 if t > 0.2 else 0.0)
        plant.step(va, vb, dt, 4)
        va, vb = vn
        if ctl.state == S.ST_FAULT and fault_t is None:
            fault_t = t
    results.append(dict(scn="E fault handling", case="encoder signal lost at 1.0 s",
                        te=float("nan"), te_ideal=float("nan"), psr=float("nan"), ang=float("nan"),
                        imax=float("nan"), fault=ctl.fault,
                        extra="fault at %s s" % ("%.3f" % fault_t if fault_t else "-")))


def write_table():
    lines = ["| Scenario | Case | Torque [Nm] | Ideal [Nm] | Ratio | Flux / target | Angle err [deg] | max |I| [A] | Fault | Notes |",
             "|---|---|---|---|---|---|---|---|---|---|"]

    def fmt(v, p=2):
        return "-" if v is None or (isinstance(v, float) and math.isnan(v)) else ("%.*f" % (p, v))
    for r in results:
        ratio = r["te"] / r["te_ideal"] if r["te_ideal"] and not math.isnan(r["te_ideal"]) and r["te_ideal"] != 0 else float("nan")
        lines.append("| %s | %s | %s | %s | %s | %s | %s | %s | %s | %s |" % (
            r["scn"], r["case"], fmt(r["te"]), fmt(r["te_ideal"]), fmt(ratio, 3), fmt(r["psr"], 3),
            fmt(r["ang"], 1), fmt(r["imax"], 0), r["fault"] or "none", r.get("extra", "")))
    with open(os.path.join(OUT, "results.md"), "w") as fh:
        fh.write("\n".join(lines) + "\n")
    print("\n".join(lines))


if __name__ == "__main__":
    which = sys.argv[1:] or ["a", "b", "c", "d", "e"]
    for w in which:
        print("running", w, flush=True)
        globals()["scenario_" + w]()
    write_table()
