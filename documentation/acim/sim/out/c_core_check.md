| Case | Python controller (Phase 3) | C core (motor/acim_core.c) |
|---|---|---|
| A encoder, sat iron, tau_r 70% | 12.40 Nm / 8.0 deg, 12.39 Nm / 8.0 deg, -12.44 Nm / -8.0 deg, fault none | 12.40 Nm / 8.0 deg, 12.39 Nm / 8.0 deg, -12.44 Nm / -8.0 deg, fault none |
| A encoder, sat iron, tau_r 100% | 14.35 Nm / -2.5 deg, 20.48 Nm / -2.3 deg, -14.39 Nm / 2.5 deg, fault none | 14.35 Nm / -2.5 deg, 20.49 Nm / -2.3 deg, -14.39 Nm / 2.5 deg, fault none |
| A encoder, sat iron, tau_r 130% | 13.92 Nm / -11.9 deg, 17.13 Nm / -12.0 deg, -13.94 Nm / 12.0 deg, fault none | 13.93 Nm / -11.9 deg, 17.13 Nm / -12.0 deg, -13.94 Nm / 12.0 deg, fault none |
| C flux build-up | RUN after 126 ms, fault none | RUN after 126 ms, fault none |
| D sensorless nominal | 11.37 Nm, angle rms 2.0 deg, speed err rms 1.68 Hz, end -0 rpm, fault none | 11.37 Nm, angle rms 2.0 deg, speed err rms 1.68 Hz, end -1 rpm, fault none |
| D sensorless tau_r -30% | 11.37 Nm, angle rms 1.9 deg, speed err rms 3.07 Hz, end -0 rpm, fault none | 11.37 Nm, angle rms 2.0 deg, speed err rms 3.08 Hz, end -1 rpm, fault none |
| D sensorless Rs -30% | 11.35 Nm, angle rms 10.3 deg, speed err rms 6.24 Hz, end -5 rpm, fault none | 11.34 Nm, angle rms 10.2 deg, speed err rms 6.17 Hz, end -8 rpm, fault none |
| D sensorless Rs +30% | 11.35 Nm, angle rms 1.8 deg, speed err rms 1.64 Hz, end -1 rpm, fault none | 11.35 Nm, angle rms 1.9 deg, speed err rms 1.64 Hz, end -1 rpm, fault none |
| E encoder wired backwards | fault none at -, max 114 rpm | fault none at -, max 114 rpm |
| E pole pairs 4 (true 6) | fault ACIM_FLUX at 0.365 s, max 323 rpm | fault ACIM_FLUX at 0.365 s, max 323 rpm |
| E pole pairs 8 (true 6) | fault ACIM_FLUX at 0.350 s, max 824 rpm | fault ACIM_FLUX at 0.350 s, max 823 rpm |
| E encoder signal lost at 1.0 s | fault ACIM_SLIP at 1.051 s | fault ACIM_SLIP at 1.051 s |

Case A columns: torque / flux angle error at the 150 A, 250 A and -150 A steps.
