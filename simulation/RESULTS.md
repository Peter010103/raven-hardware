# Initial verified screen — 24 October 2026

Native MuJoCo with four measured-model rotor-speed states; ideal COM/rate/force feedback and an assumed PI rate controller. Firmware settings are proposed; current Raven FC is unflashed/unconfigured.

| Case | Result | Moving RMS [cm] | Final attitude [deg] |
|---|---|---:|---:|
| hover | PASS | — | 0.000 |
| nominal | PASS | 0.953 | 0.000 |
| reverse | PASS | 0.895 | 0.000 |
| three_flips | PASS | 0.892 | 0.000 |
| comp100 | PASS | 0.951 | 0.000 |
| rate_1p5 | PASS | 0.949 | 0.000 |
| rate_2 | FAIL | 1.825 | 24.355 |
| motor_1p5 | PASS | 0.961 | 0.009 |
| delay20 | PASS | 1.314 | 0.000 |
| delay40 | PASS | 1.695 | 0.003 |
| force_minus10 | PASS | 1.492 | 0.001 |
| inertia_plus20 | PASS | 0.915 | 0.000 |
| voltage12p6 | PASS | 0.954 | 0.000 |
| voltage16 | PASS | 0.952 | 0.000 |

All 14 traces remain within the observed rotor command/voltage support. Thirteen pass the stated terminal criteria. The rate-scale-2 case halves the surrogate proportional/integral gains and leaves 24.36° final attitude error; its trace is retained. Maximum selected MPC calculation time across the suite is 13.45 ms or less on this host. This is a chosen sensitivity screen, not a statistical robustness certificate.

Nominal: one full inversion, 5.93 mm whole-mission RMS / 9.53 mm moving RMS; height 0.4686–1.7292 m; peak body pitch rate 8.813 rad/s; actual force 5.759–20.798 N; integer RC throttle 1322–1699. Three consecutive flips and the reverse direction also pass.

The 20/40 ms delays are known queues with command-history prediction. No observation noise, filtering or unknown delay was introduced. Rate and collective installation measurements remain the next physical identification tasks.

Evidence (generated/ignored): `build/raven_flip_verified_20261024/`, including `report/raven_initial_simulation_report.pdf`, `nominal/raven_flip.mp4`, per-case source/inputs/XML/config/trace/assessment. [models/raven.urdf](../models/raven.urdf) and [models/raven.xml](../models/raven.xml) are the checked-in candidate robot descriptions; configuration and provenance live in `parameters/` and `data/provenance/` respectively; `build/raven_model/` also contains generated mapping surfaces. [results.json](results.json) retains the compact machine-readable assessment; [README](README.md) gives reproduction commands.
