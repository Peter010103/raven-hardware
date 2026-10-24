# Initial Raven model and flip MPC simulation

This is an **offline candidate**, starting from hover. It combines Raven's measured rotor models and adopted 716 g inertial configuration with an explicitly assumed rate controller. The operator reports that **Betaflight is not currently flashed/configured on the intended Raven FC**. Historical bench dumps do not define its future flight settings.

The native MuJoCo plant and full-attitude MPC run locally with direct simulated transport. No flight, flashing, arming, network control or policy training is performed. Candidate robot descriptions are checked in at [models/raven.urdf](../models/raven.urdf) and [models/raven.xml](../models/raven.xml), following the Sanity hardware layout. Controller configuration is in [parameters/mpc_simulation.json](../parameters/mpc_simulation.json), with provenance in [data/provenance/simulation_model.json](../data/provenance/simulation_model.json). Prediction products and run evidence are written into ignored `build/`; canonical measured parameter files are read as inputs.

## Measured inputs and engineering assumptions

| Component | Source / treatment |
|---|---|
| Mass | 0.716 kg total suspended configuration |
| COM inertia | diag(0.0020581804586, 0.0017673633156, 0.0031496362931) kg m² |
| COM | Operator-adopted approximate CAD FC reference minus 25 mm body Z |
| Direct rotor plant | Existing `physics` model: degree-4 command/voltage speed surface, full quadratic thrust, square resisting-torque law |
| Rotor dynamics | Identified running speed pole 21.8914 ms and fitted FC-output-edge delay 1.76346 ms |
| Voltage | Explicit, constant delivered bus-voltage scenario; no battery-sag model |
| Rotor layout | Approximate supplied CAD: X ±68.7435 mm, Y ±89.69 mm relative to marker reference; unmeasured parallel thrust-axis height has no axial-force lever contribution |
| Replication | One tested ACW motor/prop replicated four times; unity installation/collective correction and opposite-direction transfer assumed |
| Rate loop | Engineering PI surrogate, fixed nominal-hover command slopes; **no Raven closed-loop rate identification** |
| Feedback | Ideal native COM position, orientation, velocity, body rate and actual rotor-force sum |
| Other dynamics | No aerodynamic drag, dynamic inflow, independently calibrated rotor gyro/reaction-inertia term, or flight observation noise/filtering |

The floating-base origin is the adopted COM, with body **X forward, Y left, Z up** and quaternion **WXYZ**. The generated URDF also provides the original marker-centroid reference as a fixed child frame. A future mocap adapter must transform marker-origin pose/velocity to COM; the tracking pivot is not automatically the COM.

## Proposed firmware and collective conversion

[proposed_profile.json](proposed_profile.json) is a design record, not an installed configuration or complete flash/CLI file. The reference is Betaflight **4.5.5**, pinned to commit `4adbd3ef7cb546947600e5f747bd5453c9573063` in [firmware_reference.json](firmware_reference.json).

The initial design uses Acro, DShot600/bidirectional telemetry, 14 motor poles, fixed 5.5% digital idle, linear throttle, no throttle boost, no thrust linearisation, no dynamic idle and no RPM limiter. ACTUAL centre/max rates are both 600°/s, producing a linear rate map. The MPC pitch limit is 9.5 rad/s, below that full scale. Initial timing tests assume RC smoothing off; installed transport/filter timing needs its own model later. Exact ESC firmware, FC mounting and motor order/signs remain to be verified on hardware.

The nominal primary scenario is **14.8 V / 4S / compensation 0**. A separate scenario sets compensation to 100. Cell count is explicit: an instantaneous bus voltage alone cannot distinguish 3S from a depleted 4S pack.

For integer RC throttle $r$, zero throttle expo and `min_check=1050`:

$$
t=\frac{\lfloor1000[\operatorname{clip}(r,1050,2000)-1050]/950\rfloor}{1000}.
$$

DShot low is $D_l=48+1999(0.055)$. With full motor-output limit, uncompensated high is $D_h=2047$. The settled compensation model reduces the high endpoint:

$$
b=1-\operatorname{clip}((420-V_{cell,cV})/70,0,1),\quad
A=(70/420)b(C/100),\quad D_h'=D_h-A(D_h-D_l).
$$

The source uses the fixed **420 cV** full-cell constant and the configured **350 cV** warning threshold. The sag-filtered total voltage is divided by explicit cell count using the firmware's integer cell-voltage convention. This is not a nominal/actual-voltage throttle multiplier. In the settled scenarios, sag-filtered and measured bus voltages represent the same constant voltage.

The direct-test equivalent coordinate and nominal force are

$$
D=D_l+t(D_h'-D_l),\quad c=1001+999(D-48)/1999,\quad
\widehat F(r,V)=4f_{physics}(c,V).
$$

These are continuous internal motor outputs expressed in external-test coordinates; final DShot packet quantisation is below approximately 0.25 external count. The supplied inverse returns **integer RC counts** and rejects targets outside the mapped joint command/voltage support. Its force round-trip error is bounded by the adjacent RC-count increments; that is separate from the measured model's identification error. No second voltage compensation is added by the MPC.

At 14.8 V, supported predicted collective is about **0.216–36.540 N**; the MPC upper request bound is 95% of this maximum. At a fixed delivered **16.8 V**, observed joint coverage supports only about **7.419 N**: high-load samples experienced lower bus voltage. This coverage boundary is not a physical maximum-thrust claim. Missing regions remain unsupported.

`validate.py` hashes the pinned sources, compiles the actual endpoint/external-conversion and ACTUAL-rate functions into a small C reference, and checks the restricted steady composition, inverse, hover trim, native inertia and torque-to-acceleration response. It does not validate a complete firmware binary or PID loop.

## Physical plant and MPC

The plant has a free rigid body and four independent rotor-speed states:

$$
\dot\Omega_i=(\Omega_{i,\infty}(c_i,V)-\Omega_i)/\tau_\Omega,
\qquad F=\sum_i T_i(\Omega_i),
$$

$$
m\dot v=-mg e_z+R e_zF,\quad
J\dot\omega=\sum_i[r_i\times(e_zT_i)+\sigma_iQ_i e_z]-\omega\times(J\omega).
$$

The identified running delay precedes the speed states. Source/model hulls constrain each rotor command, and each run records any attempted support clipping. Hover is trimmed to zero moment about the adopted COM instead of assuming equal rotor commands automatically balance it.

The rate-loop surrogate requests moments using $K_p(\omega_c-\omega)+I_{PI}$ and a fixed nominal-hover linear command mixer. Nominal gain scales use $K_p=\mathrm{diag}(J)/[0.04,0.04,0.08]$; these numbers are gain-design priors, not measured closed-loop time constants. `--rate-scale 2` halves both proportional and integral gains. Gain/mixer calibration stays nominal during force/inertia perturbations. This is not a complete stock Betaflight PID implementation.

The MPC reuses the Sanity **software architecture and run-in reference geometry**, with Raven mass/inertia, thrust/torque limits, allocation and force scale. It predicts a simpler 14-state force/rate servo, uses a 0.8 s horizon (20 × 40 ms), and selects commands every 20 ms. Measured Sanity flight-response coefficients are not loaded. Speed-based plant lag is applied once; the HX711/rig two-pole observation response is not inserted into the aircraft plant.

The reference has a level run-in, C4 pull-up, vertical pitch loop, exit and recovery: approximately 0.477 m horizontal radius, 0.62 m vertical radius, 0.5 m bottom and 2.4 m/s initial run-in speed. The run starts already hovering; startup/takeoff is outside the measured running-model identification.

## Reproduce

Use an environment containing the main requirements, [simulation requirements](requirements.txt), and a built **acados v0.6.0** with its Python template interface and C toolchain. See the upstream [acados installation guide](https://docs.acados.org/installation/index.html). The existing workspace `.venv-sanity-train` already supplies these dependencies.

From the hardware repository root, using that interpreter:

```sh
# One-time source fetch from pinned GitHub URLs; computation is offline afterwards.
python simulation/fetch_reference.py
python simulation/validate.py
python tools/build_models.py

OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 python simulation/screen.py --output build/NEW_SCREEN
python simulation/assess.py --suite build/NEW_SCREEN
python simulation/report.py --suite build/NEW_SCREEN
MUJOCO_GL=egl python simulation/render.py --run build/NEW_SCREEN/nominal
```

For one run or a different explicitly labelled scenario:

```sh
OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 python simulation/run.py --output build/ONE_FLIP --voltage 14.8 --cells 4
# Independent examples, each requiring a fresh output path:
python simulation/run.py --output build/THREE_FLIPS --laps 3
python simulation/run.py --output build/COMP100 --compensation 100
python simulation/run.py --output build/SLOW_RATE --rate-scale 2
```

Every run saves the native XML, controller config, reference, source snapshot/hashes, full command/rotor/state/solver trace and assessment. Default evaluation uses ideal state feedback. Delay cases use **known queues compensated by prediction**, rather than claiming tolerance to arbitrary unmodelled network delay. Video generation needs FFmpeg and an available EGL/OpenGL renderer. `tools/build_models.py` also exports `build/raven_model/raven.urdf`, `raven.xml`, mapping predictions and a model provenance record; the assets require the Python actuator/rate adapter for powered simulation.

The initial screen passes 13/14 cases: nominal and reverse flips, three continuous flips, compensation enabled, rate-gain scale 1.5, motor-lag scale 1.5, known 20/40 ms queues, −10% force, +20% inertia, and 12.6 V/3S and 16 V/4S scenarios. Halved rate-feedback gains fail final attitude recovery. Nominal whole-mission position RMS is approximately **5.93 mm**, or **9.53 mm over the moving interval**. These are conditional simulation results with ideal feedback, not flight tracking forecasts.

## Measurements needed next

1. Confirm installed motors/props, actual rotor geometry, motor order/spin signs and FC-to-body alignment; check the untested CW propeller direction.
2. Record installed/armed collective thrust and individual motor outputs at several supported voltages, including hover and manoeuvre headroom. This tests the nominal four-times-one-rotor transfer and inverse.
3. Identify the aircraft's small-signal roll/pitch/yaw rate response, then extend the envelope toward flip rates. Retain input/output/gyro timestamps, filtering, saturation and bus voltage.
4. Validate the actual observation/transport path and high-rate moving-air performance. The current ideal COM/rate/force feedback and known-delay tests do not establish those behaviours.

These gaps are explicit model fields. Initial simulation can proceed now; flight-ready MPC validation depends particularly on the rate-loop and installed collective checks.
