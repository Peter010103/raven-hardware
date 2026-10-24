<div align="center">
<h1>Raven</h1>
<p><strong>Measured actuator and inertial models for simulation</strong></p>
<p><a href="docs/actuator-modelling.md">Actuator models</a> · <a href="docs/inertia.md">Mass and inertia</a> · <a href="docs/measurement-methods.md">Measurement methods</a> · <a href="docs/reproduction.md">Reproduce</a> · <a href="data/README.md">Data</a></p>
<img src="media/inertia/cad-markers.png" alt="Approximate Raven CAD, marker layout and estimated COM" width="820">
</div>

This repository curates the Raven bench measurements from **22–24 October 2026**, using the same documentation, parameter, figure and provenance structure as `sanity-hardware`. Two different experiments are represented: **one motor and ACW propeller on a thrust/torque rig**, and **the 716 g suspended Raven configuration** on a bifilar pendulum.

| Result | Working value / meaning |
|---|---|
| Motor / propeller tested | EMAX ECO II 2207 **1900 KV**, 12N14P; HQ **5×4.8×3 V1S ACW** |
| Control path identified | Disarmed direct **MSP Motor 4**, **DShot600**, bidirectional RPM telemetry |
| Recommended running actuator | Command/voltage → speed surface; delayed first-order speed state; speed → thrust/torque |
| Speed time constant / fitted delay | **21.89 ms / 1.76 ms**, referenced to the FC output edge |
| Held-out-voltage thrust RMS | **0.0408 N** recommended cascade; **0.0371 N** independent static surface |
| Total suspended mass | **0.716 kg**, including the recorded attachments |
| Adopted COM | CAD FC reference **minus 25 mm body Z** |
| COM inertia, roll / pitch / yaw | **diag(2.05818, 1.76736, 3.14964) × 10⁻³ kg·m²** |

The actuator is a **single tested rotor**, not an identified four-motor or CTBR flight controller. Positive commands are supported only inside the measured command/voltage hull. The inertia is the operator-adopted working model of the weighed configuration; marker/fixture contributions have not been separately subtracted.

## Selected models and figures

![Measured direct-command/voltage surfaces](media/identification/command-voltage-surfaces.svg)

[The actuator guide](docs/actuator-modelling.md) compares simple, physics-cascade and independent surface variants, documents input units and shows static curves, efficiency, electrical voltage drop and step responses. [The inertia guide](docs/inertia.md) shows the COM, string locations, recorded mounting poses and oscillator fits, with the inferred contact edges explicitly marked.

## Files

| Path | Contents |
|---|---|
| [parameters/](parameters/) | Full-precision actuator variants, recommendation, inertial model and fixture/marker conventions |
| [raven_actuator.py](raven_actuator.py) | NumPy/SciPy runtime with support checks, speed dynamics and optional reaction torque |
| [media/identification/](media/identification/) | Six selected actuator figures; vector curves/text with high-resolution surface rendering |
| [media/inertia/](media/inertia/) | CAD/marker layout, COM/fixture diagram, suspension poses and measured/model overlays |
| [data/](data/README.md) | Compact processed measurements, seven completed original mocap recordings and calibration/provenance |
| [tools/](tools/) | Offline plots, inertia refit, static refit, interactive thrust surface and validation |
| [LICENSES.md](LICENSES.md) | Imported-code attribution and licensing status |

## Use and verify

```sh
python3 -m venv .venv
. .venv/bin/activate
pip install -r requirements.txt
python tools/validate_repo.py
python tools/plot_models.py
python tools/view_thrust_surface.py
```

From the repository root:

```python
from raven_actuator import steady, DynamicMotor

operating_point = steady(1500, 14.0, variant="physics")
print(operating_point)  # one rotor: omega [rad/s], T_N [N], Q_Nm [N m]
motor = DynamicMotor(variant="physics", omega0=float(operating_point["omega"]))
next_state = motor.step(1600, 14.0, dt=0.002)
```

`V` is **delivered FC voltage**, not the PSU setting or nominal battery voltage. `u` is an MSP count, not RC throttle or a measured microsecond PWM pulse. Explicitly select `physics` for the recommended cascade; `steady` retains the original runtime's `surface` default for static predictions. STOP sets zero equilibrium and retains dynamic coastdown.

All commands above are offline. Build outputs go into ignored `build/`. Full motor-campaign dumps, Blackbox logs, source CAD archives, PDFs and virtual environments remain outside this curated repository. No Raven URDF, flight-ready controller or multi-motor allocation has been inferred from these tests.
