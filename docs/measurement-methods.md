# Measurement methods and interpretation

## Single-motor actuator campaign

- **Plant:** one EMAX ECO II 2207 1900 KV, 12N14P motor and HQ 5×4.8×3 V1S ACW propeller. The reversed propeller was planned, but no CW identification is included.
- **Command/telemetry:** Matek H743 running Betaflight; direct disarmed MSP Motor 4 testing, DShot600 and bidirectional telemetry. MSP counts are not RC throttle or physical microsecond pulse measurements.
- **Force acquisition:** the existing three-HX711 Orin rig, one thrust channel and two opposing torque channels. The shared-clock acquisition retains invalid-frame records. The selected models use qualified records and bracketed tare corrections.
- **Power:** regulated Tenma 72-13350 via rear high-current outputs, eight source settings from 10.5 to 16.8 V. High-load coverage is limited by CV support, current/power capability and delivered bus voltage. Missing high-voltage/full-command points are not filled in by assumption.
- **Calibration:** inherited thrust reference up to 1 kg / 9.80665 N, and bidirectional two-channel deadweight torque calibration up to ±0.024516625 N m. Large Raven torques extrapolate this gain; no new large-moment calibration was performed.
- **Campaign:** startup/envelope, repeated static plateaus, operating-point steps and independent command sequences. 727 audited plateaus across phases; 322 stationary static plateaus define the final static identification dataset.

Blackbox provides high-rate FC gyro, command, voltage and motor telemetry. Original Blackbox logs and decoded frame arrays remain in the external archive. Processed summaries and selected transition curves are packaged here. Command edges register the FC and host clocks using an affine fit. Its residual includes jitter, and the fixed host-to-FC latency cannot be separated by that method. Motor-output-edge → RPM timing stays on the FC clock.

The primary model uses **MSP display-filtered voltage**. Blackbox's raw ADC-derived voltage has different filtering and can contain rapid excursions. Those excursions have not been verified with an oscilloscope; this repository does not promote them to calibrated power-supply transients or regeneration measurements.

The direct bench campaign did not measure an armed FC rate loop, vbat compensation behaviour, CTBR collective control or free-flight response. Reaction torque is a resisting magnitude, so body-axis/spin signs must be established separately. Thrust is positive along the rig's calibrated direction; body-force allocation requires the physical motor mounting convention.

## Bifilar measurements

The operator reported 716 g for the suspended Raven configuration. Two strings were used, with different lower pairs for each intended body axis. Capture labels alone do not verify alignment: recorded full quaternions, the nominal 180° body-X marker rotation and per-mounting contact points are used in the inertia solve. The reference COM is the operator-adopted approximate CAD FC position minus 25 mm body Z.

The final static geometry is in [fixture_geometry.json](../parameters/fixture_geometry.json). Geometry and mass supplied after collection are retained as retrospective metadata; original configs/poses remain byte-for-byte copies. The source CAD's Y-up millimetre convention is converted only for the preview geometry. Captured quaternions are not subjected to an additional guessed coordinate swap.

See [inertia.md](inertia.md) for periods, offsets, inferred contact edges, fitting criteria and the limitations of a diagonal COM tensor.
