# Packaged evidence

| Directory | Contents |
|---|---|
| `raw/bifilar/` | Seven completed original mocap captures, configs, receipts and retrospective measurement updates |
| `processed/inertia/` | Selected-window audit, period/pose projection, adopted calculation, oscillator/forecast parameters and sensitivity records |
| `processed/actuator/` | Audited 727 plateaus, 322 stationary static points, 936 accepted normalised signal transitions, validation scores and 305 vibration summaries/PSDs |
| `processed/geometry/` | Approximate compact body-frame CAD preview, with source hash and units |
| `calibration/` | Inherited bidirectional two-channel static torque calibration |
| `provenance/` | Source-file mapping, source CAD inventory, software versions and packaged-file SHA-256 manifest |

`static_enriched.csv` is the 322-point final static dataset; `plateaus.csv` retains other campaign phases. `step_curves.npz` rows align exactly with `step_metrics.csv`; its `t` array is in seconds from the FC command edge. `vibration_psd.npz` rows align with `vibration.csv`. The processed values already include the recorded calibration, invalid-frame handling, qualified clock registration and RPM/gyro conversions.

Original raw poses use XYZW quaternions and metre positions, with integer source sec/nanosec and host timestamps. Configs may contain original host paths or DDS settings as historical metadata; reproduction tools resolve only packaged relative paths. They do not contact those hosts.

The zero-sample initial yaw attempt is not packaged as a measurement. Its failure is retained in the historical collection review. All seven completed trials are included, including larger-amplitude and pose-discontinuity examples. The default inertia analysis refits the three selected windows, while the collection review retains the broader selection/repeatability evidence.

`import_sources.json` records original files before path adaptations. `manifest.json` hashes the actual packaged data, parameters and media. Original motor raw streams, full Blackbox logs, USDZ/F3Z archives and PDFs remain in the external sibling archives `Raven/actuator_modelling` and `Raven/inertia_measurements`; they are not required for the documented offline build.

The CAD preview is not a collision model or a measured mass distribution. Licensing/attribution status is recorded in [LICENSES.md](../LICENSES.md).
