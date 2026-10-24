# Reproduce the curated results

Run from a fresh clone of this repository. Python 3.13 with NumPy 2.2, SciPy 1.16, pandas 2.3 and Matplotlib 3.10 was used for the original actuator analysis; exact versions are retained in [the provenance record](../data/provenance/actuator_software_versions.json).

```sh
python3 -m venv .venv
. .venv/bin/activate
pip install -r requirements.txt
python tools/validate_repo.py
python tools/inertia/analyze.py
python tools/plot_models.py
```

Validation checks packaged hashes, documentation links, held-out actuator predictions, domain rejection, analytic delayed speed response, and the inertia recovered from original selected poses. The default build never edits packaged measurements or parameters. Recomputed outputs are written under ignored `build/`.

`plot_models.py` regenerates the six actuator figures and four inertia figures from packaged datasets and the compact preview mesh. It needs no ROS, hardware, original workspace, CAD SDK, source CAD archive, or proprietary fonts. Avenir is used when installed under the user's font directory; DejaVu Sans is the portable fallback. Curves and text are vector SVG; 3D mesh/surface raster layers and the CAD PNG use 900 dpi.

To explicitly refresh the checked-in media after reviewing a deliberate model/data change:

```sh
python tools/plot_models.py --update-results
python tools/validate_repo.py --refresh-manifest
python tools/validate_repo.py
```

`--refresh-manifest` updates packaged-data hashes, so use it only for intentional, reviewed changes. It is not needed after an ordinary build.

## Static refit

```sh
python tools/refit_static.py
```

This independently refits candidate static surfaces, selects their degree on repeat-3 development data and evaluates the reserved 12/16 V settings. It also refits the simple cascade and square/full-quadratic rotor laws, with group bootstrap intervals. It writes to `build/refit/` and does not replace canonical parameters. Solver/library versions can change last-bit coefficients; compare predictions and validation errors rather than requiring byte-identical JSON.

The retained dynamic parameters and normalised transition curves can be inspected and validated offline. Re-decoding Blackbox, re-registering clocks and reconstructing transitions from original FC/ADC streams require the external motor-campaign archive; those large sources are intentionally not duplicated here. Full original campaign analysis scripts remain alongside that archive.

## Interactive thrust surface

```sh
python tools/view_thrust_surface.py
# A file can be generated without a graphical desktop:
python tools/view_thrust_surface.py --save build/thrust-surface.png
```

The window supports normal Matplotlib 3D rotation and zoom. Unsupported parts of the command–voltage plane are masked. This tool displays direct motor command and delivered voltage, not a speed–voltage or RC-throttle surface.
