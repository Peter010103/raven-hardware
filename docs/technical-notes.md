# Robot description and simulation notes

These are **offline candidate models**, not a flight-validated aircraft description. The intended Raven flight controller is not yet flashed/configured; proposed firmware settings are recorded separately.

| File | Purpose |
|---|---|
| [models/raven.xml](../models/raven.xml) | Native MuJoCo free rigid body, explicit inertia and simplified body/arm/propeller geometry |
| [models/raven.urdf](../models/raven.urdf) | Matching mass/inertia and fixed marker-centroid/rotor frames; no visual meshes or powered actuator dynamics |
| [parameters/mpc_simulation.json](../parameters/mpc_simulation.json) | Raven simulation MPC configuration, including explicitly assumed rate-response parameters |
| [data/provenance/simulation_model.json](../data/provenance/simulation_model.json) | Units, reference frames, geometry assumptions, trim and canonical parameter hashes |

The root is the adopted **centre of mass**, with X forward, Y left and Z up. Mass is **0.716 kg**; roll/pitch/yaw inertia is **diag(0.0020581804586, 0.0017673633156, 0.0031496362931) kg·m²**. These describe the weighed suspended configuration, including attachments not separately subtracted. Rotor XY positions are approximate CAD estimates; rotor Z placement and reaction-torque signs remain assumptions. The URDF rotor order is rear-right, front-right, front-left, rear-left; it is not a verified FC output order.

**Loading the XML or URDF alone does not reproduce the powered simulation.** The measured command/voltage-to-speed model, motor lag/delay, thrust/reaction torque and assumed PI rate controller run in [simulation/plant.py](../simulation/plant.py), which applies the resulting forces and moments to MuJoCo. The XML has no motor actuators; the URDF is an inertial/frame export. See the [simulation guide](../simulation/README.md) and [conditional flip results](../simulation/RESULTS.md).

Regenerate from canonical parameters, using the simulation dependencies, from the repository root:

```sh
python tools/build_models.py
```

The generator writes robot descriptions directly into `models/`, controller configuration into `parameters/`, and model provenance into `data/provenance/`. It also writes mapping prediction plots/data into ignored `build/raven_model/`. Measured parameter files are unchanged. Installed collective thrust, actual rate-controller dynamics, motor order/signs and observation/transport behaviour still need validation before using these results for physical flight.
