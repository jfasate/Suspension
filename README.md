# Full-car active suspension control

Working seven-mechanical-DOF simulation: coupled body heave/pitch/roll and four
vertical wheels, with four actuator-force states. The representative car uses
the MIT [Andac Y. Kaya / andorianfm full-car model](https://github.com/andorianfm/7dof-active-suspension-simulation),
ported into parameter-driven Python. It is **not a verified Tesla model**.

Implemented: passive suspension, saturated full-car LQR, centralized constrained
MPC, capability-aware force allocation, full-state and measured-output feedback,
spatial two-track roads, prescribed braking/cornering, payload/CG changes, a
separate nonlinear validation plant, holdout benchmarks, plots and 3D replay.
Simulation results and limitations are reported together; contact-loss stress
cases are retained.

## Setup

Tested with Python 3.10.12 on Linux x86-64. Use a virtual environment; no MATLAB
license is needed for the Python implementation. `requirements.lock` pins direct
and transitive dependencies from the clean installation.

```bash
cd /home/jayesh/Suspension
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements.lock
python -m pip install --no-deps -e .
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
```

If your shell injects ROS or other packages through `PYTHONPATH`, run setup/tests
with `env -u PYTHONPATH PYTHONNOUSERSITE=1` before the Python executable. The clean
setup was verified this way using a fresh venv without system packages. MP4 replay
needs `ffmpeg` on PATH; GIF export uses Pillow and does not need ffmpeg.

## Run

```bash
# Passive full car over a symmetric bump; NPZ logs, JSON metrics and plot.
python -m suspension run --scenario bump --controller passive --output results/passive_bump

# Four-controller comparison: asymmetric road + braking + cornering.
python -m suspension compare --scenario combined --sensing observer --plant mismatch --output results/comparison

# Ideal-information debugging comparison.
python -m suspension compare --scenario combined --sensing ideal --output results/ideal_comparison

# Interactive native Python viewer; no external video player required.
python -m suspension view results/comparison

# Complete frozen holdout suite: 22 cases × four controllers = 88 runs.
python -m suspension benchmark --output results/holdout

# Three-case smoke benchmark, explicitly not the complete validation.
python -m suspension benchmark --quick --output results/quick

# Full-car 3D replay with synchronized force, stroke and tire-load plots.
python -m suspension animate results/comparison/mpc --output results/full_car_replay.mp4
python -m suspension animate results/comparison/mpc --output results/full_car_replay.gif --fps 12

# Automated physics, upstream-equation, controller, observer and fallback tests.
python -m pytest -q --junitxml=results/test_results.xml
python scripts/verify.py
python scripts/portfolio.py
```

`python -m suspension --help` lists commands. Scenario names include `bump`,
`one_side`, `diagonal`, `dip`, `undulations`, `rough`, `sweep`, `brake`, `corner`,
`rough_maneuver`, `combined`, `fault`, `sensor_failure`, and `solver_failure`.
`--plant payload` adds a 250 kg offset payload with CG/inertia/equilibrium changes.
Road speed is specified in m/s. The animation exaggerates vertical motion ×4
and labels it; it is a diagnostic, not proof of correct physics.

## Interactive Python visualization

```bash
python -m suspension view results/comparison
```

This opens a native Python window using Tk and Matplotlib. It works directly
from the saved simulation logs, without `xdg-open`, GIO, a video codec, or MATLAB.
Choose `passive`, `lqr`, `mpc_fixed` or `mpc` in the controller menu. Playback time
and camera orientation are preserved when switching, and signal axes share
scales across controllers. The CG acceleration panel overlays the passive run;
the top readout compares whole-run RMS performance.

Use Play/Pause, Restart, the time slider, 0.25–2× playback speed, and the
perspective/side/front/top views. Dragging the timeline pauses playback. Drag
the 3D scene to rotate it. The scene shows all four wheels, body heave/pitch/roll,
and schematic springs/dampers. Right-hand plots show CG acceleration, body
attitude, delivered force, suspension travel and tire load. The per-corner table
shows instantaneous road height, stroke, force, load, available actuator force
and warnings. Red wheels indicate contact loss; dashed lines mark travel limits.
Road lines are schematic connections between the exact logged contact heights.

Motion exaggeration (1×/4×/8×) affects only the scene; plotted values and readouts
remain in physical units. This is an interactive **replay of completed numerical
experiments**, not a fresh simulation at every display frame. To view another
experiment, run `compare` or `run` first, then pass its saved folder:

```bash
python -m suspension view results/holdout/capability_loss --speed 0.5
python -m suspension view results/passive_bump/passive --scale 8
python -m suspension view results/comparison --time 2.8 --snapshot results/viewer.png
```

Python's Tk module is the only system GUI requirement (`python3-tk` on Ubuntu);
it is already available on the development host. Snapshot export works without
a graphical display. A [viewer preview](results/python_viewer_preview.png) is
also saved locally. Snap/GTK video-player library errors do not require changing
simulation dependencies to use this viewer.

## Results and interpretation

The generated [validation report](results/holdout/validation_report.md) contains
every holdout result, comfort/attitude trade-offs, violations and capability-update
ablations. [CSV metrics](results/holdout/metrics.csv) include per-corner force,
stroke, normal-load, contact, work, solver and estimator statistics.
[Comparison plot](results/comparison/comparison.png) and
[3D replay](results/full_car_replay.mp4) show the development combined maneuver.
[Verification evidence](results/verification.json) records integration refinement
and observer checks. Generated results are ignored by Git and can be recreated.

The completed 88-run holdout includes both ordinary and deliberate failure cases.
On `combined_nominal` (seed 101, 12 m/s, mismatched nonlinear plant, noisy sensing),
aware MPC reduced CG acceleration RMS 34.5%, seat acceleration RMS 34.1%, pitch
RMS 82.2%, and roll RMS 97.5% relative to passive, with no contact or travel
violations in that run. These are case-specific results. The severe rough-road
stress case loses contact for every controller and produces MPC solver failures;
LQR also exceeds stroke in the payload case. All appear in the report. The
capability-update ablation shows a modest benefit rather than a universal gain.

All 20 automated tests pass, including the new viewer tests, in the isolated
installation. Native window controls also passed a desktop smoke check.
The 2 ms versus 1 ms integration refinement changes CG RMS by
0.00024% in the declared convergence case. This supports the integration choice
for that case and is not a convergence guarantee for all contact/stop events.

Each saved run contains `run.npz`, `metadata.json`, and `metrics.json`. Logs
separate truth, noisy measurements and estimated state. Metadata records seed,
nominal/plant parameters, sensor assumptions, weights, dependency/host versions,
source fingerprint, upstream revision and solver statuses. `declaration.json`
records the frozen holdout configuration hash and acceptance criteria before
the suite starts. Development seeds 1–9 are distinct from holdout seeds ≥101.

All controller runs share the scenario, actual actuator plant and sensing mode.
The observer assumes body IMU/gyros, four suspension-travel sensors, four force
transducers, and **current road-height perception with 0.5 mm noise and a common
vertical reference**. This is a declared idealized sensing assumption; a vehicle
perception pipeline is not implemented. No future road preview is supplied.
Capability is supplied to aware MPC, not inferred by autonomous diagnosis.
Travel/tire constraints are soft; independent plant violations remain reported.

## Sources and scope

MATLAB R2025b Update 7 is the current installation, with Simulink,
Symbolic Math Toolbox and Fuzzy Logic Toolbox. **Activation and a 10-second
upstream Simulink run succeeded**, with 10001 finite samples in each of the eight
saved reference signals. See [MATLAB setup](docs/matlab_setup.md) for setup/run
commands and [execution summary](results/upstream_simulink/summary.json) for
evidence. R2024b was subsequently uninstalled; `matlab` now launches R2025b.
This installation does not port the Python MPC into the SLX diagram.

See [source manifest](docs/source_manifest.md) for commit, license, preserved files,
actual execution evidence and known upstream issues. The initial audit used
licensed MATLAB R2024b/Simulink. The upstream symbolic initialization ran,
but its unmodified R2025b Simulink model could not load in R2024b; the newly
installed matching release resolves that issue. The portable
Python stack avoids that version dependency and provides one complete controller
implementation. Upstream numeric equations were extracted from the SLX and
independently checked against both Python models. The upstream model now executes,
but quantitative upstream PID/fuzzy versus new MPC comparisons and optional
MathWorks 14DOF cross-validation have not been completed.

[Model conventions](docs/model.md) document signs, static load sharing, payload,
actuator assumptions, road spectrum, sensing, MPC scaling and limits.
[Contribution comparison](docs/contributions.md) distinguishes upstream work from
completed additions. The original [build brief](docs/build_brief.md) is preserved.

This reduced ride model uses small angles, diagonal inertias, vertical wheel
motion, unilateral spring tires and progressive stops. It omits yaw/slip,
electrical actuator efficiency, chassis flexibility, tire enveloping and sensor
bias/drift. Maneuvers are prescribed accelerations, not closed-loop handling.
Acceleration RMS is unweighted; mechanical work is not battery energy. Results
establish behavior in the tested model envelope, not ISO compliance, production
readiness, actual-vehicle validation, or research novelty.
