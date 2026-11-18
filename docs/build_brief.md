# Full-Car Active Suspension Control — Simulation Build Brief

Owner: Jayesh Fasate
Updated: 5 November 2026
Deliverable: A working, reproducible full-vehicle suspension simulation and validated controller.

## 1. Task for Codex

Build the project described here. Inspect existing open source code, reuse suitable components with attribution, improve the model and controllers, debug the complete system, and deliver working simulation commands, comparison plots, animations and a validation report.

The project starts with the complete car: four wheels and a coupled vehicle body. Full-car simulation is the main deliverable from the first milestone, not a later extension. Do not replace implementation with a tutorial, another plan, or disconnected single-wheel demonstrations.

This brief is exclusively a simulation project. All deliverables below are software models, numerical experiments and analysis.

## 2. Selected problem statement

**Develop and validate a coordinated four-corner active suspension controller that reduces body heave acceleration, pitch and roll during uneven-road driving, braking and cornering while respecting suspension travel, tire-contact and actuator-force limits across payload changes and reduced actuator capability.**

The engineering problem is a coupled trade-off: reducing passenger vibration can increase wheel-load variation or consume suspension travel, and independently controlling each corner can miss whole-body pitch/roll interactions. A useful automotive controller must manage all four corners together and remain useful away from one nominal mass and one tuned road bump.

Primary contribution: full-car constrained MPC with coordinated corner forces, evaluated against passive suspension and full-car LQR on a separate, more demanding simulation plant. Add capability-aware force allocation within the MPC and compare it against fixed-capability MPC when one corner loses authority.

Nominal ride and handling performance comes first. Fault cases are a robustness test, not a replacement for demonstrating that the controller works in normal driving.

This is a portfolio engineering contribution. Do not claim research novelty, production readiness or actual-vehicle validation from simulation alone. Use a documented passenger-car parameter set; label it representative unless parameters for a specific vehicle are verifiable.

## 3. Verified resources and reuse decisions

Resources checked on 5 November 2026. Web inspection is not execution verification. Codex must pin revisions, inspect actual files, verify licenses and run relevant examples before claiming reproduction.

| Resource | What it supplies | Required adaptation |
| --- | --- | --- |
| [andorianfm/7dof-active-suspension-simulation](https://github.com/andorianfm/7dof-active-suspension-simulation) | MIT-licensed full-car MATLAB/Simulink model: heave, pitch, roll, four vertical wheel motions; passive, PID and fuzzy comparisons | Primary starting point. Audit equations/units, remove manual parameter-transfer steps, add reproducible road generation, centralized MPC, model uncertainty and automated metrics |
| [FULL_CAR_MODEL.m](https://github.com/andorianfm/7dof-active-suspension-simulation/blob/main/FULL_CAR_MODEL.m) | Symbolic derivation, parameter definitions and initialization | Inspect force signs and corner geometry. Replace repeated manual equation edits with parameter-driven matrices/functions |
| [FULL_CAR_SIM.slx](https://github.com/andorianfm/7dof-active-suspension-simulation/blob/main/FULL_CAR_SIM.slx) | Full-car simulation and controller comparison | Reuse suitable plant components after verification; replace scope-only assessment with logged metrics and scripted execution |
| [MathWorks Passenger Vehicle Dynamics Models](https://www.mathworks.com/help/vdynblks/ug/passenger-vehicle-dynamics-models.html) | Passenger 14DOF vehicle includes body longitudinal/lateral/vertical motion, pitch/roll/yaw and wheel vertical/rolling motion | Optional higher-fidelity cross-validation when Vehicle Dynamics Blockset is available; inspect suspension force interfaces before integration |
| [MathWorks Vehicle Body 6DOF](https://www.mathworks.com/help/vdynblks/ref/vehiclebody6dof.html) | Rigid body with forces/moments, mass/inertia, loading and six body motions | Reference or component for independent ride-and-handling validation; not a complete suspension controller/model by itself |
| [OSQP](https://github.com/osqp/osqp), [documentation](https://osqp.org/docs/) | Open source QP solver | Default solver for a portable implementation or an available MATLAB interface; verify installed API and supported updates |

Important naming distinction: the GitHub project's 7DOF means three body ride motions plus four vertical wheels. MathWorks' Passenger 7DOF means planar body motion plus wheel rotation and omits body heave/pitch/roll suspension dynamics. Do not select it for this ride-control project based on its DOF count. Use the Passenger 14DOF variant if taking that route.

The GitHub README documents manual copying of changed equations into Simulink. Remove that fragility. The script uses symbolic variables and `readfis`; check Symbolic Math Toolbox/Fuzzy Logic Toolbox dependencies. Do not require the fuzzy controller merely to initialize the passive or new MPC plant. Its road-file loading/generation workflow also needs verification.

Keep `docs/source_manifest.md`: upstream URL, commit SHA, license, reused files, verified behavior, changes, known issues. Preserve notices. MATLAB reference products are proprietary, not open source.

## 4. Implementation environment

Prefer MATLAB/Simulink when already installed/licensed because the primary source is a full-car Simulink project. Inspect available products first. Use scripted numeric model construction and programmatic simulation so parameter changes and experiments do not require hand-editing blocks.

If MATLAB or needed products are unavailable, port the licensed full-car equations and implement the same project in Python with NumPy/SciPy, OSQP and Matplotlib. Complete one working implementation rather than maintaining two incomplete stacks. Document the chosen stack and exact dependencies. A Python port must preserve the complete coupled vehicle, not shrink the model.

Do not assume Purdue product access. Missing proprietary tools must not prevent a portable full-car implementation. No named commercial vehicle simulator is required.

## 5. Full-car plant specification

### Minimum model

Use seven mechanical coordinates:

- Sprung-body heave, pitch and roll.
- FL, FR, RL and RR vertical wheel motion.

This gives 14 mechanical states (positions and velocities), augmented with four delivered actuator-force states if lag is modeled. Seven DOF does not mean seven first-order states.

Include wheelbase, front/rear track, CG position/height, sprung mass, roll/pitch inertia, four unsprung masses, four spring/damper parameter sets and four tire vertical stiffnesses. Keep corner order FL, FR, RL, RR everywhere.

Do not describe this reduced full-car ride model as a complete digital replica of a production car. It represents the motions needed for suspension control. Broader yaw/trajectory/tire-slip claims require an explicitly coupled handling model.

### Consistent geometry and equations

Codex must document one coordinate convention and validate all signs. An implementation-friendly small-angle convention is:

```text
q = [z_body, alpha, beta]
alpha = longitudinal body slope (positive front higher)
beta  = lateral body slope (positive left higher)
x_corner = [a, a, -b, -b]
y_corner = [tf/2, -tf/2, tr/2, -tr/2]
H[i,:] = [1, x_corner[i], y_corner[i]]
z_body_corner = H*q
travel = H*q - z_wheel
F_body = -K*travel - C*travel_rate + F_active
M_body*q_ddot = H' * F_body + generalized_external_loads
M_wheel*z_wheel_ddot = -F_body + K_tire*(road-z_wheel)
```

`M_body = diag(ms, I_pitch, I_roll)` under the stated decoupled inertia assumptions. Alpha/beta are geometric slopes; map them explicitly to the chosen right-hand pitch/roll conventions and to any MathWorks coordinates. The MathWorks body block uses X-forward/Y-right/Z-down, which differs from the example above.

Use perturbations about static equilibrium in the linear model. Compute static corner loads from mass, CG geometry and a documented suspension/load-sharing assumption. Force/moment balance alone does not uniquely determine all four corner loads. Recompute equilibrium and inertias consistently for changed payload; mass changes alone are not a complete CG/payload model.

Tire normal load = static corner load + dynamic contribution under the chosen signs. Avoid counting static load twice. Zero active force must leave a functioning passive spring/damper system.

### Actuator model

At each corner include commanded versus delivered force, force limits, force slew and lag. Bounds are configurable simulation assumptions unless supported by published actuator data. Reduced capability changes the available force set; distinguish this from gain loss and avoid double derating.

This project assumes fully active force actuators. A force cap is not a power limit. Log mechanical power separately; do not imply an arbitrary signed-force controller is a variable damper.

## 6. Driving and road excitation

Build a spatial road generator with two tracks and wheelbase-consistent rear-wheel delay. At constant speed, rear input follows the same track after wheelbase/speed. Use traveled distance for variable speed. Explicitly choose left/right roughness correlation; four unrelated noise traces are not a realistic default road.

Required scenarios:

1. Full-width bump: symmetric left/right with front/rear timing.
2. One-side bump and diagonal obstacle: excite coupled roll and pitch.
3. Pothole/dip and repeated undulations.
4. Seeded correlated rough roads at multiple speeds and a spatial/frequency sweep.
5. Braking/acceleration on smooth and rough roads: pitch/load-transfer excitation.
6. Cornering on smooth and rough roads: roll/load-transfer excitation.
7. Combined braking and cornering with road disturbance within the declared model range.
8. Changed payload/CG, parameter mismatch and one-corner capability loss.

A road-only vertical model does not automatically contain braking or lateral load transfer. Add signed inertial pitch/roll generalized loads from declared acceleration histories and moment arms, derived consistently from the chosen free body. Avoid double counting if a higher-fidelity body model already includes them. These are prescribed-maneuver tests unless acceleration is generated by a coupled planar model. Do not label them as closed-loop steering/braking simulation.

## 7. Controller to build and improve

### Baselines

Use passive suspension, saturated full-car LQR and nominal centralized MPC. Retain upstream PID/fuzzy results only if they run correctly and are useful; they are not required dependencies.

### Proposed centralized MPC

Use all four force commands in one optimization. Penalize body/occupant vertical acceleration, pitch/roll responses, travel at all corners, dynamic tire loads, actuator effort and force changes. Scale quantities so numerical units do not determine priorities accidentally.

Constrain force and slew at each corner. Add travel and normal-load limits with explicit feasibility handling. Soft constraints permit violations and must be reported as such. Do not clamp the vehicle state to make a travel violation disappear; the validation plant must model limits or flag them.

Include actuator lag in prediction and update each corner's capability bounds. Let optimization coordinate the remaining force authority when one corner degrades. Distinguish supplied actuator capability from inferred faults; no hidden fault-injector truth may enter an estimator presented as autonomous diagnosis.

Tune initially for comfort, body attitude and tire contact together. Optimize on development scenarios, freeze tuning, then evaluate holdouts. Demonstrate trade-offs rather than assuming every metric can improve simultaneously.

### Sensing and state estimation

First implement full-state feedback for controller debugging and label it ideal-information evaluation. Then implement a measured-output observer using declared simulated body IMU signals, suspension travel and other justified channels. Check observability/detectability of the selected states, including actuator states. Keep truth, noisy measurements and estimates separate.

During invalid measurements or solver failure use a tested fallback with bounded transitions. Do not keep driving feedback control on invalid state estimates. Fault robustness must not hide deficiencies in nominal suspension control.

## 8. Validation plant and engineering checks

Do not use an identical linear controller model and perfect states as the sole proof. Maintain a separate validation plant with parameter mismatch, actuator limits/lag, sensor noise, asymmetric parameters and nonlinear travel-stop behavior. Include tire-contact handling or terminate/flag contact-loss cases; a linear tire spring allowing tension is not physical contact validation.

If available, cross-check selected maneuvers against the MathWorks 14DOF vehicle after verifying units, signs, configuration and active-force interfaces. This is an optional independent reference, not a reason to delay the core simulation.

Required checks: equilibrium, matrix dimensions, units, opposite body/wheel actuator reactions, left/right symmetry, appropriate roll-sign response to a one-side disturbance, front/rear pitch response, passive energy decay on a flat road, integrator convergence and linear-versus-nonlinear agreement for small inputs. Check force/moment balance when mapping corner loads.

A 3D animation is a diagnostic output, not evidence that equations are correct.

## 9. Experiments and reported metrics

Use identical scenarios, actual actuator plant and sensing assumptions for controller comparisons. Include multiple rough-road seeds and declared speed/load ranges. Compare nominal and capability-aware MPC with the same estimator/fallback to isolate the capability update contribution.

| Objective | Metrics |
| --- | --- |
| Comfort | RMS/peak vertical acceleration at CG and declared seat locations; pitch/roll angular acceleration where relevant |
| Body attitude | Pitch/roll RMS, peaks and recovery/settling under maneuvers |
| Travel | Per-corner peak/RMS and duration beyond limits |
| Road holding | Per-corner tire-load variation, minimum normal load and contact-loss events |
| Effort | Force peaks/RMS, force-rate limits, saturation duration, signed and absolute mechanical work |
| Robustness | Performance across payload/CG/mismatch, capability loss, noisy measurements and solver failures |
| Numerical cost | Solver time/iterations/failures and numerical convergence on the named host |

Seat acceleration must include rotational contributions using the modeled geometry/kinematics. Call ordinary acceleration RMS unweighted; do not claim ISO ride-comfort compliance without implementing its required weighting and procedures. Mechanical work is not battery energy without an efficiency/electrical model.

Predeclare primary objectives and allowable travel/contact violations. Report improvements and regressions, all failed scenarios and invalid-model regimes. No invented success percentage or selective removal of unfavorable runs. Simulation validates behavior only within the tested model and parameter envelope.

## 10. Build milestones and acceptance

| Milestone | Required working output |
| --- | --- |
| 1. Coupled full-car plant | Parameter-driven four-wheel model, road generator, passive run, logs, physics checks |
| 2. Baseline control | Full-car LQR and nominal centralized MPC; comparable plots |
| 3. Improved control | Capability-aware MPC, realistic actuator/measurement models and observer |
| 4. Validation | Nonlinear/mismatched plant, maneuver suite, holdouts and ablations |
| 5. Deliverable | Reproducible commands, animation, generated report and tested clean setup |

Complete each milestone through executable code and tests. Fix numerical, licensing or dependency issues using available resources. Do not stop at a proposed architecture.

Suggested folders: `model/`, `controllers/`, `roads/`, `scenarios/`, `config/`, `tests/`, `scripts/`, `results/`, `docs/`, `third_party/`.

Required entry points, implemented in the chosen stack:

- Run a passive full-car bump scenario.
- Compare all controllers on asymmetric road plus braking/cornering excitation.
- Run the complete holdout benchmark and export CSV metrics.
- Replay one saved run as a full-car animation with synchronized signal plots.
- Run automated physics, controller and regression tests.

Use one parameter source and scripted experiments. No manual copying of symbolic equations between blocks after a parameter change. Log configuration, seed, dependency versions, source revisions, controller weights and model assumptions with each run.

## 11. Final outputs

1. Working full-car suspension simulation repository with pinned dependencies and source attribution.
2. Centralized controller source and reusable road/maneuver scenarios.
3. Automated baseline/proposed-controller comparison on a separately configured validation plant.
4. Full-car animation showing all wheels, heave, pitch and roll; synchronized plots show forces, travel and tire loads. Label any visual scale exaggeration.
5. Generated validation report with model checks, metrics, trade-offs, failure cases and limitations.
6. README with exact setup/run/test commands and completed-scope status.
7. Before/after upstream contribution table and resume bullets based only on measured completed results.

## 12. Immediate instruction to Codex

**Build the full-car simulation now. Inspect and adapt the seven-mechanical-DOF GitHub model, implement coordinated four-corner control, add realistic road and maneuver inputs, validate against a more demanding plant, and produce working commands, plots, animation and quantified comparisons. Preserve the complete vehicle throughout. Do not deliver another plan in place of code.**
