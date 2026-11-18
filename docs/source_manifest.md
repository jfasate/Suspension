# Source manifest

Inspected 5 November 2026. Web review is distinguished from local execution below.

Environment update: MATLAB **R2025b Update 7** was installed successfully into
`/home/jayesh/MATLAB/R2025b` alongside R2024b, including Simulink, Symbolic Math
Toolbox and Fuzzy Logic Toolbox. Its first batch launch stopped before evaluating
MATLAB code with Licensing Error 1 / -1.2 (no configured MATLAB license). After
activation, all four product license checks returned true and the unmodified
reference model completed a 10-second simulation. Eight logged reference signals
contain 10001 finite samples each; `tout` reaches 10 seconds. Evidence is in
`results/upstream_simulink/summary.json`, `execution.log` and `simulation_output.mat`.
See `matlab_setup.md`. R2024b notes remain a record of the earlier audit.
The old R2024b installation was subsequently uninstalled at the user's request;
R2025b is now the only installed release and passes the post-removal startup and
MATLAB/Simulink license checks.

| Resource | Revision/license | Use and verified behavior | Changes / limits |
|---|---|---|---|
| [andorianfm/7dof-active-suspension-simulation](https://github.com/andorianfm/7dof-active-suspension-simulation) | `098f9fdbc5c311d64110f1da30d791644c9f4a77`, MIT, copyright 2026 Andac Y. Kaya | Full clone inspected. Complete six-file source snapshot in `third_party/upstream/`; original MIT notice preserved. | Equations and baseline parameters adapted into portable numeric Python; no manual equation copying, symbolic or fuzzy dependency at runtime. |
| [FULL_CAR_MODEL.m](https://github.com/andorianfm/7dof-active-suspension-simulation/blob/098f9fdbc5c311d64110f1da30d791644c9f4a77/FULL_CAR_MODEL.m) | same revision | Executed in MATLAB R2024b Update 3. Symbolic derivation completed, `road_input.mat` and `readfis` initialization worked. Full output in `upstream_matlab_run.log`. | Script loads road/FIS before derivation; road generator is commented out. `eqns` starts with four rows then grows to seven. Numeric parameter substitutions are embedded separately in Simulink. |
| [FULL_CAR_SIM.slx](https://github.com/andorianfm/7dof-active-suspension-simulation/blob/098f9fdbc5c311d64110f1da30d791644c9f4a77/FULL_CAR_SIM.slx) | same revision, saved R2025b | Seven numeric passive-plant equations extracted from `chart_34.xml`; eight independent fixtures match both Python models to 1e-12. After installing/activating R2025b Update 7, the original model completed a 10-second run; eight logged signals have 10001 finite samples each. | Initial R2024b compatibility failure resolved by matching release. Raw reference execution verified, but quantitative PID/fuzzy versus MPC comparisons and physical acceptance of its excursions are not established. |
| [OSQP](https://github.com/osqp/osqp), [Python interface](https://osqp.org/docs/interfaces/python.html) | Python package `1.0.4`, Apache-2.0; pinned in requirements | Installed API verified by QP solves, vector updates, solver status and failure tests, and benchmark. | Used as dependency, source not vendored. Sparse normalized QP implementation is new project code. |
| [MathWorks passenger models](https://www.mathworks.com/help/vdynblks/ug/passenger-vehicle-dynamics-models.html) | Online R2026b docs, proprietary | Documentation inspected. Passenger 14DOF includes vertical body and wheel motion; Passenger 7DOF omits suspension ride DOFs. Installed MATLAB/Simulink R2024b and Vehicle Dynamics Blockset are listed by `ver`; MATLAB/Simulink license checks returned true. | Optional 14DOF integration/cross-validation not completed. No MathWorks product code copied. |
| [Vehicle Body 6DOF](https://www.mathworks.com/help/vdynblks/ref/vehiclebody6dof.html) | Online R2026b docs, proprietary | Coordinate orientation and rigid-body interface documentation inspected; mapping recorded in `model.md`. | Reference only; no claim of executing this block or matching it. |

Upstream parameters are a representative car, not verified Tesla data: sprung mass
1000 kg, pitch/roll inertias 1200 kg m², wheel masses 70 kg, wheelbase 2.7 m,
track 1.4 m, springs 30 kN/m, dampers 1502 Ns/m, tires 200 kN/m. CG height 0.55 m,
stroke, stops, actuator lag/slew/capability and sensor characteristics are explicit
project assumptions. The upstream README describes force capping as a power
limit; this project corrects that interpretation and reports mechanical work.

`upstream_files.json` records SHA-256 checksums of every preserved upstream file.
`tests/upstream_equation_fixtures.json` records source revision and SLX checksum.
No production vehicle, research novelty, or MATLAB 14DOF validation is claimed.
