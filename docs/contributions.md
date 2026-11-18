# Upstream and completed contribution

| Area | Preserved upstream contribution | This implementation |
|---|---|---|
| Full-car physics | Seven-coordinate Lagrangian geometry, equal/opposite actuator mapping, representative numeric parameters | Parameter-driven matrices plus separately written force-based validation RHS; 18 states with actuator lag; coordinate and equilibrium audit |
| Parameter changes | Symbolic rerun plus manual copy of numeric equations into blocks | Single JSON parameter source; automatic matrix, equilibrium and experiment construction |
| Controllers | Passive, PID and fuzzy Simulink paths | Passive, actuator-augmented LQR, nominal MPC and capability-aware centralized MPC with four coordinated forces |
| Constraints | Force clipping in reference controller paths | Hard command force/slew limits, actual delivered-force lag/slew/caps, soft predicted stroke/contact limits and independently measured violations |
| Roads | Stored MAT road and commented random temporal generation | Seeded correlated spatial tracks, rear wheelbase offset, obstacle suite, multiple speeds and variable-speed distance |
| Maneuvers | No validated prescribed maneuver suite claimed | Signed inertial pitch/roll moments from declared braking/cornering accelerations; combined road/maneuver cases |
| Robustness | No automated robustness validation claimed | Payload CG/inertia/equilibrium changes, asymmetric mismatch, reduced force authority, noisy sensing, invalid measurements and solver-failure injection |
| Sensing | Reference state signals | Ideal-information debugging and measured-output Kalman observer; explicit sensors/noise, rank and estimator stability checks; bounded fallback |
| Validation | Scope-based comparisons; original model now executed in matching R2025b | Automated physics/upstream-equation/convergence/control tests; held-out scenarios; per-corner safety/comfort/effort/solver metrics |
| Outputs | Simulink model and scopes | CLI, NPZ/JSON logs, CSV, plots, full-car synchronized 3D replay, generated report and tested isolated setup |

The upstream model's 10-second execution has now been reproduced in activated
MATLAB/Simulink R2025b Update 7. Saved outputs and the execution log are in
`results/upstream_simulink/`. Quantitative comparisons with upstream PID/fuzzy
have not been completed; no improvements over those controllers are claimed.
The reported Python comparisons use the implemented passive, LQR and nominal
MPC baselines on identical inputs and sensing.

Measured portfolio bullets are generated in `results/portfolio_bullets.md` after
the benchmark completes. They describe completed simulation results only.
