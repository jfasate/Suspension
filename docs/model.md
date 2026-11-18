# Model and controller conventions

Corner order is FL, FR, RL, RR. Units are metres, seconds, kilograms, newtons and radians.
World/body axes at the equilibrium orientation are x forward, y left and z up.
Body coordinates are `q_b = [z, alpha, beta]`: alpha is the front-up longitudinal
slope; beta is the left-up lateral slope. Under right-hand rotations around these
axes, alpha = negative pitch about +y, beta = positive roll about +x. Displayed
"pitch" and "roll" mean these slopes. MathWorks uses x forward, y right, z down:
its heave is `-z`, its small-angle pitch is `alpha`, and its roll is `beta` under
the corresponding right-hand rotations. No MathWorks integration is claimed.

For body CG-relative corner coordinates `(x_i,y_i)`, `H_i=[1,x_i,y_i]` and
`G=[H,-I4]`. Seven mechanical positions are `q=[q_b,z_FL,z_FR,z_RL,z_RR]`.
The state is `[q(7), qdot(7), delivered_force(4)]` with 18 entries. Positive active
force pushes up on the body and down on its wheel.

```
travel = G q
relative_velocity = G qdot
F_body = -ks * travel - cs * relative_velocity + delivered_force + stop_force
qdd_body = (H.T @ F_body + [0, ms*h*ax, ms*h*ay]) / [ms, I_pitch, I_roll]
normal_load = max(0, N_static + kt * (road - z_wheel))
zdd_wheel = (-F_body + normal_load - N_static) / mu
force_dot = clip((clip(command, -capability, capability) - force)/tau, -slew, slew)
```

The controller model omits stops/contact clipping and uses exact zero-order-hold
discretization, including lag. The independently written validation RHS evaluates
forces directly with RK4 and input values at all substages. It does not multiply
the controller A/B matrices. It adds asymmetric parameter changes, optional payload,
unilateral tires, and progressive travel stops. Force state alone is projected to
the physical available-force interval; body/wheel positions are never clamped.
Capability loss ramps over 0.4 s: its 3750 N/s bound reduction is below the declared
30000 N/s physical force slew. This avoids pretending an instantaneous cap drop
can satisfy finite force slew.

External maneuver moments follow the small-angle free body using inertial forces
`(-ms*ax,-ms*ay,0)` acting at height h relative to the declared ground reaction
reference. Forward acceleration gives a front-up slope; left acceleration gives
left-up slope and higher right-wheel loads. Braking reverses pitch. These are
prescribed tests, not steering/braking control or a planar handling model. They
omit roll/pitch center kinematics and unsprung inertial load transfer.

Static sprung axle shares are `W_front=ms*g*b/(a+b)` and
`W_rear=ms*g*a/(a+b)`. Each axle splits sides as `0.5 ± cg_y/track`.
This is the declared load-sharing assumption; three balance equations alone do
not determine four loads. `N_static = W_sprung_corner + mu*g`,
`travel_equilibrium=-W_sprung_corner/ks`. Static preload/rest lengths accommodate
the stated equilibrium; all simulated states begin at its perturbation zero.
The payload is a point mass at a declared position. Its CG shift changes a, b,
height and y offset; pitch/roll inertias use the parallel-axis theorem. Cross
inertias remain neglected. Static loads and equilibrium are recomputed. Fixed
hardware stroke is measured relative to nominal equilibrium, so extra payload
or changed stiffness consumes stroke. Travel sensors are assumed calibrated to
each experiment's loaded equilibrium; consumed hardware stroke is checked
independently by the validation plant. Controller prediction retains the nominal
equilibrium and does not receive hidden payload parameters.

Progressive stops act when hardware stroke exceeds ±80 mm, using
`-sign(stroke)*(k_stop*penetration + k_cubic*penetration^3)`. Excursions remain
visible and counted. Tire contact uses a unilateral vertical spring. Loss of
contact produces zero normal force, not tire tension; reconnection has no impact
or tire enveloping model. Stress cases with contact loss are flagged, not treated
as high-fidelity airborne-wheel validation.

Seat location is `(0.5,0.35)` m from nominal CG, fixed to the body; loaded CG shifts
are included in metric geometry. Vertical seat acceleration is
`zdd + x_seat*alpha_dd + y_seat*beta_dd`. Higher-order centripetal terms are omitted
with the small-angle model. All acceleration RMS values are unweighted.

# Roads, sensing and optimization

Roads have two spatial tracks. Rear wheels sample the same track at traveled
distance minus wheelbase; there are no independent rear random signals. Roughness
is a 64-frequency synthetic spatial spectrum from 0.025 to 3 cycles/m, with
amplitude proportional to `1/sqrt(f)`, per-track ensemble RMS 3 mm and shared
component correlation 0.65. Individual finite records can differ. Obstacles use
raised cosines. Variable-speed tests integrate the specified speed analytically.
The 25 m/s, 16 mm roughness stress case is deliberately beyond ordinary cases.
Neither the spectrum nor acceleration metrics claim an ISO class/compliance.

Ideal-information debugging supplies exact full state and current disturbance.
Measured-output runs use Gaussian independent zero-mean noise: vertical IMU
0.035 m/s², pitch/roll gyros 0.002 rad/s, four suspension-travel sensors 0.5 mm,
four delivered-force transducers 12 N, longitudinal/lateral acceleration inputs
0.025 m/s², and current four-corner road-height perception 0.5 mm. The latter
assumes a common vertical reference and is an idealized sensing assumption, not
a demonstrated perception pipeline. No future preview is supplied. Measurements,
estimates, truth and current measured disturbance are saved separately.

The discrete Kalman observer uses those measured inputs and the nominal model.
Its 11 output channels make all 18 states observable conditional on the declared
known disturbance channels; an orthonormal observability-space expansion checks
rank without poorly scaled powers. DARE provides the gain. Sensor dropout or
gross output outliers invalidates the estimate, ramps force commands toward zero,
and requires three valid updates to resume. MPC solver failure uses the same
bounded ramp. Passive spring/damper forces continue throughout. This is a tested
fallback policy, not a safety certification. IMU bias/drift is not simulated.

MPC optimizes all four commands together for a 20-step, 0.4 s horizon at 50 Hz.
Dimensionless states/output scales are explicit in `model.py`; weights live in
`config/control.json`. Costs cover CG/seat acceleration, body slopes/rates, all
corner travel, dynamic tire load, effort and force increments. Force and slew
constraints are hard. Travel and minimum normal load (10% nominal static load)
use nonnegative per-corner slacks with a quadratic penalty of 10000. A nonzero
slack is a predicted violation allowance, never a proof of safety. Current road
and acceleration measurements are held across the prediction horizon. OSQP's
solved status is required; other statuses trigger fallback. A horizon-terminal
stability certificate and recursive-feasibility proof are not claimed.

LQR uses the same output cost and actuator-augmented nominal model, then applies
the same command slew/force bounds. Capability-aware MPC receives supplied force
authority and changes its command feasible set. Fixed MPC retains nominal bounds;
the actual actuator still applies physical clipping. No gain loss is injected and
capability is not applied twice. Both MPC variants share estimator, cost, solver
and fallback, isolating the bound update.

Signed mechanical actuator power is `force * relative_velocity`; its signed and
absolute integrals are logged. This is a fully active force actuator assumption.
A force bound is not a power bound, and work is not battery energy.
