# 7-DOF Active Suspension Simulation

This repository is a MATLAB/Simulink project that simulates a car's suspension system (the spring-damper setup between the wheels and the body) on a computer. The goal is to answer this question:

> **"If we add a small computer/motor to a car's suspension (an active suspension), can we make the passenger feel bumps and potholes less than they normally would?"**

In short: **we're designing a smart shock absorber that makes the ride more comfortable**, and testing it on a computer before connecting it to a real car.

---

## 1. How is the car modeled?

A real car is very complex, so — as is always done in physics — we simplified it just enough to get useful answers. The car here consists of 3 main parts:

- **The body (chassis)** — the car itself, where the passengers sit. It can move in 3 different ways:
  - **Bouncing up and down** (heave)
  - **Tilting forward-backward** (pitch) — the "diving" feeling when braking, the "squatting" feeling when accelerating
  - **Leaning side-to-side** (roll) — the car leaning sideways in a turn
- **4 wheels** — each one can move up and down independently (when a wheel hits a pothole, it bounces while the body is affected much less — that's exactly the job of the suspension).
- **4 spring + damper units** — the flexible connection between each wheel and the body.

That adds up to **7 degrees of freedom** in total (3 for the body + 4 for the wheels), which is why this is known in the literature as a "**7-DOF full car model**." The math describing how these motions affect each other (the Lagrange equations) is derived symbolically at the top of the `.m` file — meaning the computer works out for itself the equations that answer the question "given the body's current position and speed, and a certain force applied to a certain wheel, what happens next."

## 2. How do the active controllers apply "force"?

In real life, this is done with a **hydraulic/electric actuator** (a small motor/piston) placed between the wheel and the body. In the simulation, we compute the force this actuator would produce (F1, F2, F3, F4 — one for each wheel) and add it into the physical equations. A real actuator also has limited power (it can't produce infinite force) — that's why we cap the force at a certain ceiling (`±2000 N`), just like a real motor has a limited amount of power it can deliver.

### Files

All project files live in the repository root:

- [`FULL_CAR_MODEL.m`](FULL_CAR_MODEL.m) — the physics: symbolic Lagrangian derivation of the 7-DOF equations of motion, plus the system parameters (masses, spring/damper/tire stiffness, gains).
- [`FULL_CAR_SIM.slx`](FULL_CAR_SIM.slx) — the Simulink model; runs the Passive, Fuzzy Logic (FLC), and PID controllers in parallel for comparison.
- [`FUZZY_CONTROL.fis`](FUZZY_CONTROL.fis) — the rule base and membership functions for the fuzzy logic controller.
- [`road_input.mat`](road_input.mat) — the random road profile (bump/pothole data) used to excite the model during simulation.

## 3. How to run it

1. In MATLAB, run [`FULL_CAR_MODEL.m`](FULL_CAR_MODEL.m) — this generates [`road_input.mat`](road_input.mat) and loads the parameters into the workspace.
2. Import [`road_input.mat`](road_input.mat) into the MATLAB workspace.
3. Open [`FULL_CAR_SIM.slx`](FULL_CAR_SIM.slx) and start the simulation.
4. Compare the 3 colored lines on the Scope screen.

> ⚠️ **Important Note on Parameter Changes:** 
> Because the system dynamics are derived symbolically using Lagrangian mechanics, any modifications to the vehicle parameters require a manual step. If you change any parameter in the script, you must re-run `FULL_CAR_MODEL.m` to generate the new equations of motion, and then manually copy-paste those updated equations into the corresponding MATLAB Function block inside the Simulink model (`FULL_CAR_SIM.slx`) before running the simulation.

## 4. Key parameters (briefly, what they mean)

| Parameter | Meaning | Everyday-life analogy |
|---|---|---|
| `ms` | Total weight of the body (the car) | The car's mass |
| `mu1..mu4` | Weight of each wheel + rim assembly | How "heavy" a wheel is |
| `ks1..ks4` | Spring stiffness | How "stiff" the spring is (higher = a stiffer spring) |
| `kt1..kt4` | Tire stiffness | How firm/inflated the tire is |
| `c1..c4` | Damper strength | How "gently" the damper absorbs the bump |
| `Fuzzy_Gain` | The magnitude of force produced by the smart controller | The motor's power |
| `Kp, Ki, Kd` | The PID controller's 3 tuning knobs | How stiff/soft/anticipatory its response is |

## 5. How to read the results

The Scope graph shows how the distance between the wheel and the body (the amount of suspension compression/extension) changes over time. **The faster and with less wobble it returns to 0, the more successful that controller is** — because that means the jolt felt by the passenger fades away that much sooner.

## 6. License

This project is licensed under the MIT License — free to use, modify, and cite. Attribution appreciated!

© 2026 Andac Y. Kaya