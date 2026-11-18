# MATLAB / Simulink reference environment

Status on 5 November 2026: **R2025b Update 7 installed, activated and verified**
with all four products below (8.5 GB). MATLAB reports version
`25.2.0.3360999 (R2025b) Update 7`. License checks for MATLAB, Simulink, Symbolic
and Fuzzy returned true. The first launch required license configuration; after
activation, the original initialization and **10-second Simulink run completed**.
All eight saved `z1..z4` / `d1..d4` signals contain 10001 finite samples; the saved
time vector reaches 10 seconds. Evidence is in
`results/upstream_simulink/summary.json`, `execution.log` and
`simulation_output.mat`. The old root-owned R2024b installation was subsequently
removed at the user's request, reclaiming approximately 25 GB. Only R2025b is
installed now. The `matlab` command resolves through
`/home/jayesh/.local/bin/matlab` to this release. A post-removal batch check verified
R2025b startup and MATLAB/Simulink license availability.

This verifies execution of the reference, not its engineering acceptance or an
MPC comparison. Raw upstream signal names/units and large excursions require
interpretation before reuse as performance evidence. Python benchmark results
remain separate from this reference run.

The preserved upstream SLX was saved with R2025b. R2024b cannot load it using
the default compatibility checks. Install R2025b separately to use the matching
release; an existing working R2024b installation can remain available.

The user-owned destination on this host is `/home/jayesh/MATLAB/R2025b`.
The required reference products are MATLAB, Simulink, Symbolic Math Toolbox
(`syms`/the original derivation) and Fuzzy Logic Toolbox (`readfis`/FLC blocks).
The Python implementation continues to run independently of these products.

MathWorks Package Manager installs the product files. Installing files does
not establish license entitlement; launching MATLAB still requires a valid
license for the release and its products. Use the MathWorks account/activation
interface if the installation requests authentication. No credentials belong in
this repository.

```bash
cd /home/jayesh/Suspension
bash scripts/install_matlab_r2025b.sh

# Launch the new desktop, when desired.
/home/jayesh/MATLAB/R2025b/bin/matlab

# Activate first if startup reports that no license was found.
/home/jayesh/MATLAB/R2025b/bin/glnxa64/MathWorksProductAuthorizer.sh

# Run a 10-second upstream reference experiment and save its results.
/home/jayesh/MATLAB/R2025b/bin/matlab -batch "addpath('scripts'); run_upstream_reference();"
```

The runner copies the six preserved upstream files into a temporary working
directory, executes the original initialization in the base workspace, and runs
the original Simulink model. It saves `simulation_output.mat`, `summary.json`
and `execution.log` into `results/upstream_simulink/`. Original vendored files
are not edited. This is execution of the upstream passive/PID/fuzzy model;
the project's new Python MPC is not automatically embedded in that diagram.

Optional arguments allow another output folder and duration:

```matlab
addpath('scripts');
run_upstream_reference('results/upstream_smoke', 1);
```

Installation log: `/home/jayesh/.local/share/mathworks-install/R2025b-install.log`.
Startup/license check log: `/home/jayesh/.local/share/mathworks-install/R2025b-reference-check.log`.
The installer and release files live outside the Git repository.

Official instructions: [Get a new MATLAB release](https://www.mathworks.com/help/install/ug/get-new-matlab-release.html),
[get MPM](https://www.mathworks.com/help/install/ug/get-mpm-os-command-line.html),
and [MPM install](https://www.mathworks.com/help/install/ug/mpminstall.html).
