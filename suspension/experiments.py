"""Declared development and held-out scenarios; tuning configuration is hashed."""
from dataclasses import asdict
import csv
import hashlib
import json
from pathlib import Path
import numpy as np
from .model import Vehicle, ROOT
from .roads import Scenario
from .controllers import control_config
from .simulation import simulate, save_run
from .visualization import comparison_plot

CONTROLLERS=("passive","lqr","mpc_fixed","mpc")


def holdout_suite():
    # Seeds 1..9 are development-only; these cases are frozen before evaluation.
    cases=[]
    for name in ("bump","one_side","diagonal","dip","undulations","sweep","brake","corner","combined"):
        cases.append((f"{name}_nominal",Scenario(name,seed=101,speed=12,duration=7),"mismatch"))
    for seed in (103,211,307):
        for speed in (8.,20.):
            cases.append((f"rough_{seed}_{speed:g}",Scenario("rough_maneuver",seed=seed,speed=speed,duration=7),"mismatch"))
    cases.extend([
        ("payload_cg",Scenario("combined",seed=401,speed=15,duration=7),"payload"),
        ("variable_speed",Scenario("rough_maneuver",seed=409,speed=15,duration=7,variable_speed=True),"mismatch"),
        ("capability_loss",Scenario("fault",seed=419,speed=12,duration=7),"mismatch"),
        ("payload_capability",Scenario("fault",seed=421,speed=15,duration=7),"payload"),
        ("invalid_sensor",Scenario("sensor_failure",seed=431,speed=12,duration=7),"mismatch"),
        ("failed_solver",Scenario("solver_failure",seed=433,speed=12,duration=7),"mismatch"),
        ("contact_stress",Scenario("rough",seed=439,speed=25,duration=7,roughness=.016),"mismatch")])
    return cases


def plant_for(kind,p):
    if kind=="payload":return p.payload(250.,x=-.4,y=.15,height=.8).mismatch()
    return p.mismatch() if kind=="mismatch" else p


def write_csv(rows,path):
    fields=list(dict.fromkeys(k for row in rows for k in row))
    with Path(path).open("w",newline="") as stream:
        writer=csv.DictWriter(stream,fieldnames=fields);writer.writeheader();writer.writerows(rows)


def benchmark(output,sensing="observer",quick=False):
    output=Path(output);output.mkdir(parents=True,exist_ok=True)
    cfg=control_config();encoded=json.dumps(cfg,sort_keys=True).encode()
    cases=holdout_suite()
    if quick:cases=[cases[0],cases[8],cases[17]]
    declaration={"control_sha256":hashlib.sha256(encoded).hexdigest(),"control":cfg,
                 "primary_objectives":["unweighted CG and seat acceleration RMS","pitch and roll RMS"],
                 "safety":{"travel_violation_corner_seconds":0,"contact_events":0,
                           "maximum_body_angle_rad":.12,"maximum_absolute_stroke_m":.15},
                 "development_seeds":list(range(1,10)),"tuning_frozen":True,
                 "suite":[{"id":case,"scenario":asdict(s),"plant":kind} for case,s,kind in cases]}
    (output/"declaration.json").write_text(json.dumps(declaration,indent=2)+"\n")
    rows=[];errors=[];p=Vehicle.load()
    for case,scenario,kind in cases:
        runs=[]
        for controller in CONTROLLERS:
            print(f"{case}: {controller} ({sensing})",flush=True)
            try:
                run=simulate(scenario,controller,sensing,plant_params=plant_for(kind,p),cfg=cfg)
                a,meta,metrics=run;metrics["case"]=case
                save_run(output/case/controller,*run)
                rows.append(metrics);runs.append(run)
            except Exception as exc:
                errors.append({"case":case,"controller":controller,"error":str(exc)})
                print(f"FAILED {case}/{controller}: {exc}",flush=True)
            # Incremental output survives interruptions; failed runs never disappear.
            write_csv(rows,output/"metrics.csv")
            (output/"failures.json").write_text(json.dumps(errors,indent=2)+"\n")
        if runs:comparison_plot(runs,output/case/"comparison.png")
    generate_report(output,rows,errors,declaration)
    return rows,errors


def generate_report(output,rows,errors,declaration):
    grouped={}
    for row in rows:grouped.setdefault(row["case"],{})[row["controller"]]=row
    lines=["# Full-car simulation validation report","",
           f"Configuration SHA-256: `{declaration['control_sha256']}`. Tuning frozen before this holdout run.","",
           "This is a representative passenger car, not a verified Tesla model. Results are unweighted, simulation-only, and do not establish ISO comfort compliance or vehicle readiness.","",
           "## Scope and information","",
           "Seven coupled mechanical coordinates, 18 states including four actuator forces; prescribed longitudinal/lateral accelerations, not closed-loop steering/braking. Independent force-based RK4 validation plant with unilateral tires, progressive stops, asymmetric spring/damper/tire mismatch, lag and slew limits. Controllers use the nominal linear model. Observer mode uses noisy vertical IMU acceleration (0.035 m/s² std), pitch/roll gyros (0.002 rad/s), four travel sensors (0.5 mm), four force transducers (12 N), measured maneuver accelerations (0.025 m/s²), and current road-height perception (0.5 mm). Road heights assume a common vertical reference; this idealized perception channel is required by this observer evaluation and is not a demonstrated vehicle sensor pipeline. No future road preview; current measured road and maneuver inputs are held over the MPC horizon. All controller comparisons share seeds, plant, sampling and sensing.","",
           "Force authority is supplied explicitly, not autonomously diagnosed. Fixed and aware MPC have identical cost, sensing and fallback. Soft travel/load constraints and predicted normalized slacks are logged. Contact loss and stroke violations are independently evaluated at integration substeps. Tire loss cases are flagged; they remain in every table.","",
           "## Predeclared acceptance","",
           "Primary objectives: reduce CG/seat acceleration RMS and pitch/roll RMS relative to passive. Safety: zero contact-loss events and zero time outside ±80 mm hardware stroke; body slope within 0.12 rad and stroke within 150 mm for the declared model envelope. Conflicting objectives are reported individually. These are project assumptions, not regulatory limits.","",
           "## Measured holdout results","",
           "| Case | Controller | CG RMS m/s² | Seat RMS m/s² | Pitch RMS ° | Roll RMS ° | Stroke mm | Min tire N | Contact events | Travel exceed s | QP failures | Fallback samples | Safety |",
           "|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|"]
    for row in rows:
        lines.append(f"| {row['case']} | {row['controller']} | {row['cg_accel_rms']:.3f} | {row['seat_accel_rms']:.3f} | {row['pitch_rms_deg']:.3f} | {row['roll_rms_deg']:.3f} | {1000*row['peak_travel_m']:.1f} | {row['min_normal_N']:.0f} | {row['contact_events']} | {row['travel_violation_corner_seconds']:.3f} | {row['solver_failures']} | {row['fallback_samples']} | {'pass' if row['accepted_safety'] else 'FAIL'} |")
    lines.extend(["","## Improvements and regressions","",
                  "Positive percentages mean a reduction relative to passive; negative values are regressions. No failed or stress cases are removed.","",
                  "| Case | Aware MPC CG % | Seat % | Pitch % | Roll % | Tire-load RMS % |",
                  "|---|---:|---:|---:|---:|---:|"])
    for case,rs in grouped.items():
        if "passive" not in rs or "mpc" not in rs:continue
        a,b=rs["passive"],rs["mpc"]
        values=[]
        for key in ("cg_accel_rms","seat_accel_rms","pitch_rms_deg","roll_rms_deg"):
            values.append("n/a" if abs(a[key])<1e-8 else f"{100*(1-b[key]/a[key]):.1f}")
        keys=[c+"_load_rms_dynamic_N" for c in ("FL","FR","RL","RR")]
        values.append(f"{100*(1-np.mean([b[k] for k in keys])/np.mean([a[k] for k in keys])):.1f}")
        lines.append(f"| {case} | "+" | ".join(values)+" |")
    lines.extend(["","## Capability-update ablation","",
                  "| Case | MPC variant | Seat RMS | Roll RMS ° | Absolute mechanical work J | Solver failures |",
                  "|---|---|---:|---:|---:|---:|"])
    for case,rs in grouped.items():
        if "capability" in case:
            for c in ("mpc_fixed","mpc"):
                if c in rs:
                    r=rs[c];lines.append(f"| {case} | {c} | {r['seat_accel_rms']:.3f} | {r['roll_rms_deg']:.3f} | {r['absolute_work_J']:.1f} | {r['solver_failures']} |")
    active=[r for r in rows if r["controller"].startswith("mpc")]
    lines.extend(["","## Numerical cost and reproducibility","",
                  f"{len(rows)} completed runs; {len(errors)} execution failures. Worst observed MPC solve time: {max((r['solver_ms_peak'] for r in active),default=0):.2f} ms. Times are host observations, not real-time guarantees. Each run records host, dependency versions, nominal/plant parameters, seed, weights and source revision in metadata.json. Complete per-corner metrics, constraint violations, signed/absolute work, solver iterations and estimation error are in metrics.csv.","",
                  "Physics, upstream-equation, observer, constraint, convergence and fallback checks are recorded separately in test_results.xml / verification.json when generated. Run the test command in README to reproduce them.","",
                  "## Execution failures","",json.dumps(errors,indent=2) if errors else "None.","",
                  "## Limits and remaining work","",
                  "Model uses small-angle body slopes, diagonal inertias (neglecting payload cross inertias), vertical wheel motion and linear suspension away from stops. No yaw/slip, chassis flexibility, pitch/roll center kinematics, tire enveloping, tire damping or electrical actuator model. Inertial moments use sprung CG height relative to a declared ground reference; unsprung lateral load transfer is omitted. Fixed load-sharing rule and static spring preload are recomputed for payload; the fixed hardware stroke reference consumes additional travel. Seat acceleration includes first-order rotational contributions; centripetal terms are omitted consistently with small-angle kinematics. Mechanical work is not battery energy. Observer assumes force transducers, current road-height perception with a common vertical reference, and known nominal actuator dynamics; no autonomous fault detection is claimed. Simulated IMU drift/bias is not yet modeled. Road spectrum is a declared band-limited synthetic spectrum, not an ISO road classification. Optional MathWorks 14DOF cross-validation is not completed.",""])
    (Path(output)/"validation_report.md").write_text("\n".join(lines))
