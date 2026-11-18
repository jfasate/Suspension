"""Repeatable closed-loop experiments with separate truth, measurements, estimates."""
from dataclasses import asdict
import importlib.metadata
import hashlib
import json
from pathlib import Path
import platform
import subprocess
import time
import numpy as np
from .model import Vehicle, LinearModel, Plant, CORNER, SCALE, ROOT
from .controllers import make_controller, control_config
from .observer import Observer


def rms(a,axis=None):return np.sqrt(np.mean(np.asarray(a)**2,axis=axis))


def environment():
    versions={n:importlib.metadata.version(n) for n in ("numpy","scipy","osqp","matplotlib","Pillow")}
    try:
        revision=subprocess.check_output(["git","rev-parse","HEAD"],cwd=ROOT,stderr=subprocess.DEVNULL,text=True).strip()
        dirty=bool(subprocess.check_output(["git","status","--porcelain"],cwd=ROOT,text=True).strip())
    except subprocess.CalledProcessError:
        revision,dirty="uncommitted-workspace",True
    digest=hashlib.sha256()
    sources=sorted((ROOT/"suspension").glob("*.py"))+sorted((ROOT/"config").glob("*.json"))
    sources.extend([ROOT/"pyproject.toml",ROOT/"requirements.txt"])
    for path in sources:
        digest.update(str(path.relative_to(ROOT)).encode());digest.update(path.read_bytes())
    return {"python":platform.python_version(),"platform":platform.platform(),"source_sha256":digest.hexdigest(),
            "processor":platform.processor(),"dependencies":versions,"revision":revision,"dirty":dirty,
            "upstream_revision":"098f9fdbc5c311d64110f1da30d791644c9f4a77"}


def simulate(scenario,controller="passive",sensing="ideal",plant_params=None,
             nominal=None,nonlinear=True,substep=.002,cfg=None):
    nominal=nominal or Vehicle.load();p=plant_params or nominal
    cfg=cfg or control_config();dt=cfg["dt"]
    if sensing not in ("ideal","observer"):
        raise ValueError("sensing must be ideal or observer")
    if substep<=0 or abs(dt/substep-round(dt/substep))>1e-8:
        raise ValueError("integration substep must divide controller dt")
    m=LinearModel(nominal,dt);plant=Plant(p,nonlinear,reference=nominal)
    ctrl=make_controller(controller,m,cfg);obs=Observer(m)
    rng=np.random.default_rng(scenario.seed+100000)
    times=np.arange(round(scenario.duration/dt)+1)*dt
    x=np.zeros(18)
    log={k:[] for k in ("state","estimate","measurement","measured_disturbance","command","capability","disturbance",
                        "acceleration","travel","stroke","normal_load","power","valid_estimate",
                        "solver_time","solver_iterations","solver_failed","fallback","slack","force_rate")}
    statuses={}
    tstart=time.perf_counter()
    # Track safety peaks at integration substeps, not just controller samples.
    fine={"peak_travel":np.zeros(4),"min_load":p.normal_static.copy(),"travel_duration":np.zeros(4),
          "contact_duration":np.zeros(4),"contact_events":np.zeros(4,dtype=int),
          "contact_active":np.zeros(4,dtype=bool),"force_peak":np.zeros(4),"force_rate_peak":np.zeros(4),
          "work":np.zeros(4),"absolute_work":np.zeros(4),"saturation_duration":np.zeros(4),
          "bound_override_peak":0.}
    last_command=np.zeros(4)
    nsub=round(dt/substep)
    grid=np.arange((len(times)-1)*nsub*2+1)*substep/2
    input_w=scenario.disturbance(grid,p);input_cap=scenario.capability(grid,p)
    for k,t in enumerate(times):
        w=input_w[k*nsub*2];cap=input_cap[k*nsub*2]
        rhs=plant.rhs(x,last_command,w,cap)
        measurement=obs.measure(x,rhs[7],p,rng)
        # Same random stream and input sensing across controllers.
        measured_axy=w[4:]+rng.normal(0,.025,2) if sensing=="observer" else w[4:]
        measured_road=w[:4]+rng.normal(0,.0005,4) if sensing=="observer" else w[:4]
        if (scenario.invalid_sensor or scenario.name=="sensor_failure") and 2.5<=t<2.8:
            measurement[:]=np.nan
        if sensing=="observer":
            estimate=obs.update(measurement)
            valid=obs.valid
            # Declared current road-height perception channel; no future preview.
            feedback_w=np.r_[measured_road,measured_axy]
        else:
            estimate=x.copy();valid=True
            feedback_w=np.r_[measured_road,measured_axy]
        fail=(scenario.forced_solver_failure or scenario.name=="solver_failure") and 2.5<=t<2.7
        u=ctrl.command(estimate,feedback_w,cap,valid=valid,force_failure=fail)
        status=ctrl.info.get("status","");statuses[status]=statuses.get(status,0)+1
        travel,vel,stroke,body_force,normal=plant.signals(x,w[:4])
        rhs=plant.rhs(x,u,w,cap)
        values={"state":x.copy(),"estimate":estimate.copy(),"measurement":measurement,
                "command":u,"capability":cap,"disturbance":w,"measured_disturbance":feedback_w,"acceleration":rhs[7:10],
                "travel":travel,"stroke":stroke,"normal_load":normal,"power":x[14:]*vel,
                "valid_estimate":valid,"solver_time":ctrl.info.get("solve_time",0.),
                "solver_iterations":ctrl.info.get("iterations",0),
                "solver_failed":bool(ctrl.info.get("failed",False) and not ctrl.info.get("fallback",False)),
                "fallback":bool(ctrl.info.get("fallback",False)),
                "slack":ctrl.info.get("slack",0.),"force_rate":rhs[14:]}
        # Solver failures and invalid-sensing fallback are reported separately.
        values["solver_failed"]=bool(controller.startswith("mpc") and valid and ctrl.info.get("failed",False))
        for key,value in values.items():log[key].append(value)
        if k==len(times)-1:break
        for j in range(nsub):
            tt=t+j*substep
            ii=(k*nsub+j)*2
            ww=input_w[ii];cc=input_cap[ii]
            tr,ve,st,fb,nl=plant.signals(x,ww[:4])
            fine["peak_travel"]=np.maximum(fine["peak_travel"],np.abs(st))
            fine["min_load"]=np.minimum(fine["min_load"],nl)
            fine["travel_duration"]+=(np.abs(st)>p.travel_limit)*substep
            contact=nl<=1e-6
            fine["contact_events"]+=(contact & ~fine["contact_active"])
            fine["contact_active"]=contact
            fine["contact_duration"]+=contact*substep
            fine["saturation_duration"]+=(np.abs(x[14:])>=cc*.99)*substep
            fine["work"]+=x[14:]*ve*substep
            fine["absolute_work"]+=np.abs(x[14:]*ve)*substep
            fine["force_peak"]=np.maximum(fine["force_peak"],np.abs(x[14:]))
            old=x.copy();x=plant.step(x,u,tt,substep,scenario,(input_w[ii:ii+3],input_cap[ii:ii+3]))
            rate=np.abs(x[14:]-old[14:])/substep
            fine["force_rate_peak"]=np.maximum(fine["force_rate_peak"],rate)
            fine["bound_override_peak"]=max(fine["bound_override_peak"],float(np.max(np.abs(x[14:])-input_cap[ii+2])))
            if not np.all(np.isfinite(x)) or np.max(np.abs(x[:7]))>2:
                raise RuntimeError(f"Plant left numerical/model envelope at {tt:.4f}s")
        if sensing=="observer":obs.predict(u,feedback_w)
        last_command=u
    arrays={k:np.asarray(v) for k,v in log.items()};arrays["time"]=times
    metadata={"scenario":asdict(scenario),"controller":controller,"sensing":sensing,
              "plant":p.to_dict(),"nominal":nominal.to_dict(),"control":cfg,"nonlinear":nonlinear,
              "integration_dt":substep,"host":environment(),"wall_seconds":time.perf_counter()-tstart,
              "observer":obs.observability(),"solver_status_counts":statuses,
              "information":"No future road preview. Observer mode: noisy IMU, gyros, suspension travel, force sensors and current road-height perception (0.5 mm std, common vertical reference assumed).",
              "fine_metrics":{key:(v.tolist() if isinstance(v,np.ndarray) else v) for key,v in fine.items()}}
    metrics=compute_metrics(arrays,metadata)
    return arrays,metadata,metrics


def compute_metrics(a,meta):
    p=Vehicle(**meta["plant"]);cfg=meta["control"];fine=meta["fine_metrics"]
    # A fixed physical seat position, shifted relative to the loaded CG.
    nominal=Vehicle(**meta["nominal"])
    seat_xy=[cfg["seat_xy"][0]+p.a-nominal.a,cfg["seat_xy"][1]+nominal.cg_y-p.cg_y]
    seat=np.array([1,*seat_xy]);acc=a["acceleration"];x=a["state"]
    command_rate=np.diff(a["command"],axis=0)/cfg["dt"]
    metrics={"scenario":meta["scenario"]["name"],"seed":meta["scenario"]["seed"],
             "speed_mps":meta["scenario"]["speed"],"mass_kg":p.ms,"sensing":meta["sensing"],
             "controller":meta["controller"],"cg_accel_rms":float(rms(acc[:,0])),
             "cg_accel_peak":float(np.max(np.abs(acc[:,0]))),
             "seat_accel_rms":float(rms(acc@seat)),"seat_accel_peak":float(np.max(np.abs(acc@seat))),
             "pitch_rms_deg":float(np.rad2deg(rms(x[:,1]))),"roll_rms_deg":float(np.rad2deg(rms(x[:,2]))),
             "pitch_peak_deg":float(np.rad2deg(max(abs(x[:,1])))),"roll_peak_deg":float(np.rad2deg(max(abs(x[:,2])))),
             "pitch_accel_rms":float(rms(acc[:,1])),"roll_accel_rms":float(rms(acc[:,2])),
             "peak_travel_m":max(fine["peak_travel"]),"min_normal_N":min(fine["min_load"]),
             "contact_events":sum(fine["contact_events"]),"travel_violation_corner_seconds":sum(fine["travel_duration"]),
             "absolute_work_J":sum(fine["absolute_work"]),"signed_work_J":sum(fine["work"]),
             "solver_failures":int(sum(a["solver_failed"])),"fallback_samples":int(sum(a["fallback"])),
             "invalid_estimate_samples":int(sum(~a["valid_estimate"].astype(bool))),
             "solver_ms_mean":float(np.mean(a["solver_time"])*1000),"solver_ms_peak":float(max(a["solver_time"])*1000),
             "solver_iterations_peak":int(max(a["solver_iterations"])),"predicted_slack_peak":float(max(a["slack"])),
             "estimate_normalized_rmse":float(rms((a["estimate"]-x)/SCALE)),
             "force_rate_peak_Nps":max(fine["force_rate_peak"]),
             "command_rate_peak_Nps":float(np.max(abs(command_rate))),
             "capability_violation_N":max(0.,fine["bound_override_peak"]),
             "model_envelope_exceeded":bool(np.any(abs(x[:,1:3])>.12) or max(fine["peak_travel"])>.15)}
    # Recovery after smooth maneuvers end (5.2s): require final attitude within 0.2 deg
    # and both body angular rates within 0.2 deg/s for the remainder of the record.
    after=np.where(a["time"]>=5.2)[0]
    settled=(np.max(abs(x[:,1:3]),axis=1)<np.deg2rad(.2))&(np.max(abs(x[:,8:10]),axis=1)<np.deg2rad(.2))
    metrics["attitude_recovery_s"]="not_settled"
    for i in after:
        if np.all(settled[i:]):metrics["attitude_recovery_s"]=float(a["time"][i]-5.2);break
    for j,c in enumerate(CORNER):
        for name,value in (("travel_peak_m",fine["peak_travel"][j]),("travel_rms_m",rms(a["stroke"][:,j])),
                           ("travel_violation_s",fine["travel_duration"][j]),
                           ("load_rms_dynamic_N",rms(a["normal_load"][:,j]-p.normal_static[j])),
                           ("min_load_N",fine["min_load"][j]),("contact_events",fine["contact_events"][j]),
                           ("contact_loss_s",fine["contact_duration"][j]),
                           ("force_peak_N",fine["force_peak"][j]),("force_rms_N",rms(x[:,14+j])),
                           ("force_rate_peak_Nps",fine["force_rate_peak"][j]),
                           ("saturation_s",fine["saturation_duration"][j]),
                           ("signed_work_J",fine["work"][j]),("absolute_work_J",fine["absolute_work"][j])):
            metrics[f"{c}_{name}"]=float(value)
    metrics["accepted_safety"]=bool(metrics["contact_events"]==0 and metrics["travel_violation_corner_seconds"]==0
                                    and not metrics["model_envelope_exceeded"] and metrics["capability_violation_N"]<1)
    return metrics


def save_run(folder,arrays,metadata,metrics):
    path=Path(folder);path.mkdir(parents=True,exist_ok=True)
    np.savez_compressed(path/"run.npz",**arrays)
    (path/"metadata.json").write_text(json.dumps(metadata,indent=2,allow_nan=False)+"\n")
    (path/"metrics.json").write_text(json.dumps(metrics,indent=2,allow_nan=False)+"\n")


def load_run(folder):
    path=Path(folder)
    with np.load(path/"run.npz",allow_pickle=False) as data:a=dict(data)
    return a,json.loads((path/"metadata.json").read_text()),json.loads((path/"metrics.json").read_text())
