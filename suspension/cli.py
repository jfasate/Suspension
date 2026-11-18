import argparse
from pathlib import Path
from .model import Vehicle
from .roads import Scenario,NAMES
from .simulation import simulate,save_run,load_run
from .experiments import CONTROLLERS,benchmark,plant_for,write_csv
from .visualization import comparison_plot,animate_run


def main(argv=None):
    parser=argparse.ArgumentParser(description="Coupled seven-DOF full-car suspension experiments")
    sub=parser.add_subparsers(dest="action",required=True)
    for name in ("run","compare"):
        p=sub.add_parser(name)
        p.add_argument("--scenario",choices=NAMES,default="bump" if name=="run" else "combined")
        p.add_argument("--controller",choices=CONTROLLERS,default="passive")
        p.add_argument("--sensing",choices=("ideal","observer"),default="ideal" if name=="run" else "observer")
        p.add_argument("--plant",choices=("nominal","mismatch","payload"),default="mismatch")
        p.add_argument("--seed",type=int,default=1)
        p.add_argument("--speed",type=float,default=12.)
        p.add_argument("--duration",type=float,default=7.)
        p.add_argument("--substep",type=float,default=.002)
        p.add_argument("--output",type=Path,default=Path("results")/name)
    p=sub.add_parser("benchmark");p.add_argument("--output",type=Path,default=Path("results/holdout"))
    p.add_argument("--sensing",choices=("ideal","observer"),default="observer");p.add_argument("--quick",action="store_true")
    p=sub.add_parser("animate");p.add_argument("run",type=Path);p.add_argument("--output",type=Path,default=Path("results/replay.mp4"))
    p.add_argument("--fps",type=int,default=20);p.add_argument("--scale",type=float,default=4.)
    p=sub.add_parser("view",help="Open the interactive Python suspension viewer")
    p.add_argument("run",type=Path,nargs="?",default=Path("results/comparison"),help="Saved run or directory of controller runs")
    p.add_argument("--controller",choices=CONTROLLERS,default="mpc")
    p.add_argument("--scale",type=float,default=4.,help="Visual motion exaggeration (signals stay in physical units)")
    p.add_argument("--speed",type=float,default=1.,help="Replay speed multiplier")
    p.add_argument("--time",type=float,default=0.,help="Initial simulation time [s]")
    p.add_argument("--snapshot",type=Path,help="Save a still image instead of opening a window")
    args=parser.parse_args(argv)
    if args.action=="view":
        from .viewer import view_replays
        try:
            view_replays(args.run,args.controller,args.scale,args.speed,args.time,args.snapshot)
        except (ValueError,RuntimeError,FileNotFoundError) as exc:
            parser.error(str(exc))
    elif args.action=="benchmark":
        rows,errors=benchmark(args.output,args.sensing,args.quick)
        print(f"Saved {len(rows)} runs to {args.output}; execution failures: {len(errors)}")
        if errors:raise SystemExit(1)
    elif args.action=="animate":
        a,m,_=load_run(args.run);animate_run(a,m,args.output,args.fps,args.scale)
        print(args.output)
    else:
        p=Vehicle.load();scenario=Scenario(args.scenario,args.seed,args.speed,args.duration)
        args.output.mkdir(parents=True,exist_ok=True)
        runs=[]
        for c in CONTROLLERS if args.action=="compare" else (args.controller,):
            run=simulate(scenario,c,args.sensing,plant_params=plant_for(args.plant,p),substep=args.substep)
            save_run(args.output/c,*run);runs.append(run)
            z=run[2]
            print(f"{c}: CG RMS={z['cg_accel_rms']:.3f} m/s²; seat={z['seat_accel_rms']:.3f}; pitch={z['pitch_rms_deg']:.3f}°; roll={z['roll_rms_deg']:.3f}°; safety={z['accepted_safety']}",flush=True)
        comparison_plot(runs,args.output/"comparison.png")
        write_csv([r[2] for r in runs],args.output/"metrics.csv")


if __name__=="__main__":main()
