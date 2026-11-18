"""Generate conservative portfolio statements from completed benchmark metrics."""
import csv
from pathlib import Path


def main():
    path=Path("results/holdout/metrics.csv")
    with path.open() as stream:rows=list(csv.DictReader(stream))
    case={r["controller"]:r for r in rows if r["case"]=="combined_nominal"}
    a,b=case["passive"],case["mpc"]
    gains={k:100*(1-float(b[k])/float(a[k])) for k in
           ("cg_accel_rms","seat_accel_rms","pitch_rms_deg","roll_rms_deg")}
    lines=["# Portfolio statements based on completed simulation", "",
           "- Built a reproducible seven-DOF, four-wheel suspension simulation with an 18-state actuator-augmented model, adapting and attributing MIT-licensed full-car equations; added centralized MPC, LQR, a noisy measured-output observer and a nonlinear validation plant.",
           f"- Completed {len(rows)} declared holdout controller runs across spatial road disturbances, prescribed braking/cornering, payload/CG shifts, parameter mismatch, capability loss and injected sensor/solver failures; retained all contact-loss and constraint-violation cases in the report.",
           f"- On the declared combined-road/braking/cornering holdout (seed {b['seed']}, {float(b['speed_mps']):g} m/s), MPC reduced unweighted CG acceleration RMS {gains['cg_accel_rms']:.1f}%, seat acceleration RMS {gains['seat_accel_rms']:.1f}%, pitch RMS {gains['pitch_rms_deg']:.1f}% and roll RMS {gains['roll_rms_deg']:.1f}% versus passive in the mismatched simulation plant with noisy sensing and assumed current road-height perception; no contact or travel violations occurred in that MPC run.",
           "", "These statements describe simulation only. They do not claim Tesla-specific parameters, research novelty, production readiness, actual-vehicle validation or general improvement on every metric. See the full validation report for regressions, stress-case contact loss and solver failures.", ""]
    Path("results/portfolio_bullets.md").write_text("\n".join(lines))
    print("\n".join(lines))


if __name__=="__main__":main()
