"""Export numerical verification evidence without relying on a visual plot."""
import json
from pathlib import Path
import sys
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from suspension.model import Vehicle,LinearModel,Plant
from suspension.observer import Observer
from suspension.roads import Scenario
from suspension.simulation import simulate


def main():
    p=Vehicle.load();m=LinearModel(p);s=Scenario("combined",seed=7,duration=4)
    # Identical sampled controller with progressively refined plant integration.
    runs=[simulate(s,"mpc","observer",plant_params=p.mismatch(),substep=dt) for dt in (.004,.002,.001)]
    quantities=[r[2]["cg_accel_rms"] for r in runs]
    fine_relative=abs(quantities[1]-quantities[2])/quantities[2]
    output={"observer":Observer(m).observability(),"integration_steps":[.004,.002,.001],
            "cg_accel_rms":quantities,"relative_difference_2ms_vs_1ms":fine_relative,
            "converged_below_0.5_percent":bool(fine_relative<.005),
            "static_load_balance_error":(p.H.T@p.sprung_load-np.array([p.ms*9.81,0,0])).tolist(),
            "full_car_development_case":runs[1][2],"host":runs[1][1]["host"]}
    Path("results").mkdir(exist_ok=True)
    Path("results/verification.json").write_text(json.dumps(output,indent=2)+"\n")
    print(json.dumps({k:v for k,v in output.items() if k not in ("host","full_car_development_case")},indent=2))
    if not output["converged_below_0.5_percent"]:raise SystemExit(1)


if __name__=="__main__":main()
