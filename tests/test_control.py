import numpy as np
from suspension.model import Vehicle,LinearModel,Plant
from suspension.controllers import make_controller
from suspension.observer import Observer
from suspension.roads import Scenario
from suspension.simulation import simulate,save_run,load_run


def test_observer_observability_and_stable_error_dynamics():
    obs=Observer(LinearModel(Vehicle.load()));result=obs.observability()
    assert result["rank"]==18
    assert result["closed_loop_spectral_radius"]<1
    for _ in range(5):obs.update(np.zeros(11));obs.predict(np.zeros(4),np.zeros(6))
    assert obs.valid
    obs.update(np.full(11,np.nan));assert not obs.valid
    for _ in range(3):obs.update(np.zeros(11));obs.predict(np.zeros(4),np.zeros(6))
    assert obs.valid


def test_lqr_stable_and_controller_cap_slew_fallback():
    p=Vehicle.load();m=LinearModel(p);lqr=make_controller("lqr",m)
    assert max(abs(np.linalg.eigvals(m.An-m.Bn@lqr.gain)))<1
    for name in ("lqr","mpc"):
        c=make_controller(name,m);cap=np.array([500,2000,2000,2000.])
        x=np.zeros(18);x[1]=.015;x[2]=.01
        previous=np.zeros(4)
        for _ in range(8):
            u=c.command(x,np.zeros(6),cap)
            assert np.all(abs(u)<=cap+1e-6)
            assert np.max(abs(u-previous))<=p.slew_limit*m.dt+1e-6
            previous=u.copy()
        for _ in range(5):
            u=c.command(np.full(18,np.nan),np.zeros(6),cap,valid=False)
            assert np.all(np.isfinite(u));assert np.max(abs(u-previous))<=p.slew_limit*m.dt+1e-6
            previous=u.copy()
        np.testing.assert_allclose(u,0,atol=1e-6)
        assert np.all(np.isfinite(c.command(np.zeros(18),np.full(6,np.nan),cap)))


def test_actuator_delivered_limits_lag_reactions_and_slew():
    p=Vehicle.load();plant=Plant(p);s=Scenario("brake");x=np.zeros(18)
    u=np.ones(4)*2000
    for k in range(100):
        xn=plant.step(x,u,k*.002,.002,s)
        assert np.max(abs(xn[14:]-x[14:]))/.002<=p.slew_limit+1e-6
        assert np.all(abs(xn[14:])<=2000)
        x=xn
    assert np.all(x[14:]>1900)


def test_sensor_dropout_fallback_and_serialization(tmp_path):
    s=Scenario("sensor_failure",seed=3,duration=3.2)
    a,m,z=simulate(s,"mpc","observer")
    invalid=(a["time"]>=2.5)&(a["time"]<2.8)
    assert np.all(a["fallback"][invalid])
    assert not np.any(a["valid_estimate"][invalid])
    assert np.all(a["valid_estimate"][-5:])
    assert z["capability_violation_N"]==0
    assert np.all(np.isfinite(a["command"]))
    save_run(tmp_path,a,m,z);b,n,q=load_run(tmp_path)
    np.testing.assert_array_equal(a["state"],b["state"])
    assert n["scenario"]==m["scenario"] and q==z


def test_forced_solver_failure_is_reported_and_bounded():
    s=Scenario("solver_failure",seed=3,duration=3.)
    a,m,z=simulate(s,"mpc","ideal")
    fail=(a["time"]>=2.5)&(a["time"]<2.7)
    assert np.all(a["fallback"][fail])
    assert z["solver_failures"]>=9
    assert np.max(abs(np.diff(a["command"],axis=0)))<=600+1e-6


def test_seed_reproducibility_and_baseline_nominal_comfort():
    s=Scenario("bump",seed=4,duration=3)
    passive=simulate(s,"passive","ideal")
    repeat=simulate(s,"passive","ideal")
    np.testing.assert_array_equal(passive[0]["state"],repeat[0]["state"])
    active=simulate(s,"mpc","ideal")
    # A predeclared development regression protects comfort from gross regressions.
    assert active[2]["cg_accel_rms"]<passive[2]["cg_accel_rms"]
    assert active[2]["contact_events"]==0
    assert active[2]["travel_violation_corner_seconds"]==0
    assert active[2]["solver_failures"]==0
