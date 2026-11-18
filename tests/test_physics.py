import json
from pathlib import Path
import numpy as np
import pytest
from scipy.linalg import expm
from suspension.model import Vehicle,LinearModel,Plant
from suspension.roads import Scenario


def test_equilibrium_dimensions_static_balance_and_payload():
    for p in (Vehicle.load(),Vehicle.load().payload(250.,-.4,.15,.8)):
        p.validate();m=LinearModel(p);plant=Plant(p)
        assert m.A.shape==(18,18) and m.B.shape==(18,4) and m.E.shape==(18,6)
        np.testing.assert_allclose(p.H.T@p.sprung_load,[p.ms*9.81,0,0],atol=1e-10)
        np.testing.assert_allclose(plant.rhs(np.zeros(18),np.zeros(4),np.zeros(6),np.ones(4)*2000),0,atol=1e-12)
        np.testing.assert_allclose(plant.signals(np.zeros(18),np.zeros(4))[-1],p.normal_static)
    loaded=Vehicle.load().payload()
    assert loaded.pitch_inertia>Vehicle.load().pitch_inertia
    assert loaded.roll_inertia>Vehicle.load().roll_inertia
    assert np.any(Plant(loaded,reference=Vehicle.load()).travel_offset!=0)


def test_actuator_equal_opposite_reaction_and_moment_geometry():
    p=Vehicle.load();plant=Plant(p,False)
    x=np.zeros(18);x[14:]=[100,200,-120,30]
    rhs=plant.rhs(x,x[14:],np.zeros(6),np.ones(4)*2000)
    np.testing.assert_allclose(rhs[7:10]*p.mass[:3],p.H.T@x[14:])
    np.testing.assert_allclose(rhs[10:14]*p.mu,-x[14:])
    assert abs(rhs[7]*p.ms+np.dot(rhs[10:14],p.mu))<1e-10


def test_upstream_simulink_equations_match_port():
    fixtures=json.loads((Path(__file__).parent/"upstream_equation_fixtures.json").read_text())
    m=LinearModel(Vehicle.load());p=Plant(m.p,False)
    for case in fixtures["cases"]:
        x=np.array(case["state"]);w=np.r_[case["road"],0,0]
        np.testing.assert_allclose((m.A@x+m.E@w)[7:14],case["acceleration"],atol=1e-12)
        np.testing.assert_allclose(p.rhs(x,x[14:],w,np.ones(4)*2000)[7:14],case["acceleration"],atol=1e-12)


def test_left_right_symmetry_and_roll_pitch_signs():
    p=Vehicle.load();m=LinearModel(p)
    t=np.linspace(0,3,61);w=Scenario("bump").road(t,p)
    np.testing.assert_allclose(w[:,0],w[:,1]);np.testing.assert_allclose(w[:,2],w[:,3])
    x=np.zeros(18);x[3]=x[5]=.01
    assert (m.A@x)[9]>0 # left road/wheels lift -> left-up roll
    x=np.zeros(18);x[3:5]=.01
    assert (m.A@x)[8]>0 # front lift -> front-up pitch
    x=np.zeros(18);x[3:7]=.01
    assert abs((m.A@x)[9])<1e-12


def test_passive_energy_decreases_and_matches_exact_solution():
    p=Vehicle.load();plant=Plant(p,False);m=LinearModel(p)
    s=Scenario("brake");x=np.zeros(18);x[0]=.01;x[1]=.005;x[2]=.004
    x0=x.copy();energies=[plant.energy(x)]
    # zero input uses explicit cached flat-road excitation, avoids later braking.
    inputs=(np.zeros((3,6)),np.full((3,4),2000.))
    for k in range(500):
        x=plant.step(x,np.zeros(4),k*.001,.001,s,inputs);energies.append(plant.energy(x))
    assert np.max(np.diff(energies))<1e-10
    assert energies[-1]<energies[0]*.4
    np.testing.assert_allclose(x,expm(m.A*.5)@x0,atol=2e-8)


def test_nonlinear_small_signal_agreement_contact_and_stops():
    p=Vehicle.load();lin=Plant(p,False);non=Plant(p,True)
    x=np.arange(18,dtype=float)*1e-5
    np.testing.assert_allclose(non.rhs(x,np.zeros(4),np.zeros(6),np.ones(4)*2000),lin.rhs(x,np.zeros(4),np.zeros(6),np.ones(4)*2000),atol=1e-12)
    x=np.zeros(18);x[3]=.1
    assert non.signals(x,np.zeros(4))[-1][0]==0
    assert lin.signals(x,np.zeros(4))[-1][0]<0
    x=np.zeros(18);x[0]=.12
    assert non.signals(x,np.zeros(4))[3][0]<lin.signals(x,np.zeros(4))[3][0]
    assert non.signals(x,np.zeros(4))[0][0]==.12 # state/travel is not clamped


def test_spatial_tracks_rear_delay_and_variable_speed():
    p=Vehicle.load();s=Scenario("rough",seed=22)
    t=np.linspace(1,4,31)
    np.testing.assert_allclose(s.road(t,p)[:,2:],s.road(t-(p.a+p.b)/s.speed,p)[:,:2],atol=1e-14)
    v=Scenario("rough",seed=22,variable_speed=True)
    np.testing.assert_allclose(v.road(t,p)[:,2:],v.tracks(v.distance(t)-p.a-p.b))
    # Exact shared-component construction implies configured ensemble correlation.
    road=s.tracks(np.linspace(0,3000,50000))
    assert np.corrcoef(road.T)[0,1]==pytest.approx(s.correlation,abs=.13)
    assert np.all(np.diff(v.distance(t))>0)


def test_signed_maneuver_load_transfer():
    p=Vehicle.load();m=LinearModel(p)
    assert m.E[8,4]>0 and m.E[9,5]>0
    # Quasi-static braking must increase front axle normal loads; left acceleration loads right.
    w=np.array([0,0,0,0,-2,2]);eq=np.linalg.solve(m.A[:14,:14],-m.E[:14]@w)
    x=np.r_[eq,np.zeros(4)]
    normals=Plant(p).signals(x,np.zeros(4))[-1]
    assert sum(normals[:2])>sum(p.normal_static[:2])
    assert sum(normals[[1,3]])>sum(normals[[0,2]])


def test_rk4_convergence():
    p=Vehicle.load();plant=Plant(p,False);s=Scenario("one_side")
    def integrate(dt):
        x=np.zeros(18)
        for k in range(round(1.2/dt)):x=plant.step(x,np.zeros(4),k*dt,dt,s)
        return x
    reference=integrate(.0005)
    coarse=np.linalg.norm(integrate(.004)-reference)
    fine=np.linalg.norm(integrate(.002)-reference)
    assert fine<coarse/7
