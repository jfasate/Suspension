"""Parameter-driven port of Andac Y. Kaya's MIT full-car equations.

Coordinates: x forward, y left, z up. alpha = front-up slope (-RH pitch),
beta = left-up slope (RH roll). State [q(7), qdot(7), delivered force(4)].
All displacements and forces are perturbations about the loaded equilibrium.
"""
from dataclasses import dataclass, asdict, replace
import json
from pathlib import Path
import numpy as np
from scipy.linalg import expm

ROOT = Path(__file__).resolve().parents[1]
CORNER = ("FL", "FR", "RL", "RR")
SCALE = np.array([.05, .04, .04, .03, .03, .03, .03,
                  .5, .3, .3, .5, .5, .5, .5, 2000., 2000., 2000., 2000.])


@dataclass
class Vehicle:
    ms: float
    pitch_inertia: float
    roll_inertia: float
    a: float
    b: float
    front_track: float
    rear_track: float
    cg_height: float
    cg_y: float
    mu: list
    ks: list
    cs: list
    kt: list
    force_limit: float
    slew_limit: float
    actuator_tau: float
    travel_limit: float
    stop_stiffness: float
    stop_cubic: float

    @classmethod
    def load(cls, path=None):
        p = cls(**json.loads(Path(path or ROOT / "config/vehicle.json").read_text()))
        p.validate()
        return p

    def validate(self):
        for name in ("ms", "pitch_inertia", "roll_inertia", "a", "b", "front_track",
                     "rear_track", "force_limit", "slew_limit", "actuator_tau", "travel_limit"):
            if getattr(self, name) <= 0:
                raise ValueError(f"{name} must be positive")
        for name in ("mu", "ks", "cs", "kt"):
            if np.shape(getattr(self, name)) != (4,) or np.any(np.asarray(getattr(self, name)) <= 0):
                raise ValueError(f"{name} must have four positive corner values")
        if np.any(self.sprung_load <= 0):
            raise ValueError("CG outside supported static load envelope")

    @property
    def H(self):
        return np.array([[1, self.a, self.front_track/2-self.cg_y],
                         [1, self.a, -self.front_track/2-self.cg_y],
                         [1, -self.b, self.rear_track/2-self.cg_y],
                         [1, -self.b, -self.rear_track/2-self.cg_y]])

    @property
    def G(self):
        return np.column_stack((self.H, -np.eye(4)))

    @property
    def mass(self):
        return np.array([self.ms, self.pitch_inertia, self.roll_inertia, *self.mu])

    @property
    def sprung_load(self):
        # Axle shares by CG x; side shares on each axle by CG y / that axle's track.
        # This explicitly selects a solution to the statically indeterminate load split.
        front = self.ms*9.81*self.b/(self.a+self.b)
        rear = self.ms*9.81*self.a/(self.a+self.b)
        return np.array([front*(.5+self.cg_y/self.front_track), front*(.5-self.cg_y/self.front_track),
                         rear*(.5+self.cg_y/self.rear_track), rear*(.5-self.cg_y/self.rear_track)])

    @property
    def normal_static(self):
        return self.sprung_load + np.array(self.mu)*9.81

    @property
    def equilibrium_travel(self):
        return -self.sprung_load / np.array(self.ks)

    def payload(self, mass=200., x=-.35, y=.12, height=.75):
        """Point payload; fixed axle/track positions, parallel-axis inertias.

        Heights and offsets measured from original CG ground projection.
        Cross inertias neglected explicitly in this reduced diagonal-inertia model.
        """
        if mass < 0:
            raise ValueError("payload must be nonnegative")
        total = self.ms + mass
        dx, dy = mass*x/total, mass*y/total
        dz = mass*(height-self.cg_height)/total
        reduced_mass = self.ms*mass/total
        p = replace(self, ms=total, a=self.a-dx, b=self.b+dx, cg_y=self.cg_y+dy,
                    cg_height=self.cg_height+dz,
                    pitch_inertia=self.pitch_inertia+reduced_mass*(x*x+(height-self.cg_height)**2),
                    roll_inertia=self.roll_inertia+reduced_mass*(y*y+(height-self.cg_height)**2))
        p.validate()
        return p

    def mismatch(self):
        return replace(self, ks=(np.array(self.ks)*[1.12,.94,1.08,.90]).tolist(),
                       cs=(np.array(self.cs)*[.85,1.15,.92,1.10]).tolist(),
                       kt=(np.array(self.kt)*[1.06,.97,1.04,.95]).tolist(),
                       actuator_tau=self.actuator_tau*1.25)

    def to_dict(self):
        return asdict(self)


class LinearModel:
    def __init__(self, p, dt=.02):
        self.p, self.dt = p, dt
        G, mass = p.G, p.mass
        K = G.T @ np.diag(p.ks) @ G
        K[3:,3:] += np.diag(p.kt)
        C = G.T @ np.diag(p.cs) @ G
        A, B, E = np.zeros((18,18)), np.zeros((18,4)), np.zeros((18,6))
        A[:7,7:14] = np.eye(7)
        A[7:14,:7] = -K / mass[:,None]
        A[7:14,7:14] = -C / mass[:,None]
        A[7:14,14:] = G.T / mass[:,None]
        A[14:,14:] = -np.eye(4)/p.actuator_tau
        B[14:] = np.eye(4)/p.actuator_tau
        E[10:14,:4] = np.diag(np.array(p.kt)/p.mu)
        # Inertial load at CG: forward acceleration -> nose-up; left -> left-up.
        E[8,4] = p.ms*p.cg_height/p.pitch_inertia
        E[9,5] = p.ms*p.cg_height/p.roll_inertia
        self.A, self.B, self.E = A, B, E
        aug = np.zeros((28,28))
        aug[:18,:18], aug[:18,18:22], aug[:18,22:] = A, B, E
        disc = expm(aug*dt)
        self.Ad, self.Bd, self.Ed = disc[:18,:18], disc[:18,18:22], disc[:18,22:]
        self.An = self.Ad * SCALE[None,:] / SCALE[:,None]
        self.Bn = self.Bd * p.force_limit / SCALE[:,None]
        self.En = self.Ed / SCALE[:,None]

    def output_cost(self, cfg):
        """Dimensionless output penalties: y = C*x_normalized + D*disturbance."""
        w = cfg["weights"]
        rows, ds = [], []
        def add(c, d, weight):
            rows.append(np.atleast_2d(c)*np.sqrt(weight))
            ds.append(np.atleast_2d(d)*np.sqrt(weight))
        add(self.A[7]*SCALE/2, self.E[7]/2, w["cg_accel"])
        seat = np.array([1., *cfg["seat_xy"]])
        add(seat@self.A[7:10]*SCALE/2, seat@self.E[7:10]/2, w["seat_accel"])
        add(np.eye(18)[1:3]*SCALE/0.03, np.zeros((2,6)), w["attitude"])
        add(np.eye(18)[8:10]*SCALE/.2, np.zeros((2,6)), w["angular_rate"])
        T = np.zeros((4,18)); T[:,:7] = self.p.G
        add(T*SCALE/self.p.travel_limit, np.zeros((4,6)), w["travel"])
        N = np.zeros((4,18)); N[:,3:7] = -np.diag(np.array(self.p.kt)/self.p.normal_static)
        D = np.zeros((4,6)); D[:,:4] = np.diag(np.array(self.p.kt)/self.p.normal_static)
        add(N*SCALE, D, w["tire_load"])
        return np.vstack(rows), np.vstack(ds)


class Plant:
    """Independent force-based validation RHS: unilateral tires and progressive stops."""
    def __init__(self, p, nonlinear=True, reference=None):
        self.p, self.nonlinear = p, nonlinear
        # Fixed hardware stroke relative to nominal equilibrium; payload consumes stroke.
        self.travel_offset = p.equilibrium_travel - (reference or p).equilibrium_travel

    def signals(self, x, road):
        p = self.p
        travel = p.G@x[:7]
        velocity = p.G@x[7:14]
        stroke = travel+self.travel_offset
        stop = np.zeros(4)
        if self.nonlinear:
            beyond = np.maximum(np.abs(stroke)-p.travel_limit, 0)
            stop = -np.sign(stroke)*(p.stop_stiffness*beyond+p.stop_cubic*beyond**3)
        body_force = -np.array(p.ks)*travel - np.array(p.cs)*velocity + x[14:]+stop
        raw_normal = p.normal_static + np.array(p.kt)*(road-x[3:7])
        normal = np.maximum(0, raw_normal) if self.nonlinear else raw_normal
        return travel, velocity, stroke, body_force, normal

    def rhs(self, x, command, disturbance, capability):
        p = self.p
        travel, velocity, stroke, f, normal = self.signals(x, disturbance[:4])
        external = np.array([0, p.ms*p.cg_height*disturbance[4], p.ms*p.cg_height*disturbance[5]])
        body_accel = (p.H.T@f+external)/p.mass[:3]
        wheel_accel = (-f+normal-p.normal_static)/np.array(p.mu)
        target = np.clip(command, -capability, capability)
        force_rate = np.clip((target-x[14:])/p.actuator_tau, -p.slew_limit, p.slew_limit)
        return np.r_[x[7:14],body_accel,wheel_accel,force_rate]

    def step(self, x, command, t, dt, scenario, inputs=None):
        # Time-varying road/maneuvers at RK4 substages, no zero-order road steps.
        if inputs is None:
            ww=scenario.disturbance(np.array([t,t+dt/2,t+dt]),self.p)
            cc=scenario.capability(np.array([t,t+dt/2,t+dt]),self.p)
        else:
            ww,cc=inputs
        k1 = self.rhs(x,command,ww[0],cc[0])
        k2 = self.rhs(x+dt*k1/2,command,ww[1],cc[1])
        k3 = self.rhs(x+dt*k2/2,command,ww[1],cc[1])
        k4 = self.rhs(x+dt*k3,command,ww[2],cc[2])
        xn = x+dt*(k1+2*k2+2*k3+k4)/6
        cap = cc[2]
        xn[14:] = np.clip(xn[14:], -cap, cap)
        return xn

    def energy(self,x):
        return .5*np.sum(self.p.mass*x[7:14]**2)+.5*np.sum(np.array(self.p.ks)*(self.p.G@x[:7])**2)+.5*np.sum(np.array(self.p.kt)*x[3:7]**2)
