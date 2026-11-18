"""Two spatial tracks: rear wheels follow identical tracks at wheelbase offset."""
from dataclasses import dataclass
import numpy as np


def bump(s, start, width, height):
    phase = (np.asarray(s)-start)/width
    return np.where((phase>=0)&(phase<=1),height*.5*(1-np.cos(2*np.pi*phase)),0.)


@dataclass
class Scenario:
    name: str = "bump"
    seed: int = 1
    speed: float = 12.
    duration: float = 6.
    roughness: float = .003
    correlation: float = .65
    fault: bool = False
    invalid_sensor: bool = False
    forced_solver_failure: bool = False
    variable_speed: bool = False

    def __post_init__(self):
        if self.speed <= 0 or self.duration <= 0 or not 0 <= self.correlation <= 1:
            raise ValueError("positive speed/duration and correlation in [0,1] required")
        rng = np.random.default_rng(self.seed)
        self.freq = np.geomspace(.025,3.0,64) # cycles / metre, band-limited spatial spectrum
        self.phase = rng.uniform(0,2*np.pi,(3,len(self.freq)))
        self.amp = 1/np.sqrt(self.freq)
        self.amp *= self.roughness / np.sqrt(.5*np.sum(self.amp**2))

    def distance(self,t):
        t=np.asarray(t)
        # Monotonic prescribed speed: v(t)=v0*(1+.12*sin(.8*t)).
        return self.speed*(t + (.12/.8)*(1-np.cos(.8*t)) if self.variable_speed else t)

    def tracks(self,s):
        s = np.asarray(s)
        l, r = np.zeros_like(s,dtype=float), np.zeros_like(s,dtype=float)
        if self.name in ("rough", "rough_maneuver", "combined", "fault", "sensor_failure", "solver_failure"):
            waves = np.cos(2*np.pi*s[...,None,None]*self.freq + self.phase)
            components = np.sum(waves*self.amp,axis=-1)
            l = np.sqrt(self.correlation)*components[...,0]+np.sqrt(1-self.correlation)*components[...,1]
            r = np.sqrt(self.correlation)*components[...,0]+np.sqrt(1-self.correlation)*components[...,2]
            # Smooth entry onto finite test road, preserving rear-track identity.
            gate = np.clip(s/5,0,1); gate = gate*gate*(3-2*gate)
            l, r = l*gate,r*gate
        if self.name == "bump":
            l += bump(s,10,2.5,.025); r += bump(s,10,2.5,.025)
        elif self.name == "one_side":
            l += bump(s,10,2.5,.025)
        elif self.name in ("diagonal", "combined", "fault"):
            l += bump(s,10,2.5,.022); r += bump(s,13,2.5,.022)
        elif self.name == "dip":
            l += bump(s,10,2.5,-.020); r += bump(s,10,2.5,-.020)
        elif self.name == "undulations":
            gate = np.clip((s-5)/4,0,1)
            l += .012*np.sin(2*np.pi*s/8)*gate
            r += .012*np.sin(2*np.pi*s/8+.2)*gate
        elif self.name == "sweep":
            gate = np.clip(s/5,0,1)
            phase = 2*np.pi*(.025*s+.0015*s*s)
            l += .006*np.sin(phase)*gate; r += .006*np.sin(phase+.4)*gate
        return np.stack((l,r),axis=-1)

    def road(self,t,p):
        s = self.distance(t)
        front = self.tracks(s)
        rear = self.tracks(s-p.a-p.b)
        return np.concatenate((front,rear),axis=-1)

    def maneuver(self,t):
        t=np.asarray(t)
        # Smooth acceleration histories; no closed-loop planar dynamics implied.
        def pulse(start,end):
            return .5*(np.tanh((t-start)/.22)-np.tanh((t-end)/.22))
        ax,ay = np.zeros_like(t,dtype=float),np.zeros_like(t,dtype=float)
        if self.name in ("brake","combined","rough_maneuver","fault","sensor_failure","solver_failure"):
            ax = -2.5*pulse(1.5,3.3)+1.5*pulse(4.2,5.2)
        if self.name in ("corner","combined","rough_maneuver","fault","sensor_failure","solver_failure"):
            ay = 2.8*pulse(2.,4.4)
        return np.stack((ax,ay),axis=-1)

    def disturbance(self,t,p):
        return np.concatenate((self.road(t,p),self.maneuver(t)),axis=-1)

    def capability(self,t,p):
        t=np.asarray(t)
        c=np.ones((*t.shape,4))*p.force_limit
        if self.fault or self.name=="fault":
            # Supplied actuator-authority signal, smooth physical derating, not diagnosis.
            c[...,0] *= 1-.75*np.clip((t-2.3)/.4,0,1)
        return c


NAMES = ("bump","one_side","diagonal","dip","undulations","rough","sweep",
         "brake","corner","rough_maneuver","combined","fault","sensor_failure","solver_failure")
