"""Simulated measured-output Kalman observer; no truth state input.

Channels: vertical body IMU acceleration, pitch/roll gyros, four suspension
deflections and four delivered-force transducers. ax/ay and current road height
from a simulated perception system are measured inputs (no future preview).
An absolute vertical reference for those road estimates is an explicit assumption.
"""
import numpy as np
from scipy.linalg import solve_discrete_are
from .model import SCALE


class Observer:
    def __init__(self,model):
        self.m=model
        C=np.zeros((11,18))
        C[0]=model.A[7]
        C[1,8]=1;C[2,9]=1
        C[3:7,:7]=model.p.G
        C[7:11,14:]=np.eye(4)
        self.C=C
        self.noise=np.array([.035,.002,.002,.0005,.0005,.0005,.0005,12.,12.,12.,12.])
        self.Cn=C*SCALE/self.noise[:,None]
        # Road estimation error and parameter mismatch enter process noise.
        sigma=np.array([.0002,.0003,.0003,.003,.003,.003,.003,.01,.008,.008,.08,.08,.08,.08,20.,20.,20.,20.])/SCALE
        Q=np.diag(sigma**2)
        P=solve_discrete_are(model.An.T,self.Cn.T,Q,np.eye(11))
        self.K=P@self.Cn.T@np.linalg.inv(self.Cn@P@self.Cn.T+np.eye(11))
        self.xn=np.zeros(18)
        self.valid=True
        self.invalid_count=0
        self.recovery=0

    def measure(self,truth,actual_vertical_accel,p,rng):
        y=self.C@truth
        y[0]=actual_vertical_accel
        y[3:7]=p.G@truth[:7]
        return y+rng.normal(0,self.noise)

    def update(self,measurement):
        if not np.all(np.isfinite(measurement)):
            self.valid=False;self.recovery=0;self.invalid_count+=1
            return self.xn*SCALE
        innovation=measurement/self.noise-self.Cn@self.xn
        # Reject gross outliers, allow normal model mismatch and road excitation.
        if np.max(np.abs(innovation))>150:
            self.valid=False;self.recovery=0;self.invalid_count+=1
            return self.xn*SCALE
        self.xn+=self.K@innovation
        self.recovery+=1
        self.valid=bool(np.all(np.isfinite(self.xn)) and self.recovery>=3)
        return self.xn*SCALE

    def predict(self,command,measured_disturbance):
        w=measured_disturbance
        self.xn=self.m.An@self.xn+self.m.Bn@(command/self.m.p.force_limit)+self.m.En@w

    def observability(self):
        # Iterative orthonormal row-space expansion avoids raw O(A^17) scaling.
        from scipy.linalg import orth
        basis=orth(self.Cn.T,rcond=1e-9)
        for _ in range(18):
            basis=orth(np.column_stack((basis,self.m.An.T@basis)),rcond=1e-9)
        return {"rank":basis.shape[1],"states":18,
                "closed_loop_spectral_radius":float(max(abs(np.linalg.eigvals((np.eye(18)-self.K@self.Cn)@self.m.An))))}
