"""Centralized normalized LQR and constrained MPC; all forces in one QP."""
import json
import time
import numpy as np
import osqp
from scipy import sparse
from scipy.linalg import solve_discrete_are
from .model import ROOT, SCALE


def control_config(path=None):
    return json.loads((ROOT/"config/control.json" if path is None else __import__("pathlib").Path(path)).read_text())


class Controller:
    def __init__(self, model, cfg=None):
        self.model, self.cfg = model, cfg or control_config()
        self.previous = np.zeros(4)
        self.info = {}

    def bounded(self, requested, cap):
        delta = self.model.p.slew_limit*self.model.dt
        # Capability reduction can override command slew if physical bounds shrink.
        self.previous = np.clip(self.previous+np.clip(requested-self.previous,-delta,delta),-cap,cap)
        return self.previous.copy()

    def fallback(self,cap):
        self.info = {"failed":True,"fallback":True,"status":"invalid estimate or solver failure"}
        return self.bounded(np.zeros(4),cap)


class Passive(Controller):
    def command(self,x,w,cap,**kwargs):
        self.info={"status":"passive","failed":False}
        return np.zeros(4)


class LQR(Controller):
    def __init__(self,model,cfg=None):
        super().__init__(model,cfg)
        C,_=model.output_cost(self.cfg)
        Q=C.T@C+np.eye(18)*1e-6
        R=np.eye(4)*self.cfg["weights"]["effort"]
        P=solve_discrete_are(model.An,model.Bn,Q,R)
        self.gain=np.linalg.solve(R+model.Bn.T@P@model.Bn,model.Bn.T@P@model.An)

    def command(self,x,w,cap,valid=True,**kwargs):
        if not valid or not np.all(np.isfinite(x)) or not np.all(np.isfinite(w)):
            return self.fallback(cap)
        self.info={"status":"lqr","failed":False}
        return self.bounded(-self.model.p.force_limit*self.gain@(x/SCALE),cap)


class MPC(Controller):
    def __init__(self,model,cfg=None,aware=True):
        super().__init__(model,cfg)
        self.aware=aware
        m,cfg=model,self.cfg
        n=cfg["horizon"]; nx=18*n; nu=4*n; ns=8*n
        self.n,self.nx,self.nu,self.ns=n,nx,nu,ns
        self.C,self.D=m.output_cost(cfg)
        Q=self.C.T@self.C+np.eye(18)*1e-6
        effort=cfg["weights"]["effort"]
        delta=cfg["weights"]["delta_force"]
        # First difference of commands, with previous command in the first RHS.
        self.difference=sparse.kron(sparse.eye(n)-sparse.eye(n,k=-1),sparse.eye(4),format="csc")
        Pu=2*(effort*sparse.eye(nu)+delta*self.difference.T@self.difference)
        P=sparse.block_diag((sparse.kron(sparse.eye(n),2*Q),Pu,
                             2*cfg["weights"]["slack"]*sparse.eye(ns)),format="csc")
        dyn=sparse.kron(sparse.eye(n),sparse.eye(18))-sparse.kron(sparse.eye(n,k=-1),m.An)
        Bu=sparse.kron(sparse.eye(n),-m.Bn)
        zxs=sparse.csc_matrix((nx,ns)); zus=sparse.csc_matrix((nu,ns))
        zux=sparse.csc_matrix((nu,nx))
        T=np.zeros((4,18)); T[:,:7]=m.p.G*SCALE[:7]/m.p.travel_limit
        N=np.zeros((4,18)); N[:,3:7]=-np.diag(np.array(m.p.kt)*SCALE[3:7]/m.p.normal_static)
        Tn=sparse.kron(sparse.eye(n),T)
        Nn=sparse.kron(sparse.eye(n),N)
        # slacks grouped travel[4*N], normal[4*N].
        St=sparse.hstack((-sparse.eye(nu),sparse.csc_matrix((nu,nu))))
        Sn=sparse.hstack((sparse.csc_matrix((nu,nu)),-sparse.eye(nu)))
        A=sparse.vstack((sparse.hstack((dyn,Bu,zxs)),
                         sparse.hstack((zux,sparse.eye(nu),zus)),
                         sparse.hstack((zux,self.difference,zus)),
                         sparse.hstack((Tn,sparse.csc_matrix((nu,nu)),St)),
                         sparse.hstack((-Tn,sparse.csc_matrix((nu,nu)),St)),
                         sparse.hstack((-Nn,sparse.csc_matrix((nu,nu)),Sn)),
                         sparse.hstack((sparse.csc_matrix((ns,nx+nu)),sparse.eye(ns)))),format="csc")
        self.q=np.zeros(nx+nu+ns)
        self.lower=np.full(A.shape[0],-np.inf)
        self.upper=np.full(A.shape[0],np.inf)
        self.slices={}
        start=0
        for name,size in (("dyn",nx),("force",nu),("slew",nu),("travel_pos",nu),
                          ("travel_neg",nu),("normal",nu),("slack",ns)):
            self.slices[name]=slice(start,start+size);start+=size
        self.lower[self.slices["slack"]]=0
        self.upper[self.slices["travel_pos"]]=1
        self.upper[self.slices["travel_neg"]]=1
        self.solver=osqp.OSQP()
        self.solver.setup(P=sparse.triu(P).tocsc(),q=self.q,A=A,l=self.lower,u=self.upper,
                          verbose=False,eps_abs=cfg["osqp_tolerance"],eps_rel=cfg["osqp_tolerance"],
                          max_iter=4000,polishing=False,warm_starting=True)

    def command(self,x,w,cap,valid=True,force_failure=False):
        if not valid or not np.all(np.isfinite(x)) or not np.all(np.isfinite(w)) or force_failure:
            return self.fallback(cap)
        m,n=self.model,self.n
        bounds=cap if self.aware else np.ones(4)*m.p.force_limit
        dyn=np.tile(m.En@w,n)
        dyn[:18]+=m.An@(x/SCALE)
        self.lower[self.slices["dyn"]]=dyn
        self.upper[self.slices["dyn"]]=dyn
        self.q[:self.nx]=np.tile(2*self.C.T@(self.D@w),n)
        self.q[self.nx:self.nx+self.nu]=0
        self.q[self.nx:self.nx+4]=-2*self.cfg["weights"]["delta_force"]*self.previous/m.p.force_limit
        self.lower[self.slices["force"]]=-np.tile(bounds/m.p.force_limit,n)
        self.upper[self.slices["force"]]=np.tile(bounds/m.p.force_limit,n)
        rate=m.p.slew_limit*m.dt/m.p.force_limit
        lo=np.full(4*n,-rate);hi=np.full(4*n,rate)
        # Reset previous feasible command to new capability, avoiding infeasible first move.
        prev=np.clip(self.previous,-bounds,bounds)/m.p.force_limit
        lo[:4]+=prev;hi[:4]+=prev
        self.lower[self.slices["slew"]]=lo;self.upper[self.slices["slew"]]=hi
        road_load=np.array(m.p.kt)*w[:4]/m.p.normal_static
        self.upper[self.slices["normal"]]=np.tile(1-self.cfg["min_normal_fraction"]+road_load,n)
        self.solver.update(q=self.q,l=self.lower,u=self.upper)
        start=time.perf_counter()
        try:
            result=self.solver.solve(raise_error=False)
        except Exception as exc:
            result=None
            error=str(exc)
        elapsed=time.perf_counter()-start
        if result is None or result.info.status_val != 1 or not np.all(np.isfinite(result.x)):
            command=self.fallback(cap)
            self.info.update(status=error if result is None else result.info.status,
                             solve_time=elapsed,iterations=0 if result is None else result.info.iter)
            return command
        self.info={"status":result.info.status,"failed":False,"solve_time":elapsed,
                   "iterations":result.info.iter,
                   "slack":max(0.,float(np.max(result.x[self.nx+self.nu:]))),
                   "primal_residual":result.info.prim_res,"dual_residual":result.info.dual_res}
        requested=result.x[self.nx:self.nx+4]*m.p.force_limit
        # Fixed-capability MPC is unaware of reduction; physical actuator clips later.
        return self.bounded(requested,bounds)


def make_controller(name,model,cfg=None):
    if name=="passive":return Passive(model,cfg)
    if name=="lqr":return LQR(model,cfg)
    if name in ("mpc","mpc_fixed"):return MPC(model,cfg,aware=name=="mpc")
    raise ValueError(name)
