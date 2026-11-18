"""Headless comparison plots and full-car 3D diagnostic replay."""
import json
from pathlib import Path
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter, FFMpegWriter
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import numpy as np
from .model import Vehicle, CORNER


def comparison_plot(runs,output):
    fig,axes=plt.subplots(4,2,figsize=(13,12),sharex=True)
    for a,meta,metrics in runs:
        t=a["time"];label=meta["controller"]
        items=[(a["acceleration"][:,0],"CG acceleration [m/s²]"),
               (np.rad2deg(a["state"][:,1]),"Pitch slope [deg]"),
               (np.rad2deg(a["state"][:,2]),"Roll slope [deg]"),
               (np.max(abs(a["stroke"]),axis=1)*1000,"Max corner stroke [mm]"),
               (np.min(a["normal_load"],axis=1),"Minimum corner normal load [N]"),
               (np.max(abs(a["state"][:,14:]),axis=1),"Max delivered force [N]"),
               (np.max(a["slack"][:,None],axis=1),"Maximum predicted normalized slack"),
               (a["solver_time"]*1000,"QP solve time [ms]")]
        for ax,(y,title) in zip(axes.flat,items):
            ax.plot(t,y,label=label,lw=1);ax.set_ylabel(title);ax.grid(alpha=.25)
    axes[0,0].legend(ncol=2)
    axes[1,1].axhline(runs[0][1]["plant"]["travel_limit"]*1000,c="r",ls="--",lw=.7)
    for ax in axes[-1]:ax.set_xlabel("Time [s]")
    meta=runs[0][1]
    fig.suptitle(f"Full car · {meta['scenario']['name']} · {meta['sensing']} sensing · nonlinear validation plant")
    fig.tight_layout();fig.savefig(output,dpi=140);plt.close(fig)


def animate_run(a,meta,output,fps=20,scale=4.):
    p=Vehicle(**meta["plant"])
    t=a["time"];duration=t[-1]
    frame_times=np.arange(0,duration+1e-9,1/fps)
    indices=np.minimum(np.searchsorted(t,frame_times),len(t)-1)
    fig=plt.figure(figsize=(12,8))
    gs=fig.add_gridspec(3,2,width_ratios=[1.1,1])
    ax=fig.add_subplot(gs[:,0],projection="3d")
    plots=[fig.add_subplot(gs[i,1]) for i in range(3)]
    x=p.H[:,1];y=p.H[:,2]
    order=[0,1,3,2]
    ax.set(xlim=(-p.b-.5,p.a+.5),ylim=(-1.2,1.2),zlim=(-.5,1.2),
           xlabel="Forward [m]",ylabel="Left [m]",zlabel=f"Height (motion ×{scale:g})")
    ax.view_init(elev=22,azim=-135)
    shell=Poly3DCollection([],facecolors="#277a9c",alpha=.6,edgecolors="k")
    ax.add_collection3d(shell)
    wheel_lines=[ax.plot([],[],[],lw=8,color="#30343b")[0] for _ in CORNER]
    springs=[ax.plot([],[],[],lw=2,color="#ec994a")[0] for _ in CORNER]
    road_markers=[ax.plot([],[],[],"_",ms=25,color="#627a46")[0] for _ in CORNER]
    colors=plt.cm.tab10.colors
    for j,c in enumerate(CORNER):
        plots[0].plot(t,a["state"][:,14+j],label=c,c=colors[j],lw=.8)
        plots[1].plot(t,a["stroke"][:,j]*1000,c=colors[j],lw=.8)
        plots[2].plot(t,a["normal_load"][:,j],c=colors[j],lw=.8)
    for pp,label in zip(plots,("Delivered force [N]","Stroke [mm]","Normal load [N]")):
        pp.set_ylabel(label);pp.set_xlim(0,duration);pp.grid(alpha=.25)
    plots[0].legend(ncol=4);plots[-1].set_xlabel("Time [s]")
    cursors=[pp.axvline(0,c="black",lw=1) for pp in plots]
    title=fig.suptitle("")
    def update(frame):
        k=indices[frame];xx=a["state"][k]
        body=.55+scale*(p.H@xx[:3]);wheel=.0+scale*xx[3:7]
        vertices=np.column_stack((x,y,body))[order]
        shell.set_verts([vertices])
        for j in range(4):
            wheel_lines[j].set_data_3d([x[j],x[j]],[y[j]-.09,y[j]+.09],[wheel[j],wheel[j]])
            springs[j].set_data_3d([x[j],x[j]],[y[j],y[j]],[wheel[j],body[j]])
            rh=scale*a["disturbance"][k,j]-.13
            road_markers[j].set_data_3d([x[j]],[y[j]],[rh])
        for line in cursors:line.set_xdata([t[k],t[k]])
        title.set_text(f"{meta['controller']} · {meta['scenario']['name']} · t={t[k]:.2f}s · motion exaggerated ×{scale:g}")
        return [shell,*wheel_lines,*springs,*road_markers,*cursors,title]
    animation=FuncAnimation(fig,update,frames=len(indices),interval=1000/fps,blit=False)
    fig.tight_layout()
    output=Path(output);output.parent.mkdir(parents=True,exist_ok=True)
    writer=PillowWriter(fps=fps) if output.suffix.lower()==".gif" else FFMpegWriter(fps=fps,codec="libx264")
    animation.save(str(output),writer=writer,dpi=90)
    plt.close(fig)
