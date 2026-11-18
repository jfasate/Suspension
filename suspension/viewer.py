"""Native Tk/Matplotlib replay of recorded simulations; no video player required.

The scene uses logged states/road heights; displayed forces and loads are never
visually scaled. Straight road segments connect the saved contact heights;
older runs do not depend on today's road generator.
"""
from dataclasses import dataclass
from pathlib import Path
import time

import numpy as np
from matplotlib.figure import Figure
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

from .model import Vehicle, CORNER
from .simulation import load_run

COLORS = ("#2678b2", "#dd7b20", "#259451", "#b94d65")
VIEWS = {"Perspective": (24, -130), "Side / pitch": (0, -90),
         "Front / roll": (0, 0), "Top": (90, -90)}


@dataclass
class Playback:
    duration: float
    position: float = 0.
    speed: float = 1.
    playing: bool = True
    loop: bool = True

    def seek(self, position):
        self.position = float(np.clip(position, 0, self.duration))

    def advance(self, elapsed):
        if not self.playing:
            return self.position
        target = self.position + max(0., elapsed)*self.speed
        if target > self.duration:
            if self.loop:
                target %= self.duration
            else:
                target = self.duration
                self.playing = False
        self.seek(target)
        return self.position


class ReplayRun:
    def __init__(self, folder):
        self.folder = Path(folder)
        self.a, self.meta, self.metrics = load_run(folder)
        self.p = Vehicle(**self.meta["plant"])
        self.t = self.a["time"]
        if self.t.ndim != 1 or len(self.t) < 2 or not np.all(np.isfinite(self.t)) or np.any(np.diff(self.t) <= 0):
            raise ValueError(f"{folder}: need at least two increasing, finite timestamps")
        expected = {"state": 18, "acceleration": 3, "stroke": 4,
                    "normal_load": 4, "disturbance": 6, "capability": 4}
        for key, size in expected.items():
            value = self.a[key]
            if value.shape != (len(self.t), size) or not np.all(np.isfinite(value)):
                raise ValueError(f"{folder}: invalid {key} array")
        self.duration = float(self.t[-1])
        if self.t[0] != 0 or self.duration <= 0:
            raise ValueError(f"{folder}: replay must start at zero and have positive duration")

    @property
    def controller(self):
        return self.meta["controller"]

    def index(self, position):
        # Floor sample selection: scrubbing never displays a future state.
        return int(np.clip(np.searchsorted(self.t, position, side="right")-1, 0, len(self.t)-1))

    def geometry(self, index, scale):
        x = self.a["state"][index]
        radius = .14
        body = self.p.cg_height-radius + scale*(self.p.H@x[:3])
        wheel = scale*x[3:7]
        road = -radius+scale*self.a["disturbance"][index,:4]
        return body, wheel, road


def load_replays(path):
    path = Path(path)
    folders = [path] if (path/"run.npz").is_file() else sorted(
        p for p in path.glob("*") if p.is_dir() and (p/"run.npz").is_file())
    if not folders:
        raise ValueError(f"No saved runs in {path}. First run: python -m suspension compare")
    runs = {}
    for folder in folders:
        run = ReplayRun(folder)
        if run.controller in runs:
            raise ValueError(f"Duplicate controller {run.controller} in {path}")
        runs[run.controller] = run
    first = next(iter(runs.values()))
    for run in runs.values():
        # Only overlay controllers from the same experiment and physical plant.
        if run.meta["scenario"] != first.meta["scenario"] or run.meta["plant"] != first.meta["plant"] or run.meta["sensing"] != first.meta["sensing"]:
            raise ValueError("Choose controller runs from the same scenario, plant and sensing mode")
    return runs


class ReplayFigure:
    """Backend-independent scene, also usable for headless snapshot export/tests."""
    def __init__(self, runs, controller="mpc", scale=4.):
        if not np.isfinite(scale) or scale <= 0:
            raise ValueError("Motion scale must be finite and positive")
        self.runs, self.scale = runs, scale
        self.fig = Figure(figsize=(13.5, 7.8), facecolor="#f7f9fb")
        grid = self.fig.add_gridspec(5, 2, width_ratios=(1.2, 1), hspace=.38,
                                     left=.035, right=.985, bottom=.075, top=.93, wspace=.20)
        self.ax = self.fig.add_subplot(grid[:, 0], projection="3d")
        self.plots = []
        for row in range(5):
            self.plots.append(self.fig.add_subplot(grid[row,1], sharex=self.plots[0] if row else None))
        self.title = self.fig.suptitle("", fontsize=12, fontweight="bold")
        self.view = "Perspective"
        self.select(controller if controller in runs else next(iter(runs)))

    def select(self, controller):
        camera = (self.ax.elev,self.ax.azim) if hasattr(self,"run") else VIEWS[self.view]
        self.run = self.runs[controller]
        self.p = self.run.p
        p, a, t = self.p, self.run.a, self.run.t
        self.ax.clear()
        self.ax.set(xlim=(-p.b-.5,p.a+1.), ylim=(-1.15,1.15),
                    xlabel="Forward [m]", ylabel="Left [m]", zlabel="Height [m]")
        self.ax.set_box_aspect((p.a+p.b+1.5,2.3,1.8))
        self.shell = Poly3DCollection([], facecolors="#72bbd5", alpha=.65,
                                       edgecolors="#165770", linewidths=1.5)
        self.ax.add_collection3d(self.shell)
        self.wheels = [self.ax.plot([],[],[],color="#252a32",lw=5)[0] for _ in CORNER]
        self.springs = [self.ax.plot([],[],[],color=c,lw=1.8)[0] for c in COLORS]
        self.dampers = [self.ax.plot([],[],[],color="#737b83",lw=3)[0] for _ in CORNER]
        self.contacts = [self.ax.plot([],[],[],"_",color="#4c8739",ms=22,mew=3)[0] for _ in CORNER]
        self.labels = [self.ax.text(0,0,0,c,color=color,fontsize=10,fontweight="bold") for c,color in zip(CORNER,COLORS)]
        self.road_lines = [self.ax.plot([],[],[],c="#83a66e",lw=3,alpha=.75)[0] for _ in range(2)]
        self.front = self.ax.text(0,0,0,"FRONT →",fontsize=9,color="#165770")
        self.scene_note = self.ax.text2D(.03,.96,"",transform=self.ax.transAxes,fontsize=10)
        self.ax.view_init(elev=camera[0],azim=camera[1])
        for ax in self.plots:
            ax.clear();ax.grid(alpha=.22);ax.set_facecolor("white")
        self.plots[0].plot(t,a["acceleration"][:,0],color="#226e9a",lw=1,label=controller_label(controller))
        passive = self.runs.get("passive")
        if passive is not None and controller != "passive":
            self.plots[0].plot(passive.t,passive.a["acceleration"][:,0],c="#929aa3",lw=.8,alpha=.8,label="Passive")
        self.plots[0].legend(loc="upper right",ncol=2,fontsize=7)
        self.plots[1].plot(t,np.rad2deg(a["state"][:,1]),c="#8752a2",lw=1,label="Pitch")
        self.plots[1].plot(t,np.rad2deg(a["state"][:,2]),c="#d37d25",lw=1,label="Roll")
        self.plots[1].legend(loc="upper right",ncol=2,fontsize=7)
        for j, (corner, color) in enumerate(zip(CORNER,COLORS)):
            self.plots[2].plot(t,a["state"][:,14+j],c=color,lw=.8,label=corner)
            self.plots[3].plot(t,a["stroke"][:,j]*1000,c=color,lw=.8)
            self.plots[4].plot(t,a["normal_load"][:,j],c=color,lw=.8)
        self.plots[2].legend(loc="upper right",ncol=4,fontsize=7)
        for sign in (-1,1):
            self.plots[3].axhline(sign*p.travel_limit*1000,c="#b73131",ls="--",lw=.7)
        self.plots[4].axhline(0,c="#b73131",ls="--",lw=.7)
        for ax,label in zip(self.plots,("CG accel\n[m/s²]","Pitch / roll\n[degrees]","Force\n[N]","Travel\n[mm]","Tire load\n[N]")):
            ax.set_ylabel(label,fontsize=8);ax.tick_params(labelsize=8)
            ax.set_xlim(0,self.run.duration)
        for ax in self.plots[:-1]:ax.tick_params(labelbottom=False)
        self.plots[-1].set_xlabel("Time [s]",fontsize=9)
        # Common scales make controller changes visually comparable.
        ranges = [[],[],[-p.force_limit,p.force_limit],
                  [-p.travel_limit*1000,p.travel_limit*1000],[0.]]
        for other in self.runs.values():
            values = (other.a["acceleration"][:,0],np.rad2deg(other.a["state"][:,1:3]),
                      other.a["state"][:,14:],other.a["stroke"]*1000,other.a["normal_load"])
            for extent,value in zip(ranges,values):
                extent.extend((float(np.min(value)),float(np.max(value))))
        for ax,extent in zip(self.plots,ranges):
            low,high = min(extent),max(extent)
            margin = max((high-low)*.08,1e-3)
            ax.set_ylim(low-margin,high+margin)
        self.cursors = [ax.axvline(0,c="#252a32",lw=1) for ax in self.plots]
        self.set_scale(self.scale)
        self.draw(0)

    def set_view(self, name):
        self.view = name
        elev, azim = VIEWS[name]
        self.ax.view_init(elev=elev,azim=azim)

    def set_scale(self, scale):
        scale = float(scale)
        if not np.isfinite(scale) or scale <= 0:
            raise ValueError("Motion scale must be finite and positive")
        self.scale = scale
        a, p = self.run.a, self.p
        body = p.cg_height-.14+self.scale*(a["state"][:,:3]@p.H.T)
        wheel = self.scale*a["state"][:,3:7]
        road = -.14+self.scale*a["disturbance"][:,:4]
        self.ax.set_zlim(min(-.35,float(road.min())-.1,float(wheel.min())-.2),
                         max(.85,float(body.max())+.18))

    def draw(self, position):
        r, p = self.run, self.p
        k = r.index(position)
        body,wheel,road = r.geometry(k,self.scale)
        x,y = p.H[:,1],p.H[:,2]
        vertices = np.column_stack((x,y,body))[[0,1,3,2]]
        self.shell.set_verts([vertices])
        phase = np.linspace(0,2*np.pi,35)
        coil = np.linspace(0,1,75)
        for j in range(4):
            self.wheels[j].set_data_3d(x[j]+.14*np.cos(phase),np.full_like(phase,y[j]),wheel[j]+.14*np.sin(phase))
            # Coil is schematic; its end heights follow the logged body/wheel motion.
            self.springs[j].set_data_3d(x[j]-.045+.035*np.sin(coil*12*np.pi),
                                       np.full_like(coil,y[j]),wheel[j]+(body[j]-wheel[j])*coil)
            self.dampers[j].set_data_3d([x[j]+.05,x[j]+.05],[y[j],y[j]],[wheel[j],body[j]])
            lost = r.a["normal_load"][k,j] <= 1e-6
            self.contacts[j].set_data_3d([x[j]],[y[j]],[road[j]])
            self.contacts[j].set_color("#cc2424" if lost else "#4c8739")
            self.wheels[j].set_color("#cc2424" if lost else "#252a32")
            self.labels[j].set_position((x[j],y[j]));self.labels[j].set_3d_properties(body[j]+.05,zdir=None)
        # Schematic wheel-to-wheel road segments, not a reconstructed road mesh.
        # Each contact marker uses the exact recorded height at its wheel.
        for side in range(2):
            self.road_lines[side].set_data_3d([x[side+2],x[side]], [y[side+2],y[side]],
                                              [road[side+2],road[side]])
        self.front.set_position((p.a+.25,-p.cg_y));self.front.set_3d_properties(float(np.mean(body[:2])),zdir=None)
        for cursor in self.cursors:cursor.set_xdata([position,position])
        self.title.set_text(f"{controller_label(r.controller)} · {r.meta['scenario']['name']} · recorded simulation · {position:.2f} / {r.duration:.2f} s")
        self.scene_note.set_text(f"Motion ×{self.scale:g}  ·  forces, loads and plot values unscaled\nDrag to rotate · red wheels indicate loss of contact")
        return k


def controller_label(name):
    return {"passive":"Passive", "lqr":"LQR", "mpc_fixed":"Fixed-capability MPC",
            "mpc":"Capability-aware MPC"}.get(name,name)


class ReplayWindow:
    def __init__(self, root, runs, controller="mpc", scale=4., speed=1., start=0., autoplay=True):
        import tkinter as tk
        from tkinter import ttk
        from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
        if not np.isfinite(speed) or speed <= 0:
            raise ValueError("Playback speed must be finite and positive")
        self.root = root
        root.title("Full-car suspension · Python viewer")
        width,height = min(1440,root.winfo_screenwidth()-80),min(960,root.winfo_screenheight()-80)
        root.geometry(f"{width}x{height}");root.minsize(1000,740)
        root.columnconfigure(0,weight=1);root.rowconfigure(2,weight=1)
        self.scene = ReplayFigure(runs,controller,scale)
        self.clock = Playback(self.scene.run.duration,speed=speed,playing=autoplay)
        self.clock.seek(start)
        self._last_tick = time.monotonic()
        self._closed = False
        self._updating_slider = False
        top = ttk.Frame(root,padding=(10,8));top.grid(row=0,column=0,sticky="ew")
        self.play_button = ttk.Button(top,text="Pause" if autoplay else "Play",command=self.toggle)
        self.play_button.pack(side="left",padx=(0,5))
        ttk.Button(top,text="Restart",command=self.restart).pack(side="left",padx=(0,16))
        ttk.Label(top,text="Controller").pack(side="left")
        self.controller_var = tk.StringVar(value=self.scene.run.controller)
        selector = ttk.Combobox(top,textvariable=self.controller_var,values=list(runs),state="readonly",width=11)
        selector.pack(side="left",padx=(4,12));selector.bind("<<ComboboxSelected>>",self.select)
        ttk.Label(top,text="Speed").pack(side="left")
        self.speed_var = tk.StringVar(value=f"{speed:g}")
        speed_box = ttk.Combobox(top,textvariable=self.speed_var,values=("0.25","0.5","1","2"),state="readonly",width=5)
        speed_box.pack(side="left",padx=(4,12));speed_box.bind("<<ComboboxSelected>>",self.change_speed)
        ttk.Label(top,text="Motion ×").pack(side="left")
        self.scale_var = tk.StringVar(value=f"{scale:g}")
        scale_box = ttk.Combobox(top,textvariable=self.scale_var,values=("1","4","8"),state="readonly",width=5)
        scale_box.pack(side="left",padx=(4,12));scale_box.bind("<<ComboboxSelected>>",self.change_scale)
        self.view_var = tk.StringVar(value="Perspective")
        view_box = ttk.Combobox(top,textvariable=self.view_var,values=list(VIEWS),state="readonly",width=16)
        view_box.pack(side="left",padx=(0,12));view_box.bind("<<ComboboxSelected>>",self.change_view)
        self.loop_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(top,text="Loop",variable=self.loop_var).pack(side="left")
        self.summary = ttk.Label(root,padding=(12,2),wraplength=1300)
        self.summary.grid(row=1,column=0,sticky="ew")
        self.canvas = FigureCanvasTkAgg(self.scene.fig,master=root)
        self.canvas.get_tk_widget().grid(row=2,column=0,sticky="nsew")
        transport = ttk.Frame(root,padding=(12,0));transport.grid(row=3,column=0,sticky="ew")
        transport.columnconfigure(0,weight=1)
        self.slider = tk.Scale(transport,from_=0,to=self.clock.duration,resolution=.001,
                               orient="horizontal",showvalue=False,command=self.seek,highlightthickness=0)
        self.slider.grid(row=0,column=0,sticky="ew")
        self.slider.bind("<ButtonPress-1>",self.pause_for_seek)
        self.time_label = ttk.Label(transport,width=19);self.time_label.grid(row=0,column=1)
        self.table = ttk.Treeview(root,columns=("road","travel","force","load","limit","status"),height=4,selectmode="none")
        self.table.heading("#0",text="Corner");self.table.column("#0",width=70,stretch=False)
        for key,title in (("road","Road [mm]"),("travel","Travel [mm]"),("force","Delivered force [N]"),
                          ("load","Tire load [N]"),("limit","Available force [N]"),("status","Status")):
            self.table.heading(key,text=title);self.table.column(key,width=125,anchor="center")
        for corner in CORNER:self.table.insert("","end",iid=corner,text=corner)
        self.table.tag_configure("warning",foreground="#b71c1c")
        self.table.grid(row=4,column=0,sticky="ew",padx=12,pady=(2,8))
        self.update_summary();self.render()
        root.protocol("WM_DELETE_WINDOW",self.close)
        root.after(40,self.tick)

    def update_summary(self):
        r = self.scene.run;m = r.metrics
        description = (f"Whole-run metrics: CG RMS {m['cg_accel_rms']:.3f} m/s² · "
                       f"pitch RMS {m['pitch_rms_deg']:.3f}° · roll RMS {m['roll_rms_deg']:.3f}° · "
                       f"sensing: {r.meta['sensing']} · safety: {'pass' if m['accepted_safety'] else 'VIOLATIONS'}")
        passive = self.scene.runs.get("passive")
        if passive and r.controller != "passive" and passive.metrics['cg_accel_rms'] > 1e-10:
            gain = 100*(1-m['cg_accel_rms']/passive.metrics['cg_accel_rms'])
            description += f" · CG RMS reduction vs passive {gain:.1f}%"
        self.summary.configure(text=description)

    def render(self):
        k = self.scene.draw(self.clock.position)
        r = self.scene.run;a = r.a
        for j,corner in enumerate(CORNER):
            contact = a['normal_load'][k,j] <= 1e-6
            travel = abs(a['stroke'][k,j]) > r.p.travel_limit
            status = "CONTACT LOST" if contact else "TRAVEL LIMIT" if travel else "OK"
            values = (f"{1000*a['disturbance'][k,j]:.1f}",f"{1000*a['stroke'][k,j]:.1f}",
                      f"{a['state'][k,14+j]:.0f}",f"{a['normal_load'][k,j]:.0f}",
                      f"±{a['capability'][k,j]:.0f}",status)
            self.table.item(corner,values=values,tags=("warning",) if contact or travel else ())
        self._updating_slider = True
        self.slider.set(self.clock.position)
        self._updating_slider = False
        self.time_label.configure(text=f"{self.clock.position:.2f} / {self.clock.duration:.2f} s")
        self.play_button.configure(text="Pause" if self.clock.playing else "Play")
        self.canvas.draw_idle()

    def tick(self):
        if self._closed:return
        now = time.monotonic()
        self.clock.loop = self.loop_var.get()
        before = self.clock.position
        self.clock.advance(now-self._last_tick);self._last_tick = now
        if self.clock.position != before:self.render()
        self.root.after(40,self.tick)

    def toggle(self):
        self.clock.playing = not self.clock.playing
        self._last_tick = time.monotonic()
        self.render()

    def restart(self):
        self.clock.seek(0);self._last_tick = time.monotonic();self.render()

    def seek(self, value):
        if self._updating_slider or abs(float(value)-self.clock.position)<.0011:return
        self.clock.playing = False
        self.clock.seek(float(value));self.render()

    def pause_for_seek(self, event=None):
        # Pause before the Scale's class binding changes its value, so a pending
        # playback tick cannot move the thumb while the user is dragging it.
        self.clock.playing = False
        self.play_button.configure(text="Play")

    def select(self, event=None):
        # Keep the playback clock and viewing angle when changing controller.
        self.scene.select(self.controller_var.get())
        self.clock.duration = self.scene.run.duration;self.clock.seek(self.clock.position)
        self.slider.configure(to=self.clock.duration)
        self._last_tick = time.monotonic()
        self.update_summary();self.render()

    def change_speed(self, event=None):
        self.clock.speed = float(self.speed_var.get());self._last_tick = time.monotonic()

    def change_scale(self, event=None):
        self.scene.set_scale(float(self.scale_var.get()));self.render()

    def change_view(self, event=None):
        self.scene.set_view(self.view_var.get());self.render()

    def close(self):
        self._closed = True
        self.root.destroy()


def view_replays(path, controller="mpc", scale=4., speed=1., start=0., snapshot=None):
    runs = load_replays(path)
    if snapshot:
        from matplotlib.backends.backend_agg import FigureCanvasAgg
        scene = ReplayFigure(runs,controller,scale)
        FigureCanvasAgg(scene.fig)
        scene.draw(float(np.clip(start,0,scene.run.duration)))
        output = Path(snapshot);output.parent.mkdir(parents=True,exist_ok=True)
        scene.fig.savefig(output,dpi=140)
        print(output)
        return
    try:
        import tkinter as tk
        root = tk.Tk()
    except ImportError as exc:
        raise RuntimeError("Python Tk is required. On Ubuntu install python3-tk, then retry.") from exc
    except tk.TclError as exc:
        raise RuntimeError("Could not open a graphical display. Run from a desktop terminal, or use --snapshot results/viewer.png.") from exc
    ReplayWindow(root,runs,controller,scale,speed,start)
    root.mainloop()
