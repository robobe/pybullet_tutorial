import tkinter as tk
from collections import deque
import time

import matplotlib
matplotlib.use("TkAgg")
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure

import pybullet as p
import pybullet_data


class App:
    def __init__(self, root):
        self.root = root
        root.title("PyBullet + Tkinter + Live Plot")

        # --- PyBullet setup ---
        # Use GUI if you want to see it: p.connect(p.GUI)
        self.cid = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.resetSimulation()
        p.setGravity(0, 0, -9.81)
        p.setTimeStep(1/240)

        p.loadURDF("plane.urdf")
        self.obj_id = p.loadURDF("r2d2.urdf", [0, 0, 1])

        # --- UI state ---
        self.running = False
        self.t0 = time.time()

        # --- Data buffers for plot ---
        self.max_points = 600
        self.ts = deque(maxlen=self.max_points)
        self.zs = deque(maxlen=self.max_points)

        # --- Controls ---
        btn_frame = tk.Frame(root)
        btn_frame.pack(fill="x", padx=10, pady=8)

        self.start_btn = tk.Button(btn_frame, text="Start", command=self.start)
        self.pause_btn = tk.Button(btn_frame, text="Pause", command=self.pause)
        self.reset_btn = tk.Button(btn_frame, text="Reset", command=self.reset)

        self.start_btn.pack(side="left", padx=5)
        self.pause_btn.pack(side="left", padx=5)
        self.reset_btn.pack(side="left", padx=5)

        # --- Plot area ---
        fig = Figure(figsize=(6, 3), dpi=100)
        self.ax = fig.add_subplot(111)
        self.ax.set_title("Z Height vs Time")
        self.ax.set_xlabel("t (s)")
        self.ax.set_ylabel("z (m)")
        self.line, = self.ax.plot([], [])

        self.canvas = FigureCanvasTkAgg(fig, master=root)
        self.canvas.get_tk_widget().pack(fill="both", expand=True, padx=10, pady=10)

        # Run periodic update
        self.tick()

        # clean shutdown
        root.protocol("WM_DELETE_WINDOW", self.on_close)

    def start(self):
        self.running = True

    def pause(self):
        self.running = False

    def reset(self):
        self.running = False
        p.resetBasePositionAndOrientation(self.obj_id, [0, 0, 1], [0, 0, 0, 1])
        p.resetBaseVelocity(self.obj_id, [0, 0, 0], [0, 0, 0])
        self.t0 = time.time()
        self.ts.clear()
        self.zs.clear()
        self.update_plot()

    def step_sim(self, steps=8):
        # Run several physics steps per UI tick for smoother physics
        for _ in range(steps):
            p.stepSimulation()

    def sample(self):
        pos, _ = p.getBasePositionAndOrientation(self.obj_id)
        t = time.time() - self.t0
        self.ts.append(t)
        self.zs.append(pos[2])

    def update_plot(self):
        if len(self.ts) < 2:
            self.line.set_data([], [])
        else:
            self.line.set_data(self.ts, self.zs)
            self.ax.relim()
            self.ax.autoscale_view()
        self.canvas.draw_idle()

    def tick(self):
        if self.running:
            self.step_sim(steps=8)
            self.sample()
            self.update_plot()

        # schedule next tick (20 ms ~ 50 Hz UI)
        self.root.after(20, self.tick)

    def on_close(self):
        try:
            p.disconnect()
        except Exception:
            pass
        self.root.destroy()


if __name__ == "__main__":
    root = tk.Tk()
    app = App(root)
    root.mainloop()
