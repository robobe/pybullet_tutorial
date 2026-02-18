import math
import os
import random
import time
import tkinter as tk
from collections import deque

import matplotlib
import pybullet as p
import pybullet_data
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure

matplotlib.use("TkAgg")

JOINT_INDEX = 0
TIME_STEP = 1.0 / 240.0
MAX_FORCE = 200.0
KP = 4.0
MAX_VELOCITY = 2.0
UI_PERIOD_MS = 20
DISTURB_LINK_INDEX = 0

# 3 predefined target positions (rad)
PRESET_TARGETS = [math.pi / 6.0, math.pi / 3.0, -math.pi / 4.0]


class App:
    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.root.title("RRBot Velocity Control + Joint Position Plot")

        # ---- PyBullet setup ----
        self.cid = p.connect(p.GUI)
        p.resetSimulation()
        p.setGravity(0, 0, -9.8)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.loadURDF("plane.urdf")

        cwd = os.path.dirname(os.path.abspath(__file__))
        p.setAdditionalSearchPath(cwd)
        self.robot = p.loadURDF("rrbot_2.urdf", basePosition=[0, 0, 0.5], useFixedBase=True)

        p.setTimeStep(TIME_STEP)
        p.setRealTimeSimulation(False)

        # Optional PyBullet sliders for controller tuning.
        self.kp_param_id = p.addUserDebugParameter("Kp", 0.0, 40.0, 12.0, self.cid)
        self.max_vel_param_id = p.addUserDebugParameter(
            "max velocity (rad/s)", 0.1, 20.0, 8.0, self.cid
        )
        self.noise_std_param_id = p.addUserDebugParameter(
            "sensor noise std (rad)", 0.0, 0.2, 0.01, self.cid
        )
        self.delay_steps_param_id = p.addUserDebugParameter(
            "cmd delay (steps)", 0, 60, 10, self.cid
        )
        self.dist_amp_param_id = p.addUserDebugParameter(
            "disturb torque amp (Nm)", 0.0, 20.0, 6.0, self.cid
        )
        self.dist_freq_param_id = p.addUserDebugParameter(
            "disturb freq (Hz)", 0.0, 8.0, 1.8, self.cid
        )

        # ---- Control + plotting state ----
        self.target_pos = 0.0
        self.t0 = time.time()
        self.sim_time = 0.0
        self.cmd_queue: deque[float] = deque(maxlen=5000)
        self.max_points = 1200
        self.ts = deque(maxlen=self.max_points)
        self.joint_pos = deque(maxlen=self.max_points)
        self.target_trace = deque(maxlen=self.max_points)

        # ---- Controls (4 buttons) ----
        btn_frame = tk.Frame(root)
        btn_frame.pack(fill="x", padx=10, pady=8)

        tk.Button(
            btn_frame,
            text=f"Target 1 ({PRESET_TARGETS[0]:.2f} rad)",
            command=lambda: self.set_target(PRESET_TARGETS[0]),
        ).pack(side="left", padx=5)
        tk.Button(
            btn_frame,
            text=f"Target 2 ({PRESET_TARGETS[1]:.2f} rad)",
            command=lambda: self.set_target(PRESET_TARGETS[1]),
        ).pack(side="left", padx=5)
        tk.Button(
            btn_frame,
            text=f"Target 3 ({PRESET_TARGETS[2]:.2f} rad)",
            command=lambda: self.set_target(PRESET_TARGETS[2]),
        ).pack(side="left", padx=5)
        tk.Button(btn_frame, text="Reset", command=self.reset_system).pack(side="left", padx=5)

        # ---- Plot ----
        fig = Figure(figsize=(7, 3.5), dpi=100)
        self.ax = fig.add_subplot(111)
        self.ax.set_title("Joint Position vs Time")
        self.ax.set_xlabel("t (s)")
        self.ax.set_ylabel("Joint 0 Position (rad)")
        self.pos_line, = self.ax.plot([], [], label="joint position")
        self.target_line, = self.ax.plot([], [], "--", label="target")
        self.ax.legend(loc="upper right")

        self.canvas = FigureCanvasTkAgg(fig, master=root)
        self.canvas.get_tk_widget().pack(fill="both", expand=True, padx=10, pady=10)

        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        self.tick()

    def set_target(self, target_rad: float) -> None:
        self.target_pos = target_rad

    def reset_system(self) -> None:
        self.target_pos = 0.0
        p.resetJointState(self.robot, JOINT_INDEX, targetValue=0.0, targetVelocity=0)
        self.sim_time = 0.0
        self.cmd_queue.clear()
        self.t0 = time.time()
        self.ts.clear()
        self.joint_pos.clear()
        self.target_trace.clear()
        self.update_plot()

    def control_step(self) -> None:
        kp = p.readUserDebugParameter(self.kp_param_id, self.cid)
        max_velocity = p.readUserDebugParameter(self.max_vel_param_id, self.cid)
        noise_std = p.readUserDebugParameter(self.noise_std_param_id, self.cid)
        delay_steps = int(p.readUserDebugParameter(self.delay_steps_param_id, self.cid))
        disturb_amp = p.readUserDebugParameter(self.dist_amp_param_id, self.cid)
        disturb_freq = p.readUserDebugParameter(self.dist_freq_param_id, self.cid)

        current_pos = p.getJointState(self.robot, JOINT_INDEX)[0]
        measured_pos = current_pos + random.gauss(0.0, noise_std)
        pos_error = self.target_pos - measured_pos
        raw_velocity_cmd = kp * pos_error
        self.cmd_queue.append(raw_velocity_cmd)
        if len(self.cmd_queue) > delay_steps:
            target_velocity = self.cmd_queue.popleft()
        else:
            target_velocity = 0.0
        target_velocity = max(-max_velocity, min(max_velocity, target_velocity))

        p.setJointMotorControl2(
            bodyIndex=self.robot,
            jointIndex=JOINT_INDEX,
            controlMode=p.VELOCITY_CONTROL,
            targetVelocity=target_velocity,
            force=MAX_FORCE,
        )
        if disturb_amp > 0.0 and disturb_freq > 0.0:
            disturb_torque_y = disturb_amp * math.sin(2.0 * math.pi * disturb_freq * self.sim_time)
            p.applyExternalTorque(
                objectUniqueId=self.robot,
                linkIndex=DISTURB_LINK_INDEX,
                torqueObj=[0.0, disturb_torque_y, 0.0],
                flags=p.WORLD_FRAME,
            )

        p.stepSimulation()
        self.sim_time += TIME_STEP

    def sample(self) -> None:
        t = time.time() - self.t0
        current_pos = p.getJointState(self.robot, JOINT_INDEX)[0]
        self.ts.append(t)
        self.joint_pos.append(current_pos)
        self.target_trace.append(self.target_pos)

    def update_plot(self) -> None:
        if len(self.ts) < 2:
            self.pos_line.set_data([], [])
            self.target_line.set_data([], [])
        else:
            self.pos_line.set_data(self.ts, self.joint_pos)
            self.target_line.set_data(self.ts, self.target_trace)
            self.ax.relim()
            self.ax.autoscale_view()
        self.canvas.draw_idle()

    def tick(self) -> None:
        # Run a few physics steps each UI tick for smoother simulation.
        for _ in range(8):
            self.control_step()

        self.sample()
        self.update_plot()
        self.root.after(UI_PERIOD_MS, self.tick)

    def on_close(self) -> None:
        try:
            p.disconnect(self.cid)
        except Exception:
            pass
        self.root.destroy()


if __name__ == "__main__":
    root = tk.Tk()
    app = App(root)
    root.mainloop()
