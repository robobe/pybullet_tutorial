import math
import random
import time
from pathlib import Path

import pybullet as p
import pybullet_data

from plotjuggler_udp import PlotJugglerUdpClient, TimeBase

KP = 18.0
KI = 0.0
KD = 4.0
DT = 1 / 240
TARGET_X = 2.0
SIMULATION_DURATION = 8.0
SIMULATION_STEPS = int(SIMULATION_DURATION / DT)
NOISE_STD = 0.08
FILTER_CUTOFF_HZ = 3.0
NOISE_SEED = 7


class PID:
    def __init__(self, kp, ki, kd, dt):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt
        self.integral = 0.0
        self.prev_error = 0.0

    def set_gains(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd

    def update(self, target, current):
        error = target - current
        self.integral += error * self.dt
        derivative = (error - self.prev_error) / self.dt
        self.prev_error = error

        return (
            self.kp * error
            + self.ki * self.integral
            + self.kd * derivative
        )


class LowPassFilter:
    def __init__(self, cutoff_hz, dt, initial_value=0.0):
        self.dt = dt
        self.value = initial_value
        self.set_cutoff(cutoff_hz)

    def set_cutoff(self, cutoff_hz):
        self.cutoff_hz = max(cutoff_hz, 0.001)
        tau = 1.0 / (2.0 * math.pi * self.cutoff_hz)
        self.alpha = self.dt / (tau + self.dt)

    def update(self, measurement):
        self.value += self.alpha * (measurement - self.value)
        return self.value


random.seed(NOISE_SEED)

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setTimeStep(DT)
p.setGravity(0, 0, 0)

p.loadURDF("plane.urdf")
box = p.loadURDF("cube_small.urdf", basePosition=[0, 0, 0.1])
p.changeDynamics(box, -1, linearDamping=0.2)

pid = PID(kp=KP, ki=KI, kd=KD, dt=DT)

target_x_param_id = p.addUserDebugParameter("Target X", 0, 4, TARGET_X)
kp_param_id = p.addUserDebugParameter("Kp", 0, 100, KP)
ki_param_id = p.addUserDebugParameter("Ki", 0, 100, KI)
kd_param_id = p.addUserDebugParameter("Kd", 0, 100, KD)
noise_std_param_id = p.addUserDebugParameter("Noise std", 0, 0.4, NOISE_STD)
filter_cutoff_param_id = p.addUserDebugParameter(
    "Filter cutoff Hz", 0.1, 20, FILTER_CUTOFF_HZ
)
filter_enabled_param_id = p.addUserDebugParameter("Use filter", 0, 1, 1)

pos, _ = p.getBasePositionAndOrientation(box)
low_pass_filter = LowPassFilter(
    cutoff_hz=FILTER_CUTOFF_HZ,
    dt=DT,
    initial_value=pos[0],
)

SCRIPT_DIR = Path(__file__).resolve().parent
plotjuggler = PlotJugglerUdpClient(
    save=True,
    dump_duration=SIMULATION_DURATION,
    output_dir=SCRIPT_DIR,
    time_base=TimeBase.RELATIVE,
    extra_fieldnames=[
        "position/true",
        "position/noisy",
        "position/filtered",
        "control/force",
        "noise/std",
        "filter/cutoff_hz",
        "filter/enabled",
    ],
)

for _ in range(SIMULATION_STEPS):
    kp = p.readUserDebugParameter(kp_param_id)
    ki = p.readUserDebugParameter(ki_param_id)
    kd = p.readUserDebugParameter(kd_param_id)
    target_x = p.readUserDebugParameter(target_x_param_id)
    noise_std = p.readUserDebugParameter(noise_std_param_id)
    filter_cutoff_hz = p.readUserDebugParameter(filter_cutoff_param_id)
    filter_enabled = p.readUserDebugParameter(filter_enabled_param_id) >= 0.5

    pid.set_gains(kp, ki, kd)
    low_pass_filter.set_cutoff(filter_cutoff_hz)

    pos, _ = p.getBasePositionAndOrientation(box)
    true_x = pos[0]
    noisy_x = true_x + random.gauss(0.0, noise_std)
    filtered_x = low_pass_filter.update(noisy_x)
    controller_feedback = filtered_x if filter_enabled else noisy_x

    force = pid.update(target_x, controller_feedback)
    plotjuggler.send(
        target_x,
        controller_feedback,
        extra_data={
            "position/true": true_x,
            "position/noisy": noisy_x,
            "position/filtered": filtered_x,
            "control/force": force,
            "noise/std": noise_std,
            "filter/cutoff_hz": filter_cutoff_hz,
            "filter/enabled": int(filter_enabled),
        },
    )

    p.applyExternalForce(
        objectUniqueId=box,
        linkIndex=-1,
        forceObj=[force, 0, 0],
        posObj=pos,
        flags=p.WORLD_FRAME,
    )

    p.stepSimulation()
    time.sleep(DT)

plotjuggler.close_csv()
p.disconnect()
