import json
import socket
import time
import pybullet as p
import pybullet_data

KP = 20
KI = 0
KD = 0
TARGET_X = 2.0
PLOTJUGGLER_HOST = "127.0.0.1"
PLOTJUGGLER_PORT = 9870


class PID:
    def __init__(self, kp, ki, kd, dt):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt
        self.integral = 0
        self.prev_error = 0

    def set_gains(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd

    def update(self, target, current):
        error = target - current

        self.integral += error * self.dt
        derivative = (error - self.prev_error) / self.dt

        output = (
            self.kp * error +
            self.ki * self.integral +
            self.kd * derivative
        )

        self.prev_error = error
        return output


# --- Setup ---
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

dt = 1 / 240
p.setTimeStep(dt)
p.setGravity(0, 0, 0)  # no gravity → pure control problem

p.loadURDF("plane.urdf")

# Create a simple box
box = p.loadURDF("cube_small.urdf", basePosition=[0, 0, 0.1])

# PID
pid = PID(kp=KP, ki=KI, kd=KD, dt=dt)

target_x_param_id = p.addUserDebugParameter("Target X", 0, 2, TARGET_X)
kp_param_id = p.addUserDebugParameter("Kp", 0, 100, KP)
ki_param_id = p.addUserDebugParameter("Ki", 0, 100, KI)
kd_param_id = p.addUserDebugParameter("Kd", 0, 100, KD)

plotjuggler_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
plotjuggler_address = (PLOTJUGGLER_HOST, PLOTJUGGLER_PORT)

# --- Simulation loop ---
while True:
    kp = p.readUserDebugParameter(kp_param_id)
    ki = p.readUserDebugParameter(ki_param_id)
    kd = p.readUserDebugParameter(kd_param_id)
    target_x = p.readUserDebugParameter(target_x_param_id)
    pid.set_gains(kp, ki, kd)

    pos, _ = p.getBasePositionAndOrientation(box)
    x = pos[0]

    force = pid.update(target_x, x)
    plot_data = {
        "time": time.time(),
        "position/setpoint": target_x,
        "position/feedback": x,
    }
    plotjuggler_socket.sendto(
        json.dumps(plot_data).encode("utf-8"),
        plotjuggler_address,
    )

    # Apply force along X axis
    p.applyExternalForce(
        objectUniqueId=box,
        linkIndex=-1,
        forceObj=[force, 0, 0],
        posObj=pos,
        flags=p.WORLD_FRAME
    )

    p.stepSimulation()
    time.sleep(dt)
