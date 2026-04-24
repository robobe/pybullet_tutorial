import time
from pathlib import Path
import pybullet as p
import pybullet_data
from plotjuggler_udp import PlotJugglerUdpClient


class PID:
    def __init__(self, kp, ki, kd, dt, force_limit, anti_windup=False):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt
        self.force_limit = force_limit
        self.anti_windup = anti_windup

        self.integral = 0.0
        self.prev_error = 0.0

    def update(self, target, x):
        error = target - x
        derivative = (error - self.prev_error) / self.dt

        # Candidate integral
        new_integral = self.integral + error * self.dt

        raw_force = (
            self.kp * error +
            self.ki * new_integral +
            self.kd * derivative
        )

        # Actuator saturation
        saturated_force = max(
            -self.force_limit,
            min(self.force_limit, raw_force)
        )

        if self.anti_windup:
            # Only integrate if output is not saturated
            # or if error helps move away from saturation
            is_saturated = raw_force != saturated_force

            if not is_saturated:
                self.integral = new_integral
            else:
                # Allow integral to unwind
                if saturated_force > 0 and error < 0:
                    self.integral = new_integral
                elif saturated_force < 0 and error > 0:
                    self.integral = new_integral
        else:
            # Normal PID: this causes windup
            self.integral = new_integral

        self.prev_error = error
        return saturated_force, raw_force, self.integral


# ------------------------
# PyBullet setup
# ------------------------

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

dt = 1 / 240
p.setTimeStep(dt)
p.setGravity(0, 0, 0)

p.loadURDF("plane.urdf")

box = p.loadURDF(
    "cube_small.urdf",
    basePosition=[0, 0, 0.1]
)

# Add friction/damping to make it easier to see
p.changeDynamics(box, -1, linearDamping=0.2)

target_x = 4.0

# Try first: anti_windup=False
pid = PID(
    kp=8.0,
    ki=12.0,
    kd=2.0,
    dt=dt,
    force_limit=2.0,
    anti_windup=False

)

# Change to True later:
# pid.anti_windup = True

script_dir = Path(__file__).resolve().parent
plotjuggler = PlotJugglerUdpClient(save=True, output_dir=script_dir.as_posix())

while True:
    pos, _ = p.getBasePositionAndOrientation(box)
    vel, _ = p.getBaseVelocity(box)

    x = pos[0]

    force, raw_force, integral = pid.update(target_x, x)
    plotjuggler.send(target_x, x)

    p.applyExternalForce(
        objectUniqueId=box,
        linkIndex=-1,
        forceObj=[force, 0, 0],
        posObj=pos,
        flags=p.WORLD_FRAME
    )

    p.stepSimulation()
    time.sleep(dt)
