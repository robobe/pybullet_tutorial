import pybullet as p
import pybullet_data
import time

m = 1.0
k = 20.0
b = 1.0
dt = 1.0 / 240.0

target_x = 1.0

# PID gains
Kp = 80.0
Ki = 0.0
Kd = 15.0

integral = 0.0
prev_error = 0.0

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, 0)
p.setTimeStep(dt)

p.loadURDF("plane.urdf")

mass_id = p.loadURDF(
    "cube_small.urdf",
    basePosition=[0.0, 0, 0.5],
    useFixedBase=False
)
p.changeDynamics(mass_id, -1, mass=m, lateralFriction=0)

# wall_id = p.loadURDF(
#     "cube_small.urdf",
#     basePosition=[0, -0.5, 0.5],
#     useFixedBase=True
# )

while True:
    pos, _ = p.getBasePositionAndOrientation(mass_id)
    vel, _ = p.getBaseVelocity(mass_id)

    x = pos[0]
    x_dot = vel[0]

    error = target_x - x
    integral += error * dt
    derivative = (error - prev_error) / dt
    prev_error = error

    # PID force
    u_pid = Kp * error + Ki * integral + Kd * derivative

    # spring-damper force
    f_spring = -k * x
    f_damper = -b * x_dot

    force_x = f_spring + f_damper + u_pid

    p.applyExternalForce(
        objectUniqueId=mass_id,
        linkIndex=-1,
        forceObj=[force_x, 0, 0],
        posObj=pos,
        flags=p.WORLD_FRAME
    )

    p.stepSimulation()
    time.sleep(dt)