import pybullet as p
import pybullet_data
import time

# -------------------------
# Parameters
# -------------------------
m = 1.0      # mass [kg]
k = 20.0     # spring stiffness
b = 1.0      # damping
dt = 1.0 / 240.0

# -------------------------
# PyBullet setup
# -------------------------
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, 0)
p.setTimeStep(dt)

p.loadURDF("plane.urdf")

# Create mass as a box
mass_id = p.loadURDF(
    "cube_small.urdf",
    basePosition=[1.0, 0, 0.5],
    useFixedBase=False
)

p.changeDynamics(mass_id, -1, mass=m)

# Visual fixed wall
wall_id = p.loadURDF(
    "cube_small.urdf",
    basePosition=[0, 0, 0.5],
    useFixedBase=True
)
p.changeVisualShape(wall_id, -1, rgbaColor=[1, 0, 0, 1])

# Initial condition
p.resetBasePositionAndOrientation(mass_id, [1.0, 0, 0.5], [0, 0, 0, 1])

# -------------------------
# Simulation loop
# -------------------------
while True:
    pos, orn = p.getBasePositionAndOrientation(mass_id)
    vel, ang_vel = p.getBaseVelocity(mass_id)

    x = pos[0]      # position from wall
    x_dot = vel[0]  # velocity in x direction

    # Spring-damper force
    force_x = -k * x - b * x_dot

    p.applyExternalForce(
        objectUniqueId=mass_id,
        linkIndex=-1,
        forceObj=[force_x, 0, 0],
        posObj=pos,
        flags=p.WORLD_FRAME
    )

    p.stepSimulation()
    time.sleep(dt)