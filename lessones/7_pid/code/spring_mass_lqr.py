import time
import numpy as np
import pybullet as p
import pybullet_data
from scipy.linalg import solve_continuous_are

# -------------------------
# System parameters
# -------------------------
m = 1.0
k = 20.0
b = 1.0
dt = 1.0 / 240.0

target_x = 1.0

# -------------------------
# State-space model
# -------------------------
A = np.array([
    [0.0, 1.0],
    [-k / m, -b / m]
])

B = np.array([
    [0.0],
    [1.0 / m]
])

# -------------------------
# LQR tuning
# -------------------------
Q = np.diag([100.0, 10.0])   # punish position error and velocity
R = np.array([[1.0]])        # punish force usage

Q = np.diag([300.0, 100.0])
R = np.array([[1.0]])

# Solve Riccati equation
P = solve_continuous_are(A, B, Q, R)

# LQR gain
K = np.linalg.inv(R) @ B.T @ P

print("LQR gain K =", K)

# -------------------------
# PyBullet setup
# -------------------------
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, 0)
p.setTimeStep(dt)

p.loadURDF("plane.urdf")

mass_id = p.loadURDF(
    "cube_small.urdf",
    basePosition=[0.0, 0.0, 0.5],
    useFixedBase=False
)

p.changeDynamics(
    mass_id,
    -1,
    mass=m,
    lateralFriction=0.0,
    linearDamping=0.0,
    angularDamping=0.0
)

# fixed wall / reference
wall_id = p.loadURDF(
    "cube_small.urdf",
    basePosition=[0.0, -0.5, 0.5],
    useFixedBase=True
)

# -------------------------
# Simulation loop
# -------------------------
while True:
    pos, _ = p.getBasePositionAndOrientation(mass_id)
    vel, _ = p.getBaseVelocity(mass_id)

    x = pos[0]
    x_dot = vel[0]

    # state error relative to target
    state_error = np.array([
        [x - target_x],
        [x_dot]
    ])

    # feedforward force to balance spring at target
    u_ff = k * target_x

    # LQR feedback
    u_lqr = -K @ state_error

    # total control force
    u = float(u_ff + u_lqr[0, 0])

    # physical spring-damper force
    f_spring = -k * x
    f_damper = -b * x_dot

    force_x = f_spring + f_damper + u

    p.applyExternalForce(
        objectUniqueId=mass_id,
        linkIndex=-1,
        forceObj=[force_x, 0, 0],
        posObj=pos,
        flags=p.WORLD_FRAME
    )

    p.stepSimulation()
    time.sleep(dt)