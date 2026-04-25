import time
import numpy as np
import cvxpy as cp
import pybullet as p
import pybullet_data
from scipy.signal import cont2discrete

# -------------------------
# System parameters
# -------------------------
m = 1.0
k = 20.0
b = 1.0
dt = 1.0 / 240.0
target_x = 1.0

# Continuous state-space
A = np.array([
    [0.0, 1.0],
    [-k / m, -b / m],
])

B = np.array([
    [0.0],
    [1.0 / m],
])

# Discrete model: x[k+1] = Ad x[k] + Bd u[k]
Ad, Bd, _, _, _ = cont2discrete((A, B, np.eye(2), np.zeros((2, 1))), dt)

# -------------------------
# MPC setup
# -------------------------
N = 30

Q = np.diag([80.0, 20.0])
R = np.array([[0.5]])

u_min = -50.0
u_max = 50.0

x_var = cp.Variable((2, N + 1))
u_var = cp.Variable((1, N))
x0_param = cp.Parameter(2)

cost = 0
constraints = [x_var[:, 0] == x0_param]

for i in range(N):
    cost += cp.quad_form(x_var[:, i], Q)
    cost += cp.quad_form(u_var[:, i], R)

    constraints += [
        x_var[:, i + 1] == Ad @ x_var[:, i] + Bd.flatten() * u_var[:, i],
        u_var[:, i] >= u_min,
        u_var[:, i] <= u_max,
    ]

cost += cp.quad_form(x_var[:, N], Q)

problem = cp.Problem(cp.Minimize(cost), constraints)

def mpc_control(state_error):
    x0_param.value = state_error.flatten()

    problem.solve(
        solver=cp.OSQP,
        warm_start=True,
        verbose=False,
    )

    if problem.status not in [cp.OPTIMAL, cp.OPTIMAL_INACCURATE]:
        return 0.0

    return float(u_var.value[0, 0])

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
    useFixedBase=False,
)

p.changeDynamics(
    mass_id,
    -1,
    mass=m,
    lateralFriction=0.0,
    linearDamping=0.0,
    angularDamping=0.0,
)

target_marker = p.loadURDF(
    "cube_small.urdf",
    basePosition=[target_x, -0.5, 0.5],
    useFixedBase=True,
)
p.changeVisualShape(target_marker, -1, rgbaColor=[0, 1, 0, 1])

# -------------------------
# Simulation loop
# -------------------------
while True:
    pos, _ = p.getBasePositionAndOrientation(mass_id)
    vel, _ = p.getBaseVelocity(mass_id)

    x = pos[0]
    x_dot = vel[0]

    state_error = np.array([
        [x - target_x],
        [x_dot],
    ])

    # MPC computes feedback force
    u_mpc = mpc_control(state_error)

    # Feedforward force to hold target against spring
    u_ff = k * target_x

    u = u_ff + u_mpc

    # Physical spring-damper forces
    f_spring = -k * x
    f_damper = -b * x_dot

    force_x = f_spring + f_damper + u

    p.applyExternalForce(
        objectUniqueId=mass_id,
        linkIndex=-1,
        forceObj=[force_x, 0, 0],
        posObj=pos,
        flags=p.WORLD_FRAME,
    )

    p.stepSimulation()
    time.sleep(dt)