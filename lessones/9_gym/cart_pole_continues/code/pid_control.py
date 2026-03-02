import time
import numpy as np
from cart_pole_con_env import ContinuousCartPoleEnv

# PID gains (start small and tune)
Kp_theta = 25.0
Kd_theta = 4.0
Ki_theta = 0.0

Kp_x = 1.0
Kd_x = 0.5

theta_int = 0.0

env = ContinuousCartPoleEnv(render_mode="human", force_mag=10.0, max_steps=500)

obs, _ = env.reset()
for _ in range(1000):
    x, x_dot, theta, theta_dot = obs

    # PID on theta (upright = 0)
    theta_int += theta * env.tau
    u_theta = Kp_theta * theta + Kd_theta * theta_dot + Ki_theta * theta_int

    # PD on x to keep cart near center
    u_x = Kp_x * x + Kd_x * x_dot

    # Combine (negative feedback)
    u = -(u_theta + u_x)

    # Convert to action in [-1, 1]
    action = np.clip(u / env.force_mag, -1.0, 1.0)

    obs, reward, terminated, truncated, _ = env.step([action])
    if terminated or truncated:
        obs, _ = env.reset()
        theta_int = 0.0

    time.sleep(0.02)

env.close()
