import math
from typing import Optional

import gymnasium as gym
from gymnasium import spaces
import numpy as np


class ContinuousCartPoleEnv(gym.Env):
    """
    Minimal CartPole with continuous action:
      action: Box([-1], [1]) -> force = action * force_mag
      obs: same 4D state as CartPole: [x, x_dot, theta, theta_dot]
    """
    metadata = {"render_modes": ["human", "none"], "render_fps": 50}

    def __init__(self, render_mode: str = "none", force_mag: float = 10.0, max_steps: int = 500):
        super().__init__()
        if render_mode not in self.metadata["render_modes"]:
            raise ValueError(f"Unsupported render_mode: {render_mode}")
        self.render_mode = render_mode

        # Action is continuous
        self.action_space = spaces.Box(low=-1.0, high=1.0, shape=(1,), dtype=np.float32)

        # Observation matches classic CartPole (unbounded in theory; we keep wide bounds)
        high = np.array([4.8, np.finfo(np.float32).max, 0.418, np.finfo(np.float32).max], dtype=np.float32)
        self.observation_space = spaces.Box(-high, high, dtype=np.float32)

        # Physics constants (classic CartPole)
        self.gravity = 9.8
        self.masscart = 1.0
        self.masspole = 0.1
        self.total_mass = self.masscart + self.masspole
        self.length = 0.5  # half the pole length
        self.polemass_length = self.masspole * self.length
        self.force_mag = float(force_mag)
        self.tau = 0.02  # seconds between state updates

        # Termination thresholds (classic CartPole)
        self.theta_threshold_radians = 12 * 2 * math.pi / 360
        self.x_threshold = 2.4

        self.max_steps = int(max_steps)
        self.steps = 0

        self.state: Optional[np.ndarray] = None

        # Simple text render (no pygame dependency)
        self._last_render_line = ""

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        self.steps = 0
        # small random initial state like classic env
        self.state = self.np_random.uniform(low=-0.05, high=0.05, size=(4,)).astype(np.float32)
        if self.render_mode == "human":
            self.render()
        return self.state.copy(), {}

    def step(self, action):
        assert self.state is not None, "Call reset() before step()."
        action = np.asarray(action, dtype=np.float32)
        assert self.action_space.contains(action), f"Invalid action: {action}"

        x, x_dot, theta, theta_dot = self.state

        # Map action -> force
        force = float(np.clip(action[0], -1.0, 1.0)) * self.force_mag
        costheta = math.cos(theta)
        sintheta = math.sin(theta)

        # Dynamics (standard CartPole equations)
        temp = (force + self.polemass_length * theta_dot * theta_dot * sintheta) / self.total_mass
        theta_acc = (self.gravity * sintheta - costheta * temp) / (
            self.length * (4.0 / 3.0 - self.masspole * costheta * costheta / self.total_mass)
        )
        x_acc = temp - self.polemass_length * theta_acc * costheta / self.total_mass

        # Euler integration
        x = x + self.tau * x_dot
        x_dot = x_dot + self.tau * x_acc
        theta = theta + self.tau * theta_dot
        theta_dot = theta_dot + self.tau * theta_acc

        self.state = np.array([x, x_dot, theta, theta_dot], dtype=np.float32)

        self.steps += 1

        terminated = bool(
            x < -self.x_threshold
            or x > self.x_threshold
            or theta < -self.theta_threshold_radians
            or theta > self.theta_threshold_radians
        )
        truncated = self.steps >= self.max_steps

        # Same reward style as CartPole: +1 per step until termination
        reward = 1.0 if not terminated else 1.0

        if self.render_mode == "human":
            self.render()

        return self.state.copy(), reward, terminated, truncated, {}

    def render(self):
        # Minimal console render: show x and theta
        if self.state is None:
            return
        x, _, theta, _ = self.state
        line = f"x={x:+.3f}  theta={theta:+.3f} rad"
        # overwrite same line
        print("\r" + line + " " * max(0, len(self._last_render_line) - len(line)), end="")
        self._last_render_line = line

    def close(self):
        # nothing to close for this minimal render
        pass