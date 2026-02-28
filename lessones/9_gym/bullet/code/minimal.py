import time

import gymnasium as gym
from gymnasium import spaces
import numpy as np
import pybullet as p
import pybullet_data


class MinimalBulletEnv(gym.Env):
    metadata = {"render_modes": ["human", "none"]}

    def __init__(self, render_mode="none"):
        super().__init__()

        if render_mode not in self.metadata["render_modes"]:
            raise ValueError("Unsupported render mode")

        self.render_mode = render_mode

        # Discrete state (position bins)
        self.grid_size = 11
        self.observation_space = spaces.Discrete(self.grid_size)

        # 0 = left, 1 = right
        self.action_space = spaces.Discrete(2)

        self.max_steps = 200
        self.steps = 0

        self.client = None
        self.robot = None

    # -------------------------
    # RESET
    # -------------------------
    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)

        self.steps = 0

        # Connect to PyBullet
        if self.client is None:
            if self.render_mode == "human":
                self.client = p.connect(p.GUI)
            else:
                self.client = p.connect(p.DIRECT)

        p.resetSimulation()
        p.setGravity(0, 0, -9.81)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())

        # Load plane
        p.loadURDF("plane.urdf")

        # Create small cube
        self.robot = p.loadURDF("r2d2.urdf", [0, 0, 0.1])

        return self._get_state(), {}

    # -------------------------
    # STEP
    # -------------------------
    def step(self, action):
        self.steps += 1

        assert self.action_space.contains(action)

        force = 10 if action == 1 else -10

        p.applyExternalForce(
            self.robot,
            -1,
            [force, 0, 0],
            [0, 0, 0],
            p.LINK_FRAME,
        )

        for _ in range(10):
            p.stepSimulation()

        state = self._get_state()

        # Goal: reach right side
        pos, _ = p.getBasePositionAndOrientation(self.robot)
        terminated = pos[0] > 1.0
        truncated = self.steps >= self.max_steps

        reward = 1.0 if terminated else -0.01

        return state, reward, terminated, truncated, {}

    # -------------------------
    # RENDER
    # -------------------------
    def render(self):
        # For PyBullet, GUI already renders automatically
        pass

    # -------------------------
    # CLOSE
    # -------------------------
    def close(self):
        if self.client is not None:
            p.disconnect(self.client)
            self.client = None

    # -------------------------
    # HELPER
    # -------------------------
    def _get_state(self):
        pos, _ = p.getBasePositionAndOrientation(self.robot)
        x = np.clip(pos[0], -1, 1)

        # Discretize position into grid
        bins = np.linspace(-1, 1, self.grid_size)
        state = int(np.digitize(x, bins) - 1)
        state = np.clip(state, 0, self.grid_size - 1)

        return state
    
if __name__ == "__main__":
    env = MinimalBulletEnv(render_mode="human")
    state, _ = env.reset()

    for _ in range(100):
        action = env.action_space.sample()
        state, reward, terminated, truncated, _ = env.step(action)
        if terminated or truncated:
            break
        time.sleep(0.1)

    env.close()