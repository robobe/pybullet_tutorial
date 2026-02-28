import gymnasium as gym
from gymnasium import spaces
import numpy as np
import pybullet as p
import pybullet_data
import time


class MinimalBulletEnv(gym.Env):
    metadata = {"render_modes": ["human", "none"]}

    def __init__(self, render_mode="none"):
        super().__init__()
        if render_mode not in self.metadata["render_modes"]:
            raise ValueError(f"Unsupported render_mode: {render_mode}")
        self.render_mode = render_mode

        self.grid_size = 11
        self.observation_space = spaces.Discrete(self.grid_size)
        self.action_space = spaces.Discrete(2)  # 0 left, 1 right

        self.max_steps = 200
        self.steps = 0

        self.client = None
        self.body = None

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        self.steps = 0

        if self.client is None:
            self.client = p.connect(p.GUI if self.render_mode == "human" else p.DIRECT)

        p.resetSimulation()
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        p.setTimeStep(1.0 / 240.0)

        p.loadURDF("plane.urdf")

        # ✅ create a simple box (very stable)
        half = 0.04
        col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[half, half, half])
        vis = p.createVisualShape(p.GEOM_BOX, halfExtents=[half, half, half])
        self.body = p.createMultiBody(
            baseMass=1.0,
            baseCollisionShapeIndex=col,
            baseVisualShapeIndex=vis,
            basePosition=[0, 0, half + 0.01],
        )

        # friction so it doesn't drift forever, but not so high it sticks
        p.changeDynamics(self.body, -1, lateralFriction=0.6, rollingFriction=0.0, spinningFriction=0.0)

        return self._get_state(), {}

    def step(self, action):
        self.steps += 1
        assert self.action_space.contains(action)

        # push left/right in world frame
        force = 60.0 if action == 1 else -60.0
        p.applyExternalForce(
            self.body, -1,
            [force, 0, 0],
            [0, 0, 0],
            flags=p.WORLD_FRAME,
            physicsClientId=self.client
        )

        # simulate a bit so action takes effect
        for _ in range(40):
            p.stepSimulation()
            if self.render_mode == "human":
                time.sleep(1/240)

        x = p.getBasePositionAndOrientation(self.body)[0][0]
        state = self._get_state()

        terminated = x > 1.0          # reached goal on +x
        truncated = self.steps >= self.max_steps

        reward = 1.0 if terminated else -0.01
        return state, reward, terminated, truncated, {"x": x}

    def render(self):
        # GUI renders automatically
        pass

    def close(self):
        if self.client is not None:
            p.disconnect(self.client)
            self.client = None

    def _get_state(self):
        x = p.getBasePositionAndOrientation(self.body)[0][0]
        x = float(np.clip(x, -1.0, 1.0))
        bins = np.linspace(-1.0, 1.0, self.grid_size)
        s = int(np.digitize(x, bins) - 1)
        return int(np.clip(s, 0, self.grid_size - 1))


if __name__ == "__main__":
    env = MinimalBulletEnv(render_mode="human")
    s, _ = env.reset()
    for _ in range(50):
        a = env.action_space.sample()
        s, r, term, trunc, info = env.step(a)
        if term or trunc:
            break
    env.close()