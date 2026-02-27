import pathlib
import pickle
import time

import numpy as np
import gymnasium as gym
from gymnasium import spaces

import pybullet as p
import pybullet_data


class BulletGridGoalEnv(gym.Env):
    """
    A tiny PyBullet env for tabular Q-learning:
    - A small cube on a plane.
    - Goal is a fixed point on the plane.
    - Observation: discretized (x,y) grid cell -> Discrete(N*N)
    - Actions: 4 moves (up/down/left/right) -> Discrete(4)
    """

    metadata = {"render_modes": ["human", "none"], "render_fps": 60}

    def __init__(
        self,
        render_mode="none",
        grid_size=11,          # N cells per axis (odd is nice: center at 0)
        world_range=1.0,       # coordinates in [-world_range, +world_range]
        action_step=0.08,      # how far to "nudge" per action (in meters-ish)
        max_steps=200,
        goal=(0.7, 0.7),
        goal_radius=0.10,
        time_step=1.0 / 120.0,
    ):
        super().__init__()

        self.render_mode = render_mode
        self.grid_size = int(grid_size)
        self.world_range = float(world_range)
        self.action_step = float(action_step)
        self.max_steps = int(max_steps)
        self.goal_xy = np.array(goal, dtype=np.float32)
        self.goal_radius = float(goal_radius)
        self.time_step = float(time_step)

        self.action_space = spaces.Discrete(4)  # 0:+x, 1:-x, 2:+y, 3:-y
        self.observation_space = spaces.Discrete(self.grid_size * self.grid_size)

        self._cid = None
        self._cube_id = None
        self._step_count = 0
        self._half = 0.04

        # Precompute grid edges for discretization
        self._edges = np.linspace(-self.world_range, self.world_range, self.grid_size + 1)

    def _connect(self):
        if self._cid is not None:
            return
        if self.render_mode == "human":
            self._cid = p.connect(p.GUI)
        else:
            self._cid = p.connect(p.DIRECT)

        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setTimeStep(self.time_step)
        p.setGravity(0, 0, -9.81)

    def _disconnect(self):
        if self._cid is not None:
            p.disconnect(self._cid)
            self._cid = None

    def close(self):
        self._disconnect()

    def _reset_world(self, start_xy):
        p.resetSimulation()
        p.setGravity(0, 0, -9.81)
        p.setTimeStep(self.time_step)

        # Plane
        p.loadURDF("plane.urdf")

        # Small cube (r2d2 is heavier / weird; use a cube)
        col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[self._half, self._half, self._half])
        vis = p.createVisualShape(p.GEOM_BOX, halfExtents=[self._half, self._half, self._half], rgbaColor=[0.2, 0.6, 1.0, 1.0])
        mass = 1.0
        self._cube_id = p.createMultiBody(
            baseMass=mass,
            baseCollisionShapeIndex=col,
            baseVisualShapeIndex=vis,
            basePosition=[float(start_xy[0]), float(start_xy[1]), self._half + 0.001],
        )

        # Add a bit of friction so it doesn't slide forever
        p.changeDynamics(self._cube_id, -1, lateralFriction=1.0, rollingFriction=0.02, spinningFriction=0.02)

        # (Optional) Visual marker for goal in GUI
        if self.render_mode == "human":
            goal_vis = p.createVisualShape(p.GEOM_SPHERE, radius=self.goal_radius, rgbaColor=[0.2, 1.0, 0.2, 0.6])
            p.createMultiBody(
                baseMass=0.0,
                baseVisualShapeIndex=goal_vis,
                basePosition=[float(self.goal_xy[0]), float(self.goal_xy[1]), 0.03],
            )

    def _get_xy(self):
        pos, _orn = p.getBasePositionAndOrientation(self._cube_id)
        return np.array([pos[0], pos[1]], dtype=np.float32)

    def _clip_xy(self, xy):
        return np.clip(xy, -self.world_range, self.world_range)

    def _xy_to_state(self, xy):
        # Convert continuous xy into grid cell index
        x, y = float(xy[0]), float(xy[1])

        # Find bin index [0..grid_size-1]
        ix = int(np.clip(np.digitize([x], self._edges)[0] - 1, 0, self.grid_size - 1))
        iy = int(np.clip(np.digitize([y], self._edges)[0] - 1, 0, self.grid_size - 1))

        return iy * self.grid_size + ix  # row-major

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        self._connect()

        self._step_count = 0

        # Start near (-0.7, -0.7) with a little randomness
        start = np.array([-0.7, -0.7], dtype=np.float32)
        start += self.np_random.uniform(low=-0.05, high=0.05, size=(2,)).astype(np.float32)
        start = self._clip_xy(start)

        self._reset_world(start_xy=start)

        obs = self._xy_to_state(self._get_xy())
        info = {"xy": self._get_xy().copy()}
        return obs, info

    def step(self, action):
        self._step_count += 1

        # Get current position
        xy = self._get_xy()

        # Decide target "nudge"
        dx, dy = 0.0, 0.0
        if action == 0:   # +x
            dx = self.action_step
        elif action == 1: # -x
            dx = -self.action_step
        elif action == 2: # +y
            dy = self.action_step
        elif action == 3: # -y
            dy = -self.action_step

        # Direct position update for gridworld-style transitions
        new_xy = self._clip_xy(xy + np.array([dx, dy], dtype=np.float32))
        p.resetBasePositionAndOrientation(
            self._cube_id,
            [float(new_xy[0]), float(new_xy[1]), self._half + 0.001],
            [0.0, 0.0, 0.0, 1.0],
        )
        p.resetBaseVelocity(self._cube_id, [0.0, 0.0, 0.0], [0.0, 0.0, 0.0])
        p.stepSimulation()

        new_xy = self._get_xy()
        new_xy = self._clip_xy(new_xy)

        # Reward: encourage getting closer, plus success bonus
        dist = float(np.linalg.norm(new_xy - self.goal_xy))
        terminated = dist < self.goal_radius
        truncated = self._step_count >= self.max_steps

        # Shaping reward: negative distance (small), success bonus
        reward = -0.05 * dist
        if terminated:
            reward += 1.0

        obs = self._xy_to_state(new_xy)
        info = {"xy": new_xy.copy(), "dist": dist}

        return obs, reward, terminated, truncated, info
    

import numpy as np

def q_learning_train(
    env,
    episodes=2000,
    alpha=0.15,      # learning rate
    gamma=0.99,      # discount
    eps_start=1.0,
    eps_end=0.05,
    eps_decay=0.999,
    max_steps=200,
):
    nS = env.observation_space.n
    nA = env.action_space.n
    Q = np.zeros((nS, nA), dtype=np.float32)

    eps = eps_start
    returns = []

    for ep in range(episodes):
        s, info = env.reset()
        total = 0.0

        for t in range(max_steps):
            # epsilon-greedy
            if np.random.rand() < eps:
                a = env.action_space.sample()
            else:
                a = int(np.argmax(Q[s]))

            s2, r, terminated, truncated, info = env.step(a)
            done = terminated or truncated

            # Q-learning update
            best_next = float(np.max(Q[s2]))
            Q[s, a] = (1 - alpha) * Q[s, a] + alpha * (r + (0.0 if done else gamma * best_next))

            s = s2
            total += r
            if done:
                break

        eps = max(eps_end, eps * eps_decay)
        returns.append(total)

        if (ep + 1) % 100 == 0:
            print(f"Episode {ep+1:4d} | eps={eps:.3f} | avg_return(last100)={np.mean(returns[-100:]):.3f}")

    return Q, returns

Q_TABLE_FILE = 'custom.pkl'
TRAIN_MODE = True

if __name__ == "__main__":
    q_table_path = pathlib.Path(__file__).parent / Q_TABLE_FILE

    if TRAIN_MODE:
        env = BulletGridGoalEnv(render_mode="none", grid_size=11, max_steps=200)
        Q, returns = q_learning_train(env, episodes=2000)
        env.close()

    
        f = open(q_table_path,'wb')
        pickle.dump(Q, f)
        f.close()
    # Watch the learned policy
    f = open(q_table_path, 'rb')
    Q = pickle.load(f)
    f.close()
    watch_env = BulletGridGoalEnv(render_mode="human", grid_size=11, max_steps=200)
    s, info = watch_env.reset()
    for _ in range(200):
        a = int(np.argmax(Q[s]))
        s, r, terminated, truncated, info = watch_env.step(a)
        if terminated or truncated:
            s, info = watch_env.reset()
        time.sleep(0.5)
    watch_env.close()
