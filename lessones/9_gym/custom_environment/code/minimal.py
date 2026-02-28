import gymnasium as gym
from gymnasium import spaces


class LineWorldEnv(gym.Env):
    metadata = {"render_modes": ["human", "ansi"]}

    def __init__(self, render_mode=None):
        super().__init__()

        self.size = 5
        self.max_steps = 20

        self.observation_space = spaces.Discrete(self.size)
        self.action_space = spaces.Discrete(2)

        self.state = 0
        self.steps = 0
        self.render_mode = render_mode

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        self.state = 0
        self.steps = 0

        if self.render_mode == "human":
            self.render()

        return self.state, {}

    def step(self, action):
        self.steps += 1

        if action == 0:
            self.state = max(0, self.state - 1)
        elif action == 1:
            self.state = min(self.size - 1, self.state + 1)

        terminated = self.state == self.size - 1
        truncated = self.steps >= self.max_steps

        reward = 1.0 if terminated else -0.01

        if self.render_mode == "human":
            self.render()

        return self.state, reward, terminated, truncated, {}

    def render(self):
        line = ["-"] * self.size
        line[self.state] = "A"
        line[self.size - 1] = "G"

        output = "|" + "|".join(line) + "|"

        if self.render_mode == "human":
            print(output)
        elif self.render_mode == "ansi":
            return output

    def close(self):
        pass

env = LineWorldEnv(render_mode="human")
state, _ = env.reset()

for _ in range(10):
    action = env.action_space.sample()
    state, reward, terminated, truncated, _ = env.step(action)
    if terminated or truncated:
        break