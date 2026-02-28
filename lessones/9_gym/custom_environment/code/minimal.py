import gymnasium as gym
from gymnasium import spaces

from typing import TypeAlias

ObsType: TypeAlias = int
ActionType: TypeAlias = int

ResetReturn: TypeAlias = tuple[ObsType, dict]
StepReturn: TypeAlias = tuple[ObsType, float, bool, bool, dict]

MOVE_LEFT: ActionType = 0
MOVE_RIGHT: ActionType = 1
GOAL_REWARD: float = 1.0
STEP_PENALTY: float = -0.01

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

    def reset(self, *, seed=None, options=None) -> ResetReturn:
        super().reset(seed=seed)
        self.state = 0
        self.steps = 0

        if self.render_mode == "human":
            self.render()

        return self.state, {}

    def step(self, action) -> StepReturn:
        self.steps += 1

        if action == MOVE_LEFT:
            self.state = max(0, self.state - 1)
        elif action == MOVE_RIGHT:
            self.state = min(self.size - 1, self.state + 1)

        # Check if the episode is terminated (reached the goal) 
        terminated = self.state == self.size - 1
        # or truncated (max steps reached)
        truncated = self.steps >= self.max_steps

        # calculate reward
        reward = GOAL_REWARD if terminated else STEP_PENALTY

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

if __name__ == "__main__":
    """
    create the environment and run a random policy for a few steps to see how it works
    """
    env = LineWorldEnv(render_mode="human")
    state, _ = env.reset()

    for _ in range(10):
        action = env.action_space.sample()
        state, reward, terminated, truncated, _ = env.step(action)
        if terminated or truncated:
            break
