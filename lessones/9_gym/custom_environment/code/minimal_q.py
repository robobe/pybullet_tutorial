import pathlib
import pickle
import gymnasium as gym
from gymnasium import spaces
import numpy as np


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


# Q-learning implementation for LineWorldEnv
Q_TABLE_FILE = 'minimal.pkl'

def run(is_training=True, render=False):
    q_table_path = pathlib.Path(__file__).parent / Q_TABLE_FILE
    # Create environment 
    env = LineWorldEnv(render_mode='human' if render else None)
    state, _ = env.reset()

    # Initialize Q-table from file if not training, else create a new one
    if(is_training):
        q = np.zeros((env.size, env.action_space.n)) 
    else:
        if not q_table_path.exists():
            raise FileNotFoundError(f"Q-table not found: {q_table_path}")
        with open(q_table_path, "rb") as f:
            q = pickle.load(f)


    # Hyperparameters
    if(is_training):
        alpha = 0.1
        gamma = 0.99
        epsilon = 1.0
        epsilon_decay = 0.995
        epsilon_min = 0.05

    episodes = 2
    if is_training:
        episodes = 500

    
    for ep in range(episodes):
        
        state, _ = env.reset()
        if not is_training :
            print(f'start Episode: {ep} after reset')
        done = False
        rewards=0
        while not done:
            # epsilon-greedy
            if is_training and np.random.rand() < epsilon:
                action = env.action_space.sample()
            else:
                action = np.argmax(q[state])

            next_state, reward, terminated, truncated, _ = env.step(action)
            done = terminated or truncated

            # Q update on training mode
            if is_training:
                q[state, action] += alpha * (
                    reward + gamma * np.max(q[next_state]) - q[state, action]
                )   

            state = next_state
            rewards+=reward
            if not is_training :
                print(f'Episode: {ep}  Rewards: {rewards}')

        if is_training:
            epsilon = max(epsilon_min, epsilon * epsilon_decay)

    env.close()

    # Save Q-table if training
    if is_training:
        with open(q_table_path, "wb") as f:
            pickle.dump(q, f)


if __name__ == '__main__':
    run(is_training=True, render=False)
    run(is_training=False, render=True)
    
