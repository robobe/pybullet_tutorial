import gymnasium as gym
import random
import numpy as np


environment = gym.make("FrozenLake-v1", is_slippery=False, render_mode="human")
environment.reset()
environment.render()