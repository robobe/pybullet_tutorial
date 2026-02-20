# Q-learning

Learn q-learning by solve the frozen lake

```python
import gymnasium as gym
import random
import numpy as np


environment = gym.make("FrozenLake-v1", is_slippery=False, render_mode="human")
environment.reset()
environment.render()
```
---

## Reference
1. [ Q Learning From Scratch – Using Q Tables ](https://youtu.be/L-kQSpCJH4Q)
1. [Q-Learning Basic Reinforcement Learning Tutorials](https://www.youtube.com/playlist?list=PL58zEckBH8fBW_XLPtIPlQ-mkSNNx0tLS)
1. [ Q-Learning Tutorial 1: Train Gymnasium FrozenLake-v1 with Python Reinforcement Learning ](https://youtu.be/ZhoIgo3qqLU)
  - [github](https://github.com/johnnycode8/gym_solutions/blob/main/frozen_lake_q.py)
1. [Q-learning for beginners](https://medium.com/data-science/q-learning-for-beginners-2837b777741)