# Gymnasium types

- spaces.Discrete
- spaces.Box

Think of spaces as a contract between:

- The environment
- The agent
- The RL algorithm

They define:

“What kind of data is allowed here?"

## spaces.Discrete
A **finite set of integers**

```python
from gymnasium import spaces

spaces.Discrete(4)
```

```bash
0, 1, 2, 3
```

### usage

```python
self.action_space = spaces.Discrete(2)
```

Example:

- 0 = move left
- 1 = move right

## spaces.Box
A **continuous space** defined by lower and upper bounds.
Usually multi-dimensional (vectors, images, etc.).


```python
spaces.Box(
    low=-1.0,
    high=1.0,
    shape=(4,),
    dtype=float
)
```

```bash
[-1.0 ≤ x1 ≤ 1.0
 -1.0 ≤ x2 ≤ 1.0
 -1.0 ≤ x3 ≤ 1.0
 -1.0 ≤ x4 ≤ 1.0]
```

### usage

#### image

```python
spaces.Box(
    low=0,
    high=255,
    shape=(84, 84, 3),
    dtype=np.uint8
)
```


---

# Gymnasium custom environment

To make a **custom Gymnasium environment**, you create a class that subclasses `gymnasium.Env` and implement a small set of required methods + attributes



## __init__

- **self.action_space** (what actions the agent can take)
- **self.observation_space** (what observations/states look like)
- any config (max steps, physics objects, etc.)
- **optional**: metadata = {"render_modes": [...]}

```python
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
```

## reset(self, seed=None, options=None) -> (obs, info)

Called at the start of each episode.

Must:
- call super().reset(seed=seed) to set RNG
- reset your simulator/world variables
- return (obs, info) where:
    - obs matches observation_space
    - info is a dict (can be empty)

```python
def reset(self, *, seed=None, options=None):
    super().reset(seed=seed)
    self.state = 0
    self.steps = 0

    if self.render_mode == "human":
        self.render()

    return self.state, {}
```

## step(self, action) -> (obs, reward, terminated, truncated, info)

Called every time the agent acts.

Must return 5 values:
- obs (new observation)
- reward (float)
- terminated (True if episode ended “naturally” like success/failure)
- truncated (True if episode ended due to time limit / cutoff)
- info (dict)

```python
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
```

## render(self)
Only needed if you want visuals. In Gymnasium, many envs use render_mode="human" and render during step().


## close()
Free resources (PyBullet disconnect, windows, files, etc.)

```python
def close(self):
    pass
```

---

- check [minimal environment](code/minimal.py) to 1D line world


---

> [!NOTE]
> ### Exercise
> Try to implement q_learning for the 1D line world