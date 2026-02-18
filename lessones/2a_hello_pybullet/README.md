# Hello PyBullet

TODO: explain all topics

**Connect to Physics Server**
You start a physics world in:
- GUI mode (visual)
- DIRECT mode (headless, fast for RL)



**Configure Physics World**
- Typical setup includes:
- Gravity
- Time step
- Physics solver settings
- Real-time vs manual stepping

**Step the Simulation**
The physics engine advances the world by one time step.

- In real-time mode → automatic
- In manual mode → you explicitly step (used for RL)


---

## Simulation loop

The loop always follows this pattern:

```
while running:
    apply actions
    step physics
    read state
```

```python
import pybullet as p
import pybullet_data
import time

# 1️⃣ Connect to physics server (GUI mode)
p.connect(p.GUI)

# 2️⃣ Configure simulation
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
p.setTimeStep(1/240)

# 3️⃣ Load objects
plane = p.loadURDF("plane.urdf")
box = p.loadURDF("r2d2.urdf", [0, 0, 1])  # start 1 meter above ground

# 4️⃣ Simulation loop
while p.isConnected():

    # ---- Apply actions here (forces / motor commands) ----
    # Example: nothing applied, just gravity

    # ---- Advance physics by one step ----
    p.stepSimulation()

    # ---- Read state (optional) ----
    pos, orn = p.getBasePositionAndOrientation(box)
    print("Height:", round(pos[2], 3))

    # ---- Slow down to real-time ----
    time.sleep(1/240)

```