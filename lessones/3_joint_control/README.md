# Joint control

## Read joint info

- getJointState
- getNumJoints

### getNumJoints
> [!NOTE]
> PyBullet quickstart guide page 22

Return the number of joint in the robot

|   |   |   |   |
|---|---|---|---|
| required  | bodyUniqueId  | int  | the body unique id, as returned by loadURDF etc.  |
| optional  | physicsClientId  | int  | if you are connected to multiple servers, you can pick one.  |

```python
robot = p.loadURDF("rrbot.urdf", basePosition=[0, 0, 0.5], useFixedBase=True)
num_joints = p.getNumJoints(robot)
```

### getJointState

Retrun joint position, velocity, joint reaction forces and joint motor torque.

> [!NOTE]
> PyBullet quickstart guide page 28


|   |   |   |   |
|---|---|---|---|
| required  | bodyUniqueId  | int  | the body unique id, as returned by loadURDF etc.  |
| required  | jointIndex  | int  | link index in range [0..getNumJoints(bodyUniqueId)]  |
| required  | physicsClientId  | int  | if you are connected to multiple servers, you can pick one.  |


#### Output

| Index | Name                      | Type             | Description                                                         |
| ----- | ------------------------- | ---------------- | ------------------------------------------------------------------- |
| 0     | `jointPosition`           | float            | Current joint position (radians for revolute, meters for prismatic) |
| 1     | `jointVelocity`           | float            | Current joint velocity (rad/s or m/s)                               |
| 2     | `jointReactionForces`     | tuple (6 floats) | Reaction forces at joint (Fx, Fy, Fz, Mx, My, Mz)                   |
| 3     | `appliedJointMotorTorque` | float            | Motor torque applied during last step                               |


> [!NOTE]
> ## Exercise 3_1
> Move the link using `setJointMotorControl2` method and read the joint position using `getJointState`


---

## Position control

```python
import pybullet as p
import pybullet_data
import time
import os
import math

# setup
p.connect(p.GUI)
p.resetSimulation() # type: ignore
p.setGravity(gravX=0, gravY=0, gravZ=-9.8)
p.setAdditionalSearchPath(path=pybullet_data.getDataPath())
plane = p.loadURDF("plane.urdf")

# add current file location and its 'urdf' subfolder to search paths
cwd = os.path.dirname(os.path.abspath(__file__))
p.setAdditionalSearchPath(cwd)
# load URDF

robot = p.loadURDF("rrbot.urdf", basePosition=[0, 0, 0.5], useFixedBase=True)
# set link2 (joint index 1) to 45 degrees using position control
p.setJointMotorControl2(robot, 0, p.POSITION_CONTROL, targetPosition=math.pi/4, force=200)

p.setRealTimeSimulation(True)
try:
    while True:
        keys = p.getKeyboardEvents()

        # 27 = ESC
        if 27 in keys and keys[27] & p.KEY_WAS_TRIGGERED:
            print("ESC pressed, exiting...")
            break

        time.sleep(1/240)

finally:
    p.disconnect()

```

**Download code**

- [rrbot-joint-position.py](code/rrbot-joint-position.py)
- [urdf](code/rrbot.urdf)

---

> [!NOTE]
> ### Exercise 3_2
> Add `addUserDebugParameter` parameter slider to control joint position


![alt text](images/joint_position_with_parameter.png)