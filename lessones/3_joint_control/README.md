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
--8<-- "code/rrbot-joint-position.py"
```