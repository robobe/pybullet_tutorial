# Coordinate system

## Quaternion
[youtube](https://youtu.be/zjMuIxRvygQ)

## Euler and Gimbal lock 
Euler angles are a way to describe 3D orientation using three sequential rotations around coordinate axes.

**What Each Angle Does**
- Roll (φ) → tilt left/right
- Pitch (θ) → nose up/down
- Yaw (ψ) → rotate left/right heading

**In a self-balancing robot**:
- Pitch is critical (balance axis)
- Yaw controls heading
- Roll is usually small unless terrain is uneven

[youtube](https://youtu.be/YXkra9M_a2k)

## World Coordinate
The world coordinate system is the global, fixed reference frame in which everything in your scene or simulation is defined.

**It does not move.**

---

## Robot Coordinate
The robot coordinate system is the local reference frame attached to the robot itself.

**It moves with the robot.**

> [!TIP]
> It is the coordinate system whose origin is fixed to the robot’s body (usually at its center).


| World Frame      | Robot Frame               |
| ---------------- | ------------------------- |
| Fixed in space   | Moves with robot          |
| Global reference | Local reference           |
| Like a map       | Like the robot’s own view |


---

### PyBullet method to use
- getLinkState
- getBasePositionAndOrientation
- getEulerFromQuaternion


### getLinkState

```python
p.getLinkState(robot, linkIndex, computeLinkVelocity=1)
```

| Index | Meaning                          |
| ----- | -------------------------------- |
| 0     | COM world position               |
| 1     | COM world orientation            |
| 2     | Local inertial frame position    |
| 3     | Local inertial frame orientation |
| 4     | Link frame world position        |
| 5     | Link frame world orientation     |
| 6     | Linear velocity (world)          |
| 7     | Angular velocity (world)         |


> [!TIP]
> COM -> Center Of Mass

> [!NOTE]
> ### Exercise 4_1
> Get link position and orientation using `getLinkState`
> Use the robot control position from exercise 3_2
> Convert the quaternion to euler
> use `addUserDebugText` to print pitch to simulation window
>
> ### Solution
> - [Check](Solution/4_1_read_link_position/rrbot-link_state.py)
> - [Check imu basic solution (implement imu sensor class that return orientation only)](Solution/4_1_read_link_position/rrbot-link_state_class.py)