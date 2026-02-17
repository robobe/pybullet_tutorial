# Coordinate system

## Position and Quaternion
[]()

## Euler and Gimbal lock 
[]()

## World Coordinate

## Robot Coordinate

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