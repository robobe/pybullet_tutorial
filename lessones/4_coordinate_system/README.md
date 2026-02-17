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