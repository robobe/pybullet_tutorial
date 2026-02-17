import math
import time

import pybullet as p
import pybullet_data

DT = 1.0 / 240.0
WHEEL_RADIUS = 0.05
TRACK_WIDTH = 0.30
MAX_TORQUE = 30.0

LEFT_WHEELS = [2, 5]
RIGHT_WHEELS = [3, 7]
STEERING_JOINTS = [4, 6]

TURN_W = 1.2  # rad/s target yaw rate
YAW_TOL = 0.03  # rad


def wrap_pi(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def get_yaw(body_id: int) -> float:
    _, orn = p.getBasePositionAndOrientation(body_id)
    return p.getEulerFromQuaternion(orn)[2]


def cmd_steering_zero(car_id: int) -> None:
    for j in STEERING_JOINTS:
        p.setJointMotorControl2(
            car_id,
            j,
            controlMode=p.POSITION_CONTROL,
            targetPosition=0.0,
            force=200.0,
        )


def cmd_diff_drive(car_id: int, v: float, w: float) -> None:
    omega_r = (v + 0.5 * TRACK_WIDTH * w) / WHEEL_RADIUS
    omega_l = (v - 0.5 * TRACK_WIDTH * w) / WHEEL_RADIUS

    for j in LEFT_WHEELS:
        p.setJointMotorControl2(
            car_id,
            j,
            controlMode=p.VELOCITY_CONTROL,
            targetVelocity=omega_l,
            force=MAX_TORQUE,
        )

    for j in RIGHT_WHEELS:
        p.setJointMotorControl2(
            car_id,
            j,
            controlMode=p.VELOCITY_CONTROL,
            targetVelocity=omega_r,
            force=MAX_TORQUE,
        )


def hold_stop(car_id: int, seconds: float = 0.5) -> None:
    steps = int(seconds / DT)
    for _ in range(steps):
        cmd_steering_zero(car_id)
        cmd_diff_drive(car_id, v=0.0, w=0.0)
        p.stepSimulation()
        time.sleep(DT)


def turn_by_angle(car_id: int, delta_yaw: float) -> None:
    yaw0 = get_yaw(car_id)
    target = wrap_pi(yaw0 + delta_yaw)
    turn_dir = 1.0 if delta_yaw >= 0.0 else -1.0

    while True:
        yaw = get_yaw(car_id)
        err = wrap_pi(target - yaw)
        if abs(err) < YAW_TOL:
            break

        cmd_steering_zero(car_id)
        cmd_diff_drive(car_id, v=0.0, w=turn_dir * TURN_W)
        p.stepSimulation()
        time.sleep(DT)

    hold_stop(car_id, 0.4)


def main() -> None:
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)

    p.loadURDF("plane.urdf")
    car = p.loadURDF("racecar/racecar.urdf", basePosition=[0, 0, 0.2])

    hold_stop(car, 1.0)

    # Rotate 180 deg CCW (+pi)
    turn_by_angle(car, delta_yaw=math.pi)

    # Rotate 180 deg CW (-pi)
    turn_by_angle(car, delta_yaw=-math.pi)

    while p.isConnected():
        p.stepSimulation()
        time.sleep(DT)


if __name__ == "__main__":
    main()
