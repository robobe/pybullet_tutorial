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


def drive_for_distance(car_id: int, distance_m: float, speed_mps: float) -> None:
    duration = abs(distance_m) / abs(speed_mps)
    v = speed_mps if distance_m >= 0 else -speed_mps
    steps = int(duration / DT)

    for _ in range(steps):
        cmd_steering_zero(car_id)
        cmd_diff_drive(car_id, v=v, w=0.0)
        p.stepSimulation()
        time.sleep(DT)


def hold_stop(car_id: int, seconds: float = 0.5) -> None:
    steps = int(seconds / DT)
    for _ in range(steps):
        cmd_steering_zero(car_id)
        cmd_diff_drive(car_id, v=0.0, w=0.0)
        p.stepSimulation()
        time.sleep(DT)


def main() -> None:
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)

    p.loadURDF("plane.urdf")
    car = p.loadURDF("racecar/racecar.urdf", basePosition=[0, 0, 0.2])

    hold_stop(car, 1.0)

    # Forward 2 meters
    drive_for_distance(car, distance_m=2.0, speed_mps=1.0)
    hold_stop(car, 0.5)

    # Backward 2 meters
    drive_for_distance(car, distance_m=-2.0, speed_mps=1.0)
    hold_stop(car, 1.0)

    while p.isConnected():
        p.stepSimulation()
        time.sleep(DT)


if __name__ == "__main__":
    main()
