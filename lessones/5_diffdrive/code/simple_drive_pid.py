import math
import time
from dataclasses import dataclass

import pybullet as p
import pybullet_data

DT = 1.0 / 240.0
WHEEL_RADIUS = 0.05
TRACK_WIDTH = 0.30
MAX_TORQUE = 30.0

LEFT_WHEELS = [2, 5]
RIGHT_WHEELS = [3, 7]
STEERING_JOINTS = [4, 6]

# Pose controller gains (unicycle style)
K_RHO = 1.2
K_ALPHA = 4.0
K_BETA = -0.8

MAX_V_CMD = 1.2
MAX_W_CMD = 2.2

POS_TOL = 0.05      # meters
YAW_TOL = 0.06      # rad
HOLD_TIME = 0.3     # seconds within tolerance before done


@dataclass
class Pose2D:
    x: float
    y: float
    yaw: float


@dataclass
class Twist2D:
    linear_x: float   # m/s
    angular_z: float  # rad/s


def wrap_pi(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def get_pose(car_id: int) -> Pose2D:
    pos, orn = p.getBasePositionAndOrientation(car_id)
    yaw = p.getEulerFromQuaternion(orn)[2]
    return Pose2D(x=pos[0], y=pos[1], yaw=yaw)


def cmd_steering_zero(car_id: int) -> None:
    for j in STEERING_JOINTS:
        p.setJointMotorControl2(
            car_id,
            j,
            controlMode=p.POSITION_CONTROL,
            targetPosition=0.0,
            force=200.0,
        )


def twist_to_wheel_omegas(twist: Twist2D) -> tuple[float, float]:
    omega_r = (twist.linear_x + 0.5 * TRACK_WIDTH * twist.angular_z) / WHEEL_RADIUS
    omega_l = (twist.linear_x - 0.5 * TRACK_WIDTH * twist.angular_z) / WHEEL_RADIUS
    return omega_l, omega_r


def send_wheel_velocity_commands(car_id: int, omega_l: float, omega_r: float) -> None:
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


def cmd_diff_drive_from_twist(car_id: int, twist: Twist2D) -> None:
    omega_l, omega_r = twist_to_wheel_omegas(twist)
    send_wheel_velocity_commands(car_id, omega_l=omega_l, omega_r=omega_r)


class PoseToTwistController:
    """
    Input: current pose + target pose
    Output: Twist command (linear_x, angular_z)
    """

    def update(self, current: Pose2D, target: Pose2D) -> Twist2D:
        dx = target.x - current.x
        dy = target.y - current.y

        rho = math.hypot(dx, dy)
        goal_heading = math.atan2(dy, dx) if rho > 1e-9 else current.yaw

        alpha = wrap_pi(goal_heading - current.yaw)
        beta = wrap_pi(target.yaw - goal_heading)

        linear_x = K_RHO * rho
        angular_z = K_ALPHA * alpha + K_BETA * beta

        # If heading is far off, slow down forward motion to avoid arc overshoot.
        if abs(alpha) > 0.7:
            linear_x *= 0.35

        linear_x = clamp(linear_x, -MAX_V_CMD, MAX_V_CMD)
        angular_z = clamp(angular_z, -MAX_W_CMD, MAX_W_CMD)

        return Twist2D(linear_x=linear_x, angular_z=angular_z)


def hold_stop(car_id: int, seconds: float) -> None:
    steps = int(seconds / DT)
    for _ in range(steps):
        cmd_steering_zero(car_id)
        cmd_diff_drive_from_twist(car_id, Twist2D(linear_x=0.0, angular_z=0.0))
        p.stepSimulation()
        time.sleep(DT)


def drive_to_pose(car_id: int, target: Pose2D, timeout_s: float = 15.0) -> None:
    controller = PoseToTwistController()

    stable_time = 0.0
    elapsed = 0.0

    while elapsed < timeout_s:
        pose = get_pose(car_id)
        pos_err = math.hypot(target.x - pose.x, target.y - pose.y)
        yaw_err = abs(wrap_pi(target.yaw - pose.yaw))

        twist_cmd = controller.update(pose, target)

        cmd_steering_zero(car_id)
        cmd_diff_drive_from_twist(car_id, twist_cmd)

        p.stepSimulation()
        time.sleep(DT)

        if pos_err < POS_TOL and yaw_err < YAW_TOL:
            stable_time += DT
            if stable_time >= HOLD_TIME:
                break
        else:
            stable_time = 0.0

        elapsed += DT

    hold_stop(car_id, 0.3)


def main() -> None:
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)

    p.loadURDF("plane.urdf")
    car = p.loadURDF("racecar/racecar.urdf", basePosition=[0, 0, 0.2])

    hold_stop(car, 1.0)

    # Example: go to a pose 3 m forward and face +X
    drive_to_pose(car, Pose2D(x=3.0, y=0.0, yaw=0.0))

    # Example: return near origin and face opposite direction
    drive_to_pose(car, Pose2D(x=0.0, y=0.0, yaw=math.pi))

    hold_stop(car, 1.0)

    while p.isConnected():
        p.stepSimulation()
        time.sleep(DT)


if __name__ == "__main__":
    main()
