import math
import time
import pybullet as p
import pybullet_data

DT = 1.0 / 240.0

# --- Diff-drive-ish parameters (tune if needed) ---
WHEEL_RADIUS = 0.05   # meters (approx; tune distance)
TRACK_WIDTH  = 0.30   # meters (effective left-right distance; tune turn angle)
MAX_TORQUE   = 30.0

# Motion plan
V_FWD = 1.0           # m/s
SIDE_LEN = 2.0        # meters (open-loop time = SIDE_LEN / V_FWD)
TURN_ANGLE = math.pi / 2
W_TURN = 1.2          # rad/s yaw rate while turning


def wrap_pi(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


def get_yaw(body_id: int) -> float:
    _, orn = p.getBasePositionAndOrientation(body_id)
    return p.getEulerFromQuaternion(orn)[2]


def cmd_steering_zero(car_id: int, steering_joints: list[int]):
    # keep steering centered
    for j in steering_joints:
        p.setJointMotorControl2(
            car_id, j,
            controlMode=p.POSITION_CONTROL,
            targetPosition=0.0,
            force=200.0
        )


def cmd_diff_drive(car_id: int, left_wheels: list[int], right_wheels: list[int], v: float, w: float):
    """
    v = linear velocity (m/s)
    w = angular velocity (rad/s)
    left_wheels, right_wheels = joint ids for each side

    TRACK_WIDTH: distance between left and right wheel contact lines.

    
    Diff-drive inverse kinematics:
      omega_R = (v + (b/2)*w)/r
      omega_L = (v - (b/2)*w)/r
    
    divide by wheel radius to get from linear velocity to angular velocity
    """


    omega_r = (v + 0.5 * TRACK_WIDTH * w) / WHEEL_RADIUS
    omega_l = (v - 0.5 * TRACK_WIDTH * w) / WHEEL_RADIUS

    for j in left_wheels:
        p.setJointMotorControl2(
            car_id, j,
            controlMode=p.VELOCITY_CONTROL,
            targetVelocity=omega_l,
            force=MAX_TORQUE,
        )
    for j in right_wheels:
        p.setJointMotorControl2(
            car_id, j,
            controlMode=p.VELOCITY_CONTROL,
            targetVelocity=omega_r,
            force=MAX_TORQUE,
        )


def step_for(car_id: int, left_wheels, right_wheels, steering_joints, duration: float, v: float, w: float):
    """
    move the car forward,
    keep the steering zero, and
    """
    steps = int(duration / DT)
    for _ in range(steps):
        cmd_steering_zero(car_id, steering_joints)
        cmd_diff_drive(car_id, left_wheels, right_wheels, v, w)
        p.stepSimulation()
        time.sleep(DT)


def turn_90_deg(car_id: int, left_wheels, right_wheels, steering_joints, direction: int):
    """
    direction: +1 left, -1 right
    Uses yaw feedback to stop around 90 degrees.
    """
    yaw0 = get_yaw(car_id)
    target = wrap_pi(yaw0 + direction * TURN_ANGLE)

    while True:
        yaw = get_yaw(car_id)
        err = wrap_pi(target - yaw)
        if abs(err) < 0.03:  # ~1.7 degrees
            break

        cmd_steering_zero(car_id, steering_joints)
        cmd_diff_drive(car_id, left_wheels, right_wheels, v=0.0, w=direction * W_TURN)
        p.stepSimulation()
        time.sleep(DT)

    # stop briefly
    step_for(car_id, left_wheels, right_wheels, steering_joints, 0.2, v=0.0, w=0.0)


def main():
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)

    plane = p.loadURDF("plane.urdf")

    car = p.loadURDF("racecar/racecar.urdf", basePosition=[0, 0, 0.2])
    p.resetBaseVelocity(car, [0, 0, 0], [0, 0, 0])

    # Increase friction to reduce slip (helps square look square)
    p.changeDynamics(plane, -1, lateralFriction=1.2)
    for j in range(p.getNumJoints(car)):
        p.changeDynamics(car, j, lateralFriction=1.2)

    # Racecar joint indices (standard pybullet_data racecar)
    # Wheels: 2,3,5,7
    # Steering: 4,6
    left_wheels = [2, 5]
    right_wheels = [3, 7]
    steering_joints = [4, 6]

    # Let settle
    step_for(car, left_wheels, right_wheels, steering_joints, 1.0, v=0.0, w=0.0)

    # the duration to drive forward for each side of the square, based on the desired forward speed and side length
    t_forward = SIDE_LEN / V_FWD

    # run for x loops
    LOOPS = 1
    for i in range(LOOPS):
        print(f"Side {i+1}: forward")
        step_for(car, left_wheels, right_wheels, steering_joints, t_forward, v=V_FWD, w=0.0)

        print(f"Side {i+1}: turn 90 deg left")
        turn_90_deg(car, left_wheels, right_wheels, steering_joints, direction=+1)

    # Stop
    step_for(car, left_wheels, right_wheels, steering_joints, 2.0, v=0.0, w=0.0)

    while p.isConnected():
        p.stepSimulation()
        time.sleep(DT)


if __name__ == "__main__":
    main()
