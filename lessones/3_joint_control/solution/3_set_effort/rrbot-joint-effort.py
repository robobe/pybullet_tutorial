import logging
import math
import os
import time

import pybullet_data
import structlog

import pybullet as p

JOINT_INDEX = 0
TIME_STEP = 1.0 / 240.0
MAX_TORQUE = 50.0
JOINT_POSITION_INDEX = 0
JOINT_VELOCITY_INDEX = 1
JOINT_TORQUE_INDEX = 3

logging.basicConfig(level=logging.INFO, format="%(message)s")

structlog.configure(
    processors=[
        structlog.processors.add_log_level,
        structlog.processors.TimeStamper(fmt="iso"),
        structlog.dev.ConsoleRenderer(),  # <-- colored output
    ],
    logger_factory=structlog.stdlib.LoggerFactory(),
)

log = structlog.get_logger()

# setup
physicsClient = p.connect(p.GUI)
p.resetSimulation()  # type: ignore
p.setGravity(gravX=0, gravY=0, gravZ=-9.8)
p.setAdditionalSearchPath(path=pybullet_data.getDataPath())
plane = p.loadURDF("plane.urdf")


# add current file location and its 'urdf' subfolder to search paths
cwd = os.path.dirname(os.path.abspath(__file__))
p.setAdditionalSearchPath(os.path.join(cwd, "urdf"))
# load URDF
robot = p.loadURDF("rrbot.urdf", basePosition=[0, 0, 0.5], useFixedBase=True)

# Disable the default motor so we can directly apply torque.
p.setJointMotorControl2(
    bodyIndex=robot,
    jointIndex=JOINT_INDEX,
    controlMode=p.VELOCITY_CONTROL,
    force=0.0,
)

# declare parameters for outer-loop position tracking with effort command
pose_param_id = p.addUserDebugParameter(
    "target position (rad)", -math.pi, math.pi, 0.0, physicsClient
)
kp_param_id = p.addUserDebugParameter(
    "Kp (Nm per rad error)", 0.0, 100.0, 20.0, physicsClient
)
kd_param_id = p.addUserDebugParameter(
    "Kd (Nms per rad velocity)", 0.0, 20.0, 2.0, physicsClient
)
max_torque_param_id = p.addUserDebugParameter(
    "max torque (Nm)", 0.1, 200.0, MAX_TORQUE, physicsClient
)

p.setTimeStep(TIME_STEP)
p.setRealTimeSimulation(False)

try:
    while True:
        target_pos = p.readUserDebugParameter(pose_param_id)
        kp = p.readUserDebugParameter(kp_param_id)
        kd = p.readUserDebugParameter(kd_param_id)
        max_torque = p.readUserDebugParameter(max_torque_param_id)

        js = p.getJointState(robot, JOINT_INDEX)
        current_pos = js[JOINT_POSITION_INDEX]
        current_vel = js[JOINT_VELOCITY_INDEX]
        pos_error = target_pos - current_pos

        # PD torque control: proportional position correction + velocity damping.
        p_term = kp * pos_error
        d_term = -kd * current_vel
        commanded_torque = p_term + d_term
        # clip torque to max limits
        commanded_torque = max(-max_torque, min(max_torque, commanded_torque))

        p.setJointMotorControl2(
            bodyIndex=robot,
            jointIndex=JOINT_INDEX,
            controlMode=p.TORQUE_CONTROL,
            force=commanded_torque,
        )

        p.stepSimulation()
        keys = p.getKeyboardEvents()
        joint_state = p.getJointState(robot, JOINT_INDEX)
        joint_velocity = joint_state[JOINT_VELOCITY_INDEX]
        applied_motor_torque = joint_state[JOINT_TORQUE_INDEX]
        log.info(
            "joint_tracking",
            joint=JOINT_INDEX,
            target_position_rad=target_pos,
            current_position_rad=current_pos,
            current_velocity_rad_s=current_vel,
            position_error_rad=pos_error,
            p_term_nm=p_term,
            d_term_nm=d_term,
            commanded_torque_nm=commanded_torque,
            applied_motor_torque_nm=applied_motor_torque,
            measured_velocity_rad_s=joint_velocity,
        )

        # 27 = ESC
        if 27 in keys and keys[27] & p.KEY_WAS_TRIGGERED:
            print("ESC pressed, exiting...")
            break

        time.sleep(TIME_STEP)

finally:
    p.disconnect()
