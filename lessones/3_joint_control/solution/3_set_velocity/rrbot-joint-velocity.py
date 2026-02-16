import logging
import math
import os
import time

import pybullet_data
import structlog

import pybullet as p

JOINT_INDEX = 0
TIME_STEP = 1.0 / 240.0
MAX_FORCE = 200.0

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

# declare parameters for outer-loop position tracking with velocity command
pose_param_id = p.addUserDebugParameter(
    "target position (rad)", -math.pi, math.pi, 0.0, physicsClient
)
kp_param_id = p.addUserDebugParameter(
    "Kp (rad/s per rad error)", 0.0, 20.0, 4.0, physicsClient
)
max_vel_param_id = p.addUserDebugParameter(
    "max velocity (rad/s)", 0.1, 10.0, 2.0, physicsClient
)

p.setTimeStep(TIME_STEP)
p.setRealTimeSimulation(False)

try:
    while True:
        target_pos = p.readUserDebugParameter(pose_param_id)
        kp = p.readUserDebugParameter(kp_param_id)
        max_velocity = p.readUserDebugParameter(max_vel_param_id)

        js = p.getJointState(robot, JOINT_INDEX)
        current_pos = js[0]
        pos_error = target_pos - current_pos

        # Outer-loop P controller: position error -> target velocity
        target_velocity = kp * pos_error
        target_velocity = max(-max_velocity, min(max_velocity, target_velocity))

        p.setJointMotorControl2(
            bodyIndex=robot,
            jointIndex=JOINT_INDEX,
            controlMode=p.VELOCITY_CONTROL,
            targetVelocity=target_velocity,
            force=MAX_FORCE,
        )

        p.stepSimulation()
        keys = p.getKeyboardEvents()
        joint_velocity = p.getJointState(robot, JOINT_INDEX)[1]
        log.info(
            "joint_tracking",
            joint=JOINT_INDEX,
            target_position_rad=target_pos,
            current_position_rad=current_pos,
            position_error_rad=pos_error,
            commanded_velocity_rad_s=target_velocity,
            measured_velocity_rad_s=joint_velocity,
        )

        # 27 = ESC
        if 27 in keys and keys[27] & p.KEY_WAS_TRIGGERED:
            print("ESC pressed, exiting...")
            break

        time.sleep(TIME_STEP)

finally:
    p.disconnect()
