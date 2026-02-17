import pybullet as p
import pybullet_data
import time
import os
import math
import logging
import structlog
from typing import TypeAlias, cast

COLOR_BLACK = [0.0, 0.0, 0.0]
JOINT_INDEX = 0
TARGET_LINK_NAME = "link2"
MAX_FORCE = 200.0

# tuple type alias
Vec3: TypeAlias = tuple[float, float, float]
Quat4: TypeAlias = tuple[float, float, float, float]

# region logging
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
# endregion logging

# setup
physicsClient = p.connect(p.GUI)
p.resetSimulation() # type: ignore
p.setGravity(gravX=0, gravY=0, gravZ=-9.8)
p.setAdditionalSearchPath(path=pybullet_data.getDataPath())
plane = p.loadURDF("plane.urdf")


# add current file location and its 'urdf' subfolder to search paths
cwd = os.path.dirname(os.path.abspath(__file__))
p.setAdditionalSearchPath(os.path.join(cwd, "urdf"))
# load URDF
robot = p.loadURDF("rrbot.urdf", basePosition=[0, 0, 0.5], useFixedBase=True)

# Map link name -> link index (joint index for child link in PyBullet).
link2_index = -1
for i in range(p.getNumJoints(robot, physicsClient)):
    link_name = p.getJointInfo(robot, i, physicsClient)[12].decode("utf-8")
    if link_name == TARGET_LINK_NAME:
        link2_index = i
        break

if link2_index < 0:
    raise RuntimeError(f"Link '{TARGET_LINK_NAME}' not found in robot")

# declare parameters
pose_param_id = p.addUserDebugParameter("position (rad)", 0, 2*math.pi, 0, physicsClient)
pitch_text_id = -1

p.setRealTimeSimulation(True)
try:
    while True:
        current_pose = p.readUserDebugParameter(pose_param_id)
        p.setJointMotorControl2(robot, JOINT_INDEX, p.POSITION_CONTROL, targetPosition=current_pose, force=MAX_FORCE)

        keys = p.getKeyboardEvents()

        link_state = p.getLinkState(
            robot,
            link2_index,
            computeForwardKinematics=1,
            physicsClientId=physicsClient,
        )
        link_world_pos = cast(Vec3, link_state[4])
        link_world_quat = cast(Quat4, link_state[5])
        link_world_euler = cast(
            Vec3, p.getEulerFromQuaternion(list(link_world_quat), physicsClient)
        )
        pitch_rad = link_world_euler[1]
        pitch_deg = math.degrees(pitch_rad)

        pitch_text_id = p.addUserDebugText(
            text=f"Pitch: {pitch_rad:.3f} rad ({pitch_deg:.1f} deg)",
            textPosition=[
                0.1,
                0.1,
                0.2
            ],
            textColorRGB=COLOR_BLACK,
            textSize=1.4,
            replaceItemUniqueId=pitch_text_id,
            physicsClientId=physicsClient,
        )
       
        # 27 = ESC
        if 27 in keys and keys[27] & p.KEY_WAS_TRIGGERED:
            print("ESC pressed, exiting...")
            break

        time.sleep(1/240)

finally:
    p.disconnect()
