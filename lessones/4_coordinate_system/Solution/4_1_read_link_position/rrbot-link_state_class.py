import pybullet as p
import pybullet_data
import time
import os
import math
import logging
import structlog
from dataclasses import dataclass
from typing import TypeAlias, cast

COLOR_BLACK = [0.0, 0.0, 0.0]
JOINT_INDEX = 0
TARGET_LINK_NAME = "link2"
MAX_FORCE = 200.0

# tuple type alias
Vec3: TypeAlias = tuple[float, float, float]
Quat4: TypeAlias = tuple[float, float, float, float]


@dataclass(frozen=True)
class ImuReading:
    position_world: Vec3
    orientation_quat_xyzw: Quat4
    orientation_euler_rpy_rad: Vec3


class ImuSensor:
    def __init__(self, body_id: int, link_name: str, physics_client_id: int):
        self.body_id = body_id
        self.link_name = link_name
        self.physics_client_id = physics_client_id
        self.link_index = self._find_link_index(link_name)

    def _find_link_index(self, link_name: str) -> int:
        for i in range(p.getNumJoints(self.body_id, self.physics_client_id)):
            current = p.getJointInfo(self.body_id, i, self.physics_client_id)[12].decode("utf-8")
            if current == link_name:
                return i
        raise RuntimeError(f"Link '{link_name}' not found in robot")

    def read(self) -> ImuReading:
        link_state = p.getLinkState(
            self.body_id,
            self.link_index,
            computeForwardKinematics=1,
            physicsClientId=self.physics_client_id,
        )
        link_world_pos = cast(Vec3, link_state[4])
        link_world_quat = cast(Quat4, link_state[5])
        link_world_euler = cast(
            Vec3, p.getEulerFromQuaternion(list(link_world_quat), self.physics_client_id)
        )
        return ImuReading(
            position_world=link_world_pos,
            orientation_quat_xyzw=link_world_quat,
            orientation_euler_rpy_rad=link_world_euler,
        )

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

imu = ImuSensor(body_id=robot, link_name=TARGET_LINK_NAME, physics_client_id=physicsClient)

# declare parameters
pose_param_id = p.addUserDebugParameter("position (rad)", 0, 2*math.pi, 0, physicsClient)
pitch_text_id = -1

p.setRealTimeSimulation(True)
try:
    while True:
        current_pose = p.readUserDebugParameter(pose_param_id)
        p.setJointMotorControl2(robot, JOINT_INDEX, p.POSITION_CONTROL, targetPosition=current_pose, force=MAX_FORCE)

        keys = p.getKeyboardEvents()

        imu_reading = imu.read()
        pitch_rad = imu_reading.orientation_euler_rpy_rad[1]
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

        log.info(
            "imu_reading",
            link_name=TARGET_LINK_NAME,
            orientation_quat_xyzw=imu_reading.orientation_quat_xyzw,
            orientation_euler_rpy_rad=imu_reading.orientation_euler_rpy_rad,
        )
       
        # 27 = ESC
        if 27 in keys and keys[27] & p.KEY_WAS_TRIGGERED:
            print("ESC pressed, exiting...")
            break

        time.sleep(1/240)

finally:
    p.disconnect()
