import pybullet as p
import pybullet_data
import time
import os
import math
import logging
import structlog

JOINT_INDEX = 0

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
p.resetSimulation() # type: ignore
p.setGravity(gravX=0, gravY=0, gravZ=-9.8)
p.setAdditionalSearchPath(path=pybullet_data.getDataPath())
plane = p.loadURDF("plane.urdf")




# add current file location and its 'urdf' subfolder to search paths
cwd = os.path.dirname(os.path.abspath(__file__))
p.setAdditionalSearchPath(os.path.join(cwd, "urdf"))
# load URDF
robot = p.loadURDF("rrbot.urdf", basePosition=[0, 0, 0.5], useFixedBase=True)

# declare parameters
pose_param_id = p.addUserDebugParameter("position (rad)", 0, 2*math.pi, 0, physicsClient)

# control joint 0 to position 45 degree
p.setJointMotorControl2(robot, JOINT_INDEX, p.POSITION_CONTROL, targetPosition=math.pi/4, force=200)
# read current position of joint 0
js = p.getJointState(robot, JOINT_INDEX)
joint0_pos = js[0]  # position (rad)
log.info("joint_position", joint=JOINT_INDEX, position_rad=joint0_pos, position_deg=joint0_pos * 180.0 / math.pi)

# read positions of all joints
num_joints = p.getNumJoints(robot)
all_positions = [p.getJointState(robot, i)[0] for i in range(num_joints)]
log.info("all_joint_positions", positions=all_positions)
p.setRealTimeSimulation(True)
try:
    while True:
        current_pose = p.readUserDebugParameter(pose_param_id)
        p.setJointMotorControl2(robot, JOINT_INDEX, p.POSITION_CONTROL, targetPosition=current_pose, force=200)
        keys = p.getKeyboardEvents()
        js = p.getJointState(robot, JOINT_INDEX)
        joint0_pos = js[0]  # position (rad)
        log.info("joint_position", joint=JOINT_INDEX, position_rad=joint0_pos, position_deg=joint0_pos * 180.0 / math.pi)
        # 27 = ESC
        if 27 in keys and keys[27] & p.KEY_WAS_TRIGGERED:
            print("ESC pressed, exiting...")
            break

        time.sleep(1/240)

finally:
    p.disconnect()
