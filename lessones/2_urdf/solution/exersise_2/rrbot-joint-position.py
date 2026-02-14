import pybullet as p
import pybullet_data
import time
import os
import math

# setup
p.connect(p.GUI)
p.resetSimulation() # type: ignore
p.setGravity(gravX=0, gravY=0, gravZ=-9.8)
p.setAdditionalSearchPath(path=pybullet_data.getDataPath())
plane = p.loadURDF("plane.urdf")

# add current file location and its 'urdf' subfolder to search paths
cwd = os.path.dirname(os.path.abspath(__file__))
p.setAdditionalSearchPath(os.path.join(cwd, "urdf"))
# load URDF

robot = p.loadURDF("rrbot.urdf", basePosition=[0, 0, 0.5], useFixedBase=True)
# set link2 (joint index 1) to 45 degrees using position control
p.setJointMotorControl2(robot, 0, p.POSITION_CONTROL, targetPosition=math.pi/4, force=200)

p.setRealTimeSimulation(True)
try:
    while True:
        keys = p.getKeyboardEvents()

        # 27 = ESC
        if 27 in keys and keys[27] & p.KEY_WAS_TRIGGERED:
            print("ESC pressed, exiting...")
            break

        time.sleep(1/240)

finally:
    p.disconnect()
