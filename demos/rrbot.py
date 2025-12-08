import pybullet as p
import time
import pybullet_data


p.connect(p.GUI)
p.setGravity(0, 0, -9.8)

# Default PyBullet assets
p.setAdditionalSearchPath(pybullet_data.getDataPath())
plane_id = p.loadURDF("plane.urdf")

p.setAdditionalSearchPath("/workspace/demos/urdf")
robot_id = p.loadURDF("rrbot.urdf", useFixedBase=True)
# joint information
joint_sliders = {}
num_joints = p.getNumJoints(robot_id)
for i in range(num_joints):
    info = p.getJointInfo(robot_id, i)
    j_name = info[1].decode("utf-8")
    j_type = info[2]

    slider_id = p.addUserDebugParameter(j_name, -3.14, 3.14, 0.0)
    joint_sliders[i] = slider_id


while p.isConnected():
    for j, slider_id in joint_sliders.items():
        target = p.readUserDebugParameter(slider_id)

        p.setJointMotorControl2(
            robot_id,
            j,
            controlMode=p.POSITION_CONTROL,
            targetPosition=target,
            force=5.0
        )
    p.stepSimulation()
    time.sleep(1./240.)

