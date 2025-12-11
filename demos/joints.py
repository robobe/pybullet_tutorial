import pybullet as p
import time
import pybullet_data

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
# Add gravity
p.setGravity(0, 0, -9.8)
plane_id = p.loadURDF("plane.urdf")
robot = p.loadURDF("kuka_iiwa/model.urdf", useFixedBase=True)

joint_index = 2  # pick a joint index
move = 0.001
for _ in range(1000):
    p.setJointMotorControl2(
        bodyUniqueId=robot,
        jointIndex=joint_index,
        controlMode=p.POSITION_CONTROL,
        targetPosition=1.0 + move,  # radians
        force=100  # maximum torque
    )
    move += 0.01
    pos, vel, reaction_forces, torque = p.getJointState(robot, 2)
    # print(f"Position={pos:.3f}, Velocity={vel:.3f}")
    p.stepSimulation()
    time.sleep(1/240.)
p.disconnect()
