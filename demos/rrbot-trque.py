import typeings.pybullet as p
import time
import pybullet_data

p.connect(p.GUI)
p.setGravity(0, 0, -9.8)

# Default assets
p.setAdditionalSearchPath(pybullet_data.getDataPath())
plane_id = p.loadURDF("plane.urdf")

# Your URDFs
p.setAdditionalSearchPath("/workspaces/pybullet_tutorial/demos/urdf")
robot_id = p.loadURDF("rrbot.urdf", useFixedBase=True)

num_joints = p.getNumJoints(robot_id)

# Disable default motors
for j in range(num_joints):
    p.setJointMotorControl2(robot_id, j, p.VELOCITY_CONTROL, force=0)

# Optional: reduce damping so it moves easier
# for j in range(num_joints):
#     p.changeDynamics(robot_id, j, linearDamping=0, angularDamping=0)

# Start slightly away from upright so gravity is non-zero
p.resetJointState(robot_id, 0, 0.2)

# Torque slider ONLY for joint 1 (revolute)
MAX_TORQUE = 20.0
slider_id = p.addUserDebugParameter("joint1_torque", -MAX_TORQUE, MAX_TORQUE, 0.0)

while p.isConnected():
    torque = p.readUserDebugParameter(slider_id)

    p.setJointMotorControl2(
        robot_id,
        0,  # control joint1 only
        controlMode=p.TORQUE_CONTROL,
        force=torque
    )

    pos, vel, _, _ = p.getJointState(robot_id, jointIndex=0)
    print(pos)
    p.stepSimulation()
    time.sleep(1./240.)
