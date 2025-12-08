import pybullet as p
import time
import pybullet_data

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
# Add gravity
p.setGravity(0, 0, -9.8)
plane_id = p.loadURDF("plane.urdf")
# cube_id = p.loadURDF("urdf/self_balance.urdf", [0, 0, 1])
robot = p.loadURDF("kuka_iiwa/model.urdf", useFixedBase=True)

# Run simulation loop
while True:
    p.stepSimulation()
    time.sleep(1./240.)