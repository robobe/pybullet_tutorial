import typeings.pybullet as p
import pybullet_data

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -10)
planeId = p.loadURDF("plane.urdf")
boxId = p.loadURDF("cube.urdf", [0, 0, 1])

while p.isConnected():
    keys = p.getKeyboardEvents()

    if p.B3G_SPACE in keys and keys[p.B3G_SPACE] & p.KEY_WAS_TRIGGERED:
        print("Spacebar was pressed!")
        # Trigger an event, e.g., toggle a variable

    p.stepSimulation()
