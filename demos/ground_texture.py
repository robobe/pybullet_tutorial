import pybullet as p
import pybullet_data
import time
import pathlib

# Initialize PyBullet
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Create ground plane
planeId = p.loadURDF("plane.urdf")

# Add texture to the ground
image_path = pathlib.Path(__file__).parent.joinpath("tarmac.png").as_posix()
textureId = p.loadTexture(image_path)
p.changeVisualShape(planeId, -1, textureUniqueId=textureId)

# Set gravity and run simulation
p.setGravity(0, 0, -9.81)

while True:
    p.stepSimulation()
    time.sleep(1./240.)
