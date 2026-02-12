import pybullet as p
import pybullet_data
import time

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -10)
planeId = p.loadURDF("plane.urdf")
boxId = p.loadURDF("cube.urdf", [0, 0, 1])

force_param_id = p.addUserDebugParameter("Force (N)", 0, 100, 0, physicsClient)


text_id = p.addUserDebugText(
    "Applied Force", 
    textPosition=[0, 0, 2],
    textColorRGB=[1, 0, 0],
    textSize=12,
    lifeTime=0,
    physicsClientId=physicsClient
)

while p.isConnected():
    current_force = p.readUserDebugParameter(force_param_id)
    p.applyExternalForce(boxId, -1, [current_force, 0, 0], [0, 0, 0], p.WORLD_FRAME, physicsClient)

    # if text_id is not None:
    #     p.removeUserDebugItem(text_id)

    text_string = f"Applied Force: {current_force:.2f} N"
    text_id = p.addUserDebugText(
        text_string,
        textPosition=[0, 0, 2],
        textColorRGB=[1, 0, 0],
        replaceItemUniqueId=text_id,
    )

    p.stepSimulation()
    time.sleep(1.0 / 240.0)
