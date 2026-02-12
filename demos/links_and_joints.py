import typeings.pybullet as p
import pybullet_data

# setup
p.connect(p.DIRECT)
p.resetSimulation()
p.setGravity(gravX=0, gravY=0, gravZ=-9.8)
p.setAdditionalSearchPath(path=pybullet_data.getDataPath())

# load URDF
robo = p.loadURDF(
    "franka_panda/panda.urdf", basePosition=[1, 0, 0.1], useFixedBase=True
)

# joints
dic_info = {
    0: "joint Index",  # starts at 0
    1: "joint Name",
    2: "joint Type",  # 0=revolute (rotational), 1=prismatic (sliding), 4=fixed
    3: "state vectorIndex",
    4: "velocity vectorIndex",
    5: "flags",  # nvm always 0
    6: "joint Damping",
    7: "joint Friction",  # coefficient
    8: "joint lowerLimit",  # min angle
    9: "joint upperLimit",  # max angle
    10: "joint maxForce",  # max force allowed
    11: "joint maxVelocity",  # max speed
    12: "link Name",  # child link connected to this joint
    13: "joint Axis",
    14: "parent FramePos",  # position
    15: "parent FrameOrn",  # orientation
    16: "parent Index",  # −1 = base
}
for i in range(p.getNumJoints(bodyUniqueId=robo)):
    joint_info = p.getJointInfo(bodyUniqueId=robo, jointIndex=i)
    print(f"--- JOINT {i} ---")
    print({dic_info[k]: joint_info[k] for k in dic_info.keys()})

# links
for i in range(p.getNumJoints(robo)):
    link_name = p.getJointInfo(robo, i)[12].decode("utf-8")  # field 12="link Name"
    dyn = p.getDynamicsInfo(robo, i)
    pos, orn, *_ = p.getLinkState(robo, i)
    dic_info = {"Mass": dyn[0], "Friction": dyn[1], "Position": pos, "Orientation": orn}
    print(f"--- LINK {i}: {link_name} ---")
    print(dic_info)
