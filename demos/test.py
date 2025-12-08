import gymnasium as gym
from gymnasium import spaces
from gymnasium.envs.registration import register
import os, pybullet as p, pybullet_data, math, numpy as np, random

MAX_EPISODE_LEN = 20*100

class PandaEnv(gym.Env):
    metadata = {"render_modes": ["human", "rgb_array"]}

    def __init__(self, use_gui=True):
        self.step_counter = 0
        self.use_gui = use_gui
        p.connect(p.GUI if use_gui else p.DIRECT)
        self._renderer = p.ER_BULLET_HARDWARE_OPENGL if use_gui else p.ER_TINY_RENDERER
        p.resetDebugVisualizerCamera(cameraDistance=1.5, cameraYaw=0, cameraPitch=-40, cameraTargetPosition=[0.55,-0.35,0.2])
        self.action_space = spaces.Box(low=-1.0, high=1.0, shape=(4,), dtype=np.float32)
        self.observation_space = spaces.Box(low=-1.0, high=1.0, shape=(5,), dtype=np.float32)

    def step(self, action):
        p.configureDebugVisualizer(p.COV_ENABLE_SINGLE_STEP_RENDERING)
        orientation = p.getQuaternionFromEuler([0., -math.pi, math.pi/2.])
        dv = 0.005
        dx, dy, dz, fingers = float(action[0])*dv, float(action[1])*dv, float(action[2])*dv, float(action[3])

        currentPose = p.getLinkState(self.pandaUid, 11)
        currentPosition = currentPose[0]
        newPosition = [currentPosition[0] + dx, currentPosition[1] + dy, currentPosition[2] + dz]
        jointPoses = p.calculateInverseKinematics(self.pandaUid, 11, newPosition, orientation)[0:7]
        p.setJointMotorControlArray(self.pandaUid, list(range(7)) + [9, 10], p.POSITION_CONTROL, list(jointPoses) + 2*[fingers])
        p.stepSimulation()

        state_object, _ = p.getBasePositionAndOrientation(self.objectUid)
        state_robot = p.getLinkState(self.pandaUid, 11)[0]
        state_fingers = (p.getJointState(self.pandaUid, 9)[0], p.getJointState(self.pandaUid, 10)[0])

        terminated = state_object[2] > 0.45
        reward = 1.0 if terminated else 0.0

        self.step_counter += 1
        truncated = self.step_counter > MAX_EPISODE_LEN
        if truncated and not terminated:
            reward = 0.0

        info = {"object_position": state_object}
        self.observation = state_robot + state_fingers
        obs = np.array(self.observation, dtype=np.float32)
        return obs, reward, terminated, truncated, info

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        self.step_counter = 0
        p.resetSimulation()
        p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)
        urdfRootPath = pybullet_data.getDataPath()
        p.setGravity(0, 0, -10)

        p.loadURDF(os.path.join(urdfRootPath, "plane.urdf"), basePosition=[0, 0, -0.65])
        rest_poses = [0, -0.215, 0, -2.57, 0, 2.356, 2.356, 0.08, 0.08]
        self.pandaUid = p.loadURDF(os.path.join(urdfRootPath, "franka_panda/panda.urdf"), useFixedBase=True)
        for i in range(7):
            p.resetJointState(self.pandaUid, i, rest_poses[i])
        p.resetJointState(self.pandaUid, 9, 0.08)
        p.resetJointState(self.pandaUid, 10, 0.08)
        p.loadURDF(os.path.join(urdfRootPath, "table/table.urdf"), basePosition=[0.5, 0, -0.65])
        p.loadURDF(os.path.join(urdfRootPath, "tray/traybox.urdf"), basePosition=[0.65, 0, 0])

        state_object = [random.uniform(0.5, 0.8), random.uniform(-0.2, 0.2), 0.05]
        self.objectUid = p.loadURDF(os.path.join(urdfRootPath, "random_urdfs/000/000.urdf"), basePosition=state_object)
        state_robot = p.getLinkState(self.pandaUid, 11)[0]
        state_fingers = (p.getJointState(self.pandaUid, 9)[0], p.getJointState(self.pandaUid, 10)[0])
        self.observation = state_robot + state_fingers
        p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
        obs = np.array(self.observation, dtype=np.float32)
        info = {}
        return obs, info

    def render(self):
        view_matrix = p.computeViewMatrixFromYawPitchRoll(cameraTargetPosition=[0.7, 0, 0.05],
                                                          distance=.7, yaw=90, pitch=-70, roll=0, upAxisIndex=2)
        proj_matrix = p.computeProjectionMatrixFOV(fov=60, aspect=float(960)/720, nearVal=0.1, farVal=100.0)
        (_, _, px, _, _) = p.getCameraImage(width=960, height=720, viewMatrix=view_matrix,
                                            projectionMatrix=proj_matrix, renderer=self._renderer)
        rgb_array = np.array(px, dtype=np.uint8).reshape((720, 960, 4))[:, :, :3]
        return rgb_array

    def close(self):
        p.disconnect()

register(id="panda-v0", entry_point=PandaEnv)

if __name__ == "__main__":
    # For Docker/headless, pass use_gui=False to avoid GLX
    env = gym.make("panda-v0", use_gui=True)
    obs, info = env.reset()
    for _ in range(100):
        _ = env.render()
        obs, reward, terminated, truncated, info = env.step(env.action_space.sample())
        if terminated or truncated:
            break
    env.close()