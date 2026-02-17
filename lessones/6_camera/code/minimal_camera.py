import pybullet as p
import pybullet_data
import time
import numpy as np
import cv2
import os

IMAGE_RGBA_INDEX = 2

# Connect
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.8)

plane = p.loadURDF("plane.urdf")
cwd = os.path.dirname(os.path.abspath(__file__))
p.setAdditionalSearchPath(cwd)
# load URDF

robot = p.loadURDF("rrbot.urdf", basePosition=[0, 0, 0.5], useFixedBase=True)

# Camera parameters
width = 640
height = 480

# Camera position in WORLD frame
camera_pos = [5, 0, 3]
target_pos = [0, 0, 0]
up_vector = [0, 0, 1] # saying “world +Z is up”. So the camera horizon is aligned so Z points upward in the image.

# Builds camera extrinsic matrix (where camera is, where it looks, what is “up”).
# define the camera pose in the world frame
# - cameraEyePosition: camera location
# - cameraTargetPosition: look-at point
# - cameraUpVector: camera up direction

view_matrix = p.computeViewMatrix(
    cameraEyePosition=camera_pos,
    cameraTargetPosition=target_pos,
    cameraUpVector=up_vector
)

# Builds projection/intrinsic matrix (how 3D gets projected to image plane).
# - fov: vertical field of view in degrees
# - aspect: width/height
# - nearVal, farVal: clipping planes
projection_matrix = p.computeProjectionMatrixFOV(
    fov=60,
    aspect=width / height,
    nearVal=0.1,
    farVal=100
)

while True:
    p.stepSimulation()
    # Renders the scene using those two matrices.
    # - Returns tuple including:
    # - width, height
    # - RGBA image buffer (img[2])
    # - depth buffer (img[3])
    # - segmentation mask (img[4])
    img = p.getCameraImage(
        width,
        height,
        view_matrix,
        projection_matrix
    )

    # img[IMAGE_RGBA_INDEX] is the raw color buffer from getCameraImage, 
    # usually a flat 1D array of length height * width * 4 (RGBA per pixel).
    rgba = np.array(img[IMAGE_RGBA_INDEX], dtype=np.uint8).reshape((height, width, 4))
    # drops alpha, keeps only RGB.
    rgb  = rgba[:, :, :3]  # uint8 now

    # (optional) OpenCV expects BGR
    bgr = rgb[:, :, ::-1]

    cv2.imshow("Camera View", bgr)
    cv2.waitKey(1)

    time.sleep(1/240)
