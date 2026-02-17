import pybullet as p
import pybullet_data
import time
import math
import cv2
import numpy as np

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.resetSimulation()
p.setGravity(0, 0, -9.8)
p.loadURDF("plane.urdf")

# simple loop track points
pts = []
for i in range(200):
    t = 2 * math.pi * i / 200
    r = 2.0 * (1 + 0.2 * math.sin(3 * t))
    pts.append((r * math.cos(t), r * math.sin(t), 0.01))

# draw a thick physical strip so it clearly appears in getCameraImage.
track_seg_visual = p.createVisualShape(
    p.GEOM_BOX, halfExtents=[0.07, 0.025, 0.003], rgbaColor=[1, 0, 0, 1]
)
for i in range(len(pts)):
    a = pts[i]
    b = pts[(i + 1) % len(pts)]
    mx = 0.5 * (a[0] + b[0])
    my = 0.5 * (a[1] + b[1])
    yaw = math.atan2(b[1] - a[1], b[0] - a[0])
    q = p.getQuaternionFromEuler([0, 0, yaw])
    p.createMultiBody(
        baseMass=0.0,
        baseVisualShapeIndex=track_seg_visual,
        basePosition=[mx, my, 0.02],
        baseOrientation=q,
    )

# place racecar on the track and align yaw with the local tangent
spawn_idx = 0
spawn_pos = pts[spawn_idx]
next_pos = pts[(spawn_idx + 1) % len(pts)]
spawn_yaw = math.atan2(next_pos[1] - spawn_pos[1], next_pos[0] - spawn_pos[0])
spawn_orn = p.getQuaternionFromEuler([0, 0, spawn_yaw])
car = p.loadURDF(
    "racecar/racecar.urdf",
    basePosition=[spawn_pos[0], spawn_pos[1], 0.2],
    baseOrientation=spawn_orn,
)

width = 640
height = 480
projection_matrix = p.computeProjectionMatrixFOV(
    fov=90.0, aspect=width / height, nearVal=0.02, farVal=30.0
)

# Camera mounted on the car body (position only).
cam_local_pos = [0.32, 0.0, 0.35]
# Camera always looks at a ground point ahead in car frame.
target_local_pos = [2.8, 0.0, 0.02]

while p.isConnected():
    p.stepSimulation()

    car_pos, car_quat = p.getBasePositionAndOrientation(car)
    cam_pos, _ = p.multiplyTransforms(
        car_pos, car_quat, cam_local_pos, [0, 0, 0, 1]
    )
    target, _ = p.multiplyTransforms(
        car_pos, car_quat, target_local_pos, [0, 0, 0, 1]
    )
    up = [0, 0, 1]

    view_matrix = p.computeViewMatrix(cam_pos, target, up)
    img = p.getCameraImage(
        width=width,
        height=height,
        viewMatrix=view_matrix,
        projectionMatrix=projection_matrix,
    )

    rgba = np.array(img[2], dtype=np.uint8).reshape((height, width, 4))
    bgr = rgba[:, :, :3][:, :, ::-1]
    cv2.imshow("Car Mounted Camera", bgr)
    if cv2.waitKey(1) & 0xFF == 27:
        break

    time.sleep(1 / 240)

cv2.destroyAllWindows()
