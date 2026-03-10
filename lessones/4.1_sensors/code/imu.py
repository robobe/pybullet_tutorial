import time
import numpy as np
import pybullet as p
import pybullet_data
import pathlib

class SimpleIMUSim:
    def __init__(self, body_id, dt, accel_noise_std=0.05, gyro_noise_std=0.01):
        self.body_id = body_id
        self.dt = dt
        self.prev_linear_velocity = None

        self.accel_noise_std = accel_noise_std
        self.gyro_noise_std = gyro_noise_std

        # Optional constant bias
        self.accel_bias = np.zeros(3)
        self.gyro_bias = np.zeros(3)

    def read(self):
        # Pose
        pos, quat = p.getBasePositionAndOrientation(self.body_id)

        # Velocities in world frame
        linear_vel, angular_vel = p.getBaseVelocity(self.body_id)
        linear_vel = np.array(linear_vel, dtype=float)
        angular_vel = np.array(angular_vel, dtype=float)

        # Rotation matrix: body -> world
        rot = np.array(p.getMatrixFromQuaternion(quat), dtype=float).reshape(3, 3)

        # Finite-difference linear acceleration in world frame
        # “Finite difference” means approximating a derivative using two nearby samples
        if self.prev_linear_velocity is None:
            linear_acc_world = np.zeros(3)
        else:
            linear_acc_world = (linear_vel - self.prev_linear_velocity) / self.dt

        self.prev_linear_velocity = linear_vel.copy()

        # Gravity in world frame
        gravity_world = np.array([0.0, 0.0, -9.81])

        # Accelerometer measures specific force:
        # a_meas = a_world - g_world
        accel_meas_world = linear_acc_world - gravity_world

        # Convert world -> body frame
        accel_meas_body = rot.T @ accel_meas_world
        gyro_meas_body = rot.T @ angular_vel

        # Add bias + Gaussian noise
        accel_meas_body += self.accel_bias + np.random.normal(0, self.accel_noise_std, 3)
        gyro_meas_body += self.gyro_bias + np.random.normal(0, self.gyro_noise_std, 3)

        return {
            "accel": accel_meas_body,   # m/s^2, body frame
            "gyro": gyro_meas_body,     # rad/s, body frame
            "quat": np.array(quat),
            "pos": np.array(pos),
        }


# -----------------------------
# Example usage
# -----------------------------
physics_client = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)

dt = 1.0 / 240.0
p.setTimeStep(dt)

plane = p.loadURDF("plane.urdf")
urdf_path = pathlib.Path(__file__).parent.as_posix()
p.setAdditionalSearchPath(urdf_path)
robot =  p.loadURDF("box.urdf", [0,0,0.1])

imu = SimpleIMUSim(robot, dt)
force_x = 8.0  # Newtons, world +X direction
torque_z = 0.1  # N*m

for i in range(2000):
    if i < 300:
        p.applyExternalForce(
            objectUniqueId=robot,
            linkIndex=-1,
            forceObj=[force_x, 0.0, 0.0],
            posObj=[0.0, 0.0, 0.0],
            flags=p.WORLD_FRAME,
        )

        p.applyExternalTorque(
            objectUniqueId=robot,
            linkIndex=-1,
            torqueObj=[0.0, 0.0, torque_z],  # spin around Z
            flags=p.WORLD_FRAME,
        )
    p.stepSimulation()

    sample = imu.read()
    accel = sample["accel"]
    gyro = sample["gyro"]

    if i % 120 == 0:
        print(f"accel [m/s^2]: {round(accel[0], 3)}, {round(accel[1], 3)}, {round(accel[2], 3)}")
        print(f"gyro  [rad/s ]: {round(gyro[0], 3)}, {round(gyro[1], 3)}, {round(gyro[2], 3)}")
        print("-" * 40)

    time.sleep(dt)

p.disconnect()
