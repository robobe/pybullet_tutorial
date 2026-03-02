import time
import pybullet as p
import pybullet_data


def main():
    # --- parameters (tune these) ---
    k = 50.0       # spring stiffness (N/m)
    c = 1.5        # damping (N*s/m)
    x0 = 0.0       # rest position (m)
    max_force = 200.0  # clamp for stability/safety

    dt = 1.0 / 240.0

    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.resetSimulation()
    p.setGravity(0, 0, -9.81)
    p.setTimeStep(dt)

    p.loadURDF("plane.urdf")

    # Create a 1-DOF prismatic joint multibody:
    # base is fixed at origin, link is the "mass" that slides on x-axis
    mass = 1.0
    radius = 0.06

    col = p.createCollisionShape(p.GEOM_SPHERE, radius=radius)
    vis = p.createVisualShape(p.GEOM_SPHERE, radius=radius)

    body_id = p.createMultiBody(
        baseMass=0.0,  # fixed base
        baseCollisionShapeIndex=-1,
        baseVisualShapeIndex=-1,
        basePosition=[0, 0, 0.25],
        baseOrientation=[0, 0, 0, 1],
        linkMasses=[mass],
        linkCollisionShapeIndices=[col],
        linkVisualShapeIndices=[vis],
        linkPositions=[[0, 0, 0]],          # link frame relative to parent
        linkOrientations=[[0, 0, 0, 1]],
        linkInertialFramePositions=[[0, 0, 0]],
        linkInertialFrameOrientations=[[0, 0, 0, 1]],
        linkParentIndices=[0],
        linkJointTypes=[p.JOINT_PRISMATIC],
        linkJointAxis=[[1, 0, 0]],          # slide along X
    )

    joint = 0

    # IMPORTANT: disable PyBullet's default motor control
    p.setJointMotorControl2(body_id, joint, p.VELOCITY_CONTROL, force=0)

    # Start at some offset to see oscillation
    p.resetJointState(body_id, joint, targetValue=0.25, targetVelocity=0)

    # (Optional) reduce friction/bounce a bit
    p.changeDynamics(body_id, -1, lateralFriction=0.6, restitution=0.0)
    p.changeDynamics(body_id, 0, lateralFriction=0.6, restitution=0.0)

    while p.isConnected():
        x, xdot, _, _ = p.getJointState(body_id, joint)

        # Spring-damper force
        F = -k * (x - x0) - c * xdot

        # Clamp force for stability
        if F > max_force:
            F = max_force
        elif F < -max_force:
            F = -max_force

        # Apply force to prismatic joint
        p.setJointMotorControl2(body_id, joint, p.TORQUE_CONTROL, force=F)

        p.stepSimulation()
        time.sleep(dt)

    p.disconnect()


if __name__ == "__main__":
    main()