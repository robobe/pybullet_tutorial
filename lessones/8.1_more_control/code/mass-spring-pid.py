import time
import pybullet as p
import pybullet_data


def clamp(v: float, lo: float, hi: float) -> float:
    return lo if v < lo else hi if v > hi else v


def main():
    # ---------- plant (spring-damper) ----------
    k = 50.0      # N/m
    c = 1.5       # N*s/m
    x0 = 0.0      # spring rest position

    # ---------- PID controller (external force u) ----------
    x_ref = 0.30  # target position (m)

    Kp = 120.0
    Ki = 10.0
    Kd = 25.0

    u_limit = 200.0     # clamp for PID output (N)
    int_limit = 1.0     # anti-windup clamp for integral term

    # ---------- disturbance (optional) ----------
    use_disturbance = False
    F_dist = 0.0  # will be updated in loop

    # ---------- sim ----------
    dt = 1.0 / 240.0
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.resetSimulation()
    p.setGravity(0, 0, -9.81)
    p.setTimeStep(dt)
    p.loadURDF("plane.urdf")

    # 1-DOF prismatic joint mass
    mass = 1.0
    radius = 0.06
    col = p.createCollisionShape(p.GEOM_SPHERE, radius=radius)
    vis = p.createVisualShape(p.GEOM_SPHERE, radius=radius)

    body_id = p.createMultiBody(
        baseMass=0.0,
        baseCollisionShapeIndex=-1,
        baseVisualShapeIndex=-1,
        basePosition=[0, 0, 0.25],
        baseOrientation=[0, 0, 0, 1],
        linkMasses=[mass],
        linkCollisionShapeIndices=[col],
        linkVisualShapeIndices=[vis],
        linkPositions=[[0, 0, 0]],
        linkOrientations=[[0, 0, 0, 1]],
        linkInertialFramePositions=[[0, 0, 0]],
        linkInertialFrameOrientations=[[0, 0, 0, 1]],
        linkParentIndices=[0],
        linkJointTypes=[p.JOINT_PRISMATIC],
        linkJointAxis=[[1, 0, 0]],  # slide along X
    )
    joint = 0

    # Disable default motor
    p.setJointMotorControl2(body_id, joint, p.VELOCITY_CONTROL, force=0)

    # Start offset
    p.resetJointState(body_id, joint, targetValue=0.0, targetVelocity=0.0)

    # PID state
    integral = 0.0
    prev_error = 0.0

    t = 0.0
    while p.isConnected():
        x, xdot, _, _ = p.getJointState(body_id, joint)

        # --- spring + damper force ---
        F_sd = -k * (x - x0) - c * xdot

        # --- PID external force u ---
        error = x_ref - x
        integral += error * dt
        integral = clamp(integral, -int_limit, int_limit)  # anti-windup

        derivative = (error - prev_error) / dt
        prev_error = error

        u = Kp * error + Ki * integral + Kd * derivative
        u = clamp(u, -u_limit, u_limit)

        # --- optional disturbance force (e.g., push every 2 seconds) ---
        if use_disturbance:
            # a short pulse push in +x direction for 0.15s every 2s
            phase = (t % 2.0)
            F_dist = 40.0 if phase < 0.15 else 0.0
        else:
            F_dist = 0.0

        # Total force on the prismatic axis
        F_total = F_sd + u + F_dist

        # Apply
        p.setJointMotorControl2(body_id, joint, p.TORQUE_CONTROL, force=F_total)

        p.stepSimulation()
        time.sleep(dt)
        t += dt

    p.disconnect()


if __name__ == "__main__":
    main()