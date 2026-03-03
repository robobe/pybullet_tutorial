import argparse
import time

import pybullet as p
import pybullet_data


def clamp(v: float, lo: float, hi: float) -> float:
    return lo if v < lo else hi if v > hi else v


def main():
    parser = argparse.ArgumentParser(
        description="Mass-spring-damper with configurable start position and external force triggers."
    )
    parser.add_argument("--start-pos", type=float, default=0.25, help="Initial joint position in meters.")
    parser.add_argument("--k", type=float, default=50.0, help="Spring stiffness (N/m).")
    parser.add_argument("--c", type=float, default=1.5, help="Damping (N*s/m).")
    parser.add_argument("--x0", type=float, default=0.0, help="Spring rest position (m).")
    parser.add_argument("--max-force", type=float, default=200.0, help="Clamp force magnitude (N).")
    parser.add_argument(
        "--push-force",
        type=float,
        default=40.0,
        help="Magnitude of external keyboard push force (N).",
    )
    parser.add_argument(
        "--pulse-duration",
        type=float,
        default=0.15,
        help="Duration of SPACE-triggered pulse (s).",
    )
    args = parser.parse_args()

    dt = 1.0 / 240.0

    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.resetSimulation()
    p.setGravity(0, 0, -9.81)
    p.setTimeStep(dt)
    p.loadURDF("plane.urdf")

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
        linkJointAxis=[[1, 0, 0]],
    )
    joint = 0

    p.setJointMotorControl2(body_id, joint, p.VELOCITY_CONTROL, force=0)
    p.resetJointState(body_id, joint, targetValue=args.start_pos, targetVelocity=0.0)
    p.changeDynamics(body_id, 0, lateralFriction=0.6, restitution=0.0)

    print("Controls:")
    print("  LEFT_ARROW  -> push in -X direction while held")
    print("  RIGHT_ARROW -> push in +X direction while held")
    print("  SPACE       -> one pulse in +X direction")
    print("  R           -> reset position to --start-pos")

    pulse_time_left = 0.0

    while p.isConnected():
        keys = p.getKeyboardEvents()

        if p.B3G_LEFT_ARROW in keys and keys[p.B3G_LEFT_ARROW] & p.KEY_IS_DOWN:
            hold_force = -args.push_force
        elif p.B3G_RIGHT_ARROW in keys and keys[p.B3G_RIGHT_ARROW] & p.KEY_IS_DOWN:
            hold_force = args.push_force
        else:
            hold_force = 0.0

        if ord(" ") in keys and keys[ord(" ")] & p.KEY_WAS_TRIGGERED:
            pulse_time_left = args.pulse_duration

        if ord("r") in keys and keys[ord("r")] & p.KEY_WAS_TRIGGERED:
            p.resetJointState(body_id, joint, targetValue=args.start_pos, targetVelocity=0.0)
            pulse_time_left = 0.0

        pulse_force = args.push_force if pulse_time_left > 0.0 else 0.0
        pulse_time_left = max(0.0, pulse_time_left - dt)

        x, xdot, _, _ = p.getJointState(body_id, joint)
        spring_force = -args.k * (x - args.x0) - args.c * xdot
        external_force = hold_force + pulse_force
        total_force = clamp(spring_force + external_force, -args.max_force, args.max_force)

        p.setJointMotorControl2(body_id, joint, p.TORQUE_CONTROL, force=total_force)
        p.stepSimulation()
        time.sleep(dt)

    p.disconnect()


if __name__ == "__main__":
    main()
