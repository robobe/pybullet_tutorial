import argparse
import time
from typing import NamedTuple

import numpy as np
import pybullet as p2
import pybullet_data
from pybullet_utils import bullet_client as bc

JOINT_CART = 0
JOINT_POLE = 1


class CartPoleState(NamedTuple):
    theta: float
    theta_dot: float
    x: float
    x_dot: float


class PIDController:
    def __init__(
        self,
        kp=120.0,
        ki=0.5,
        kd=22.0,
        dt=0.02,
        output_limit=30.0,
        integral_limit=2.0,
    ):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt
        self.output_limit = abs(output_limit)
        self.integral_limit = abs(integral_limit)
        self.integral = 0.0
        self.prev_error = 0.0

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0

    def compute(self, error):
        self.integral += error * self.dt
        self.integral = float(
            np.clip(self.integral, -self.integral_limit, self.integral_limit)
        )
        derivative = (error - self.prev_error) / self.dt
        self.prev_error = error
        output = (
            self.kp * error
            + self.ki * self.integral
            + self.kd * derivative
        )
        return float(np.clip(output, -self.output_limit, self.output_limit))


class CartPoleSimulation:
    def __init__(self, gui=True, force_mag=30.0, init_pole_angle=0.05, init_cart_pos=0.0):
        self.gui = gui
        self.force_mag = force_mag
        self.time_step = 0.02
        self.init_pole_angle = init_pole_angle
        self.init_cart_pos = init_cart_pos
        self.theta_threshold_radians = 12 * 2 * np.pi / 360
        self.x_threshold = 2.4

        connection_mode = p2.GUI if gui else p2.DIRECT
        self.client = bc.BulletClient(connection_mode=connection_mode)
        self.cartpole = -1
        self._load_world()

    def _load_world(self):
        p = self.client
        p.resetSimulation()
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        self.cartpole = p.loadURDF("cartpole.urdf", [0, 0, 0])
        # Remove damping from the system so it's more responsive to our forces and easier to tune.
        p.changeDynamics(self.cartpole, -1, linearDamping=0, angularDamping=0)
        p.changeDynamics(self.cartpole, JOINT_CART, linearDamping=0, angularDamping=0)
        p.changeDynamics(self.cartpole, JOINT_POLE, linearDamping=0, angularDamping=0)
        # Disable default motor control, we will apply forces directly. 
        p.setJointMotorControl2(self.cartpole, JOINT_POLE, p.VELOCITY_CONTROL, force=0)
        p.setJointMotorControl2(self.cartpole, JOINT_CART, p.VELOCITY_CONTROL, force=0)

        p.setGravity(0, 0, -9.8)
        # Use a fixed time step and disable real-time simulation to ensure consistent physics regardless of rendering performance.
        p.setTimeStep(self.time_step)
        # disable real-time simulation, we will step manually in sync with our control loop
        p.setRealTimeSimulation(0)

    def reset(self):
        # Start away from perfect equilibrium so PID has non-zero error.
        self.client.resetJointState(
            self.cartpole, JOINT_POLE, self.init_pole_angle, 0.0
        )
        self.client.resetJointState(self.cartpole, JOINT_CART, self.init_cart_pos, 0.0)
        return self.get_state()

    def get_state(self):
        p = self.client
        theta, theta_dot = p.getJointState(self.cartpole, JOINT_POLE)[0:2]
        x, x_dot = p.getJointState(self.cartpole, JOINT_CART)[0:2]
        return CartPoleState(theta=float(theta), theta_dot=float(theta_dot), x=float(x), x_dot=float(x_dot))

    def step(self, force):
        force = float(np.clip(force, -self.force_mag, self.force_mag))
        p = self.client
        p.setJointMotorControl2(
            self.cartpole, JOINT_CART, p.TORQUE_CONTROL, force=force
        )
        p.stepSimulation()

        state = self.get_state()
        theta = state.theta
        x = state.x
        done = (
            x < -self.x_threshold
            or x > self.x_threshold
            or theta < -self.theta_threshold_radians
            or theta > self.theta_threshold_radians
        )
        return state, bool(done)

    def close(self):
        self.client.disconnect()


def run_pid_episode(sim, pid, max_steps=500, realtime=True, kx=1.0, kv=2.0):
    state = sim.reset()
    pid.reset()
    for t in range(max_steps):
        theta = state.theta
        # PID on pole angle + PD centering on cart position/velocity.
        force = pid.compute(error=theta)
        state, done = sim.step(force)
        if realtime and sim.gui:
            time.sleep(sim.time_step)
        if done:
            return t + 1
    return max_steps


def main():
    parser = argparse.ArgumentParser(description="PyBullet cart-pole with PID control")
    parser.add_argument(
        "--episodes",
        type=int,
        default=0,
        help="Number of episodes. Use 0 or negative for infinite run.",
    )
    parser.add_argument("--max-steps", type=int, default=500)
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--kp", type=float, default=100.0)
    parser.add_argument("--ki", type=float, default=0.0)
    parser.add_argument("--kd", type=float, default=0.0)
    parser.add_argument(
        "--kx",
        type=float,
        default=1.0,
        help="Cart position centering gain.",
    )
    parser.add_argument(
        "--kv",
        type=float,
        default=2.0,
        help="Cart velocity damping gain.",
    )
    parser.add_argument("--force-mag", type=float, default=30.0)
    parser.add_argument(
        "--integral-limit",
        type=float,
        default=2.0,
        help="Clamp for PID integral term (anti-windup).",
    )
    parser.add_argument(
        "--init-pole-angle",
        type=float,
        default=0.05,
        help="Initial pole angle in radians. 0 means perfect upright equilibrium.",
    )
    parser.add_argument(
        "--init-cart-pos",
        type=float,
        default=0.0,
        help="Initial cart position in meters.",
    )
    args = parser.parse_args()

    sim = CartPoleSimulation(
        gui=not args.headless,
        force_mag=args.force_mag,
        init_pole_angle=args.init_pole_angle,
        init_cart_pos=args.init_cart_pos,
    )
    pid = PIDController(
        kp=args.kp,
        ki=args.ki,
        kd=args.kd,
        dt=sim.time_step,
        output_limit=args.force_mag,
        integral_limit=args.integral_limit,
    )

    try:
        episode = 0
        while args.episodes <= 0 or episode < args.episodes:
            steps = run_pid_episode(
                sim,
                pid,
                max_steps=args.max_steps,
                realtime=not args.headless,
                kx=args.kx,
                kv=args.kv,
            )
            print(f"Episode {episode + 1}: steps={steps}")
            episode += 1
    finally:
        sim.close()


if __name__ == "__main__":
    main()
