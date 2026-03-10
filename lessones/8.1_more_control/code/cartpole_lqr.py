import argparse
import time
from typing import NamedTuple

import numpy as np
import pybullet as p2
import pybullet_data
from scipy.linalg import solve_discrete_are
from pybullet_utils import bullet_client as bc

JOINT_CART = 0
JOINT_POLE = 1


class CartPoleState(NamedTuple):
    theta: float
    theta_dot: float
    x: float
    x_dot: float


class CartPoleModel(NamedTuple):
    m_cart: float
    m_pole: float
    length: float
    gravity: float


class LQRController:
    def __init__(self, model: CartPoleModel, dt=0.02, force_limit=30.0, q=None, r=0.1):
        self.model = model
        self.dt = dt
        self.force_limit = abs(force_limit)
        self.k = self._build_gain(q=q, r=r)

    def _build_gain(self, q=None, r=0.1):
        # Linearized cart-pole around upright equilibrium.
        # State order: [x, x_dot, theta, theta_dot]
        g = self.model.gravity
        m_cart = self.model.m_cart
        m_pole = self.model.m_pole
        total_mass = m_cart + m_pole
        length = self.model.length
        polemass_length = m_pole * length

        A = np.array(
            [
                [0.0, 1.0, 0.0, 0.0],
                [0.0, 0.0, (m_pole * g) / m_cart, 0.0],
                [0.0, 0.0, 0.0, 1.0],
                [0.0, 0.0, (total_mass * g) / polemass_length, 0.0],
            ],
            dtype=float,
        )
        B = np.array([[0.0], [1.0 / m_cart], [0.0], [1.0 / polemass_length]], dtype=float)

        Ad = np.eye(4) + A * self.dt
        Bd = B * self.dt

        if q is None:
            q = np.diag([2.0, 1.0, 40.0, 6.0])
        else:
            q = np.diag(q)
        R = np.array([[r]], dtype=float)

        P = solve_discrete_are(Ad, Bd, q, R)
        K = np.linalg.inv(Bd.T @ P @ Bd + R) @ (Bd.T @ P @ Ad)
        return K

    def compute(self, state: CartPoleState, gain_scale=1.0, control_sign=-1.0):
        x_vec = np.array([[state.x], [state.x_dot], [state.theta], [state.theta_dot]])
        force = float(control_sign * gain_scale * (self.k @ x_vec)[0, 0])
        return float(np.clip(force, -self.force_limit, self.force_limit))


class CartPoleSimulation:
    def __init__(self, gui=True, force_mag=30.0, init_pole_angle=0.05, init_cart_pos=0.0):
        self.gui = gui
        self.force_mag = force_mag
        self.time_step = 0.02
        self.gravity = 9.8
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
        p.changeDynamics(self.cartpole, -1, linearDamping=0, angularDamping=0)
        p.changeDynamics(self.cartpole, JOINT_CART, linearDamping=0, angularDamping=0)
        p.changeDynamics(self.cartpole, JOINT_POLE, linearDamping=0, angularDamping=0)
        p.setJointMotorControl2(self.cartpole, JOINT_POLE, p.VELOCITY_CONTROL, force=0)
        p.setJointMotorControl2(self.cartpole, JOINT_CART, p.VELOCITY_CONTROL, force=0)
        p.setGravity(0, 0, -self.gravity)
        p.setTimeStep(self.time_step)
        p.setRealTimeSimulation(0)

    def get_model(self):
        p = self.client
        base_mass = float(p.getDynamicsInfo(self.cartpole, -1)[0])
        cart_mass = float(p.getDynamicsInfo(self.cartpole, JOINT_CART)[0])
        m_cart = base_mass + cart_mass
        m_pole = float(p.getDynamicsInfo(self.cartpole, JOINT_POLE)[0])

        # URDF pole COM offset from the pole joint frame gives half-pole length.
        pole_com_offset = p.getDynamicsInfo(self.cartpole, JOINT_POLE)[3]
        length = float(np.linalg.norm(pole_com_offset))
        if length <= 1e-8:
            length = 0.5

        return CartPoleModel(
            m_cart=m_cart,
            m_pole=m_pole,
            length=length,
            gravity=self.gravity,
        )

    def reset(self):
        self.client.resetJointState(self.cartpole, JOINT_POLE, self.init_pole_angle, 0.0)
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
        p.setJointMotorControl2(self.cartpole, JOINT_CART, p.TORQUE_CONTROL, force=force)
        p.stepSimulation()

        state = self.get_state()
        done = (
            state.x < -self.x_threshold
            or state.x > self.x_threshold
            or state.theta < -self.theta_threshold_radians
            or state.theta > self.theta_threshold_radians
        )
        return state, bool(done)

    def close(self):
        self.client.disconnect()


def run_lqr_episode(
    sim, lqr, max_steps=500, realtime=True, gain_scale=1.0, control_sign=-1.0
):
    state = sim.reset()
    for t in range(max_steps):
        force = lqr.compute(state, gain_scale=100, control_sign=1)
        state, done = sim.step(force)
        if realtime and sim.gui:
            time.sleep(sim.time_step)
        if done:
            return t + 1
    return max_steps


def main():
    parser = argparse.ArgumentParser(description="PyBullet cart-pole with LQR control")
    parser.add_argument("--episodes", type=int, default=0, help="0 or negative for infinite run.")
    parser.add_argument("--max-steps", type=int, default=500)
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--force-mag", type=float, default=40.0)
    parser.add_argument("--r", type=float, default=0.05, help="LQR control penalty (R).")
    parser.add_argument(
        "--gain-scale",
        type=float,
        default=1.0,
        help="Multiplier on LQR output before clipping.",
    )
    parser.add_argument(
        "--control-sign",
        type=float,
        default=-1.0,
        help="Use -1 or +1 to match your plant/control direction.",
    )
    parser.add_argument("--qx", type=float, default=2.0)
    parser.add_argument("--qxdot", type=float, default=1.0)
    parser.add_argument("--qtheta", type=float, default=80.0)
    parser.add_argument("--qthetadot", type=float, default=10.0)
    parser.add_argument("--init-pole-angle", type=float, default=0.05)
    parser.add_argument("--init-cart-pos", type=float, default=0.0)
    args = parser.parse_args()

    sim = CartPoleSimulation(
        gui=not args.headless,
        force_mag=args.force_mag,
        init_pole_angle=args.init_pole_angle,
        init_cart_pos=args.init_cart_pos,
    )
    lqr = LQRController(
        model=sim.get_model(),
        dt=sim.time_step,
        force_limit=args.force_mag,
        q=[args.qx, args.qxdot, args.qtheta, args.qthetadot],
        r=args.r,
    )
    print(f"LQR gain K: {lqr.k}")

    try:
        episode = 0
        while args.episodes <= 0 or episode < args.episodes:
            steps = run_lqr_episode(
                sim,
                lqr,
                max_steps=args.max_steps,
                realtime=not args.headless,
                gain_scale=args.gain_scale,
                control_sign=args.control_sign,
            )
            print(f"Episode {episode + 1}: steps={steps}")
            episode += 1
    finally:
        sim.close()


if __name__ == "__main__":
    main()
