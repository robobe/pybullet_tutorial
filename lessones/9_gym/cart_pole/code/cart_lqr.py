import numpy as np
import gymnasium as gym


def solve_dare_iterative(
    A: np.ndarray,
    B: np.ndarray,
    Q: np.ndarray,
    R: np.ndarray,
    max_iters: int = 10_000,
    tol: float = 1e-10,
) -> np.ndarray:
    """Solve discrete-time ARE with fixed-point iteration."""
    P = Q.copy()
    for _ in range(max_iters):
        BT_P = B.T @ P
        S = R + BT_P @ B
        K = np.linalg.solve(S, BT_P @ A)
        P_next = A.T @ P @ A - A.T @ P @ B @ K + Q
        if np.max(np.abs(P_next - P)) < tol:
            return P_next
        P = P_next
    return P


def build_lqr_gain(dt: float) -> np.ndarray:
    # CartPole constants (same values used by Gym CartPole env)
    g = 9.8
    masscart = 1.0
    masspole = 0.1
    length = 0.5  # half pole length

    mc = masscart
    mp = masspole
    l = length

    # Linearized around upright theta = 0 (continuous-time)
    A_c = np.array(
        [
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, -(mp * g) / mc, 0.0],
            [0.0, 0.0, 0.0, 1.0],
            [0.0, 0.0, ((mc + mp) * g) / (l * mc), 0.0],
        ]
    )
    B_c = np.array([[0.0], [1.0 / mc], [0.0], [-1.0 / (l * mc)]])

    # Discretize with forward Euler
    A = np.eye(4) + dt * A_c
    B = dt * B_c

    # Penalize pole angle most strongly
    Q = np.diag([1.0, 1.0, 25.0, 3.0])
    R = np.array([[0.08]])

    P = solve_dare_iterative(A, B, Q, R)
    K = np.linalg.solve(R + B.T @ P @ B, B.T @ P @ A)
    return K


def main() -> None:
    env = gym.make("CartPole-v1", render_mode="human")
    dt = env.unwrapped.tau
    K = build_lqr_gain(dt)

    PUSH_LEFT = 0
    PUSH_RIGHT = 1
    NO_OF_EPISODES = 3
    NO_OF_STEPS = 500
    EPS = 0.02  # deadband for less action chattering

    for i_episode in range(NO_OF_EPISODES):
        state, info = env.reset()
        action = PUSH_RIGHT

        for t in range(NO_OF_STEPS):
            x = np.asarray(state, dtype=float).reshape(4, 1)
            u = float(-(K @ x)[0, 0])

            if u > EPS:
                action = PUSH_RIGHT
            elif u < -EPS:
                action = PUSH_LEFT

            state, reward, terminated, truncated, info = env.step(action)
            if terminated or truncated:
                print(f"Episode {i_episode + 1}: finished after {t + 1} timesteps")
                break
        else:
            print(f"Episode {i_episode + 1}: reached max steps ({NO_OF_STEPS})")

    env.close()


if __name__ == "__main__":
    main()
