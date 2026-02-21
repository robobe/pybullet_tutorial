import gymnasium as gym


env = gym.make("CartPole-v1", render_mode="human")

Kp, Ki, Kd = 0.1, 0.01, 0.5
NO_OF_EPISODES = 3
NO_OF_STEPS = 100
DT = env.unwrapped.tau  # type: ignore #0.02  # CartPole timestep (seconds)

PUSH_RIGHT = 1
PUSH_LEFT = 0

INDEX_THETA = 2

for i_episode in range(NO_OF_EPISODES):
    print(f"Episode {i_episode + 1}/{NO_OF_EPISODES}")
    state, info = env.reset()

    integral = 0
    prev_error = 0
    for t in range(NO_OF_STEPS):
        theta = state[INDEX_THETA]

        # region pid control
        error = -theta
        integral += error * DT
        derivative = (error - prev_error) / DT
        prev_error = error

        pid = Kp * error + Ki * integral + Kd * derivative
        action = PUSH_LEFT if pid > 0 else PUSH_RIGHT
        # endregion
        state, reward, terminated, truncated, info = env.step(action)

        if terminated or truncated:
            print("Episode finished after {} timesteps".format(t + 1))
            break

env.close()
