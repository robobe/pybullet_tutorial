from cart_pole_con_env import ContinuousCartPoleEnv
from stable_baselines3 import SAC
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.evaluation import evaluate_policy
import pathlib

# Create env
env = ContinuousCartPoleEnv(render_mode="none", force_mag=10.0, max_steps=500)
env = Monitor(env)

# SAC (works because action_space is Box)
model = SAC(
    policy="MlpPolicy",
    env=env,
    learning_rate=3e-4,
    buffer_size=100_000,
    batch_size=256,
    tau=0.005,
    gamma=0.99,
    train_freq=1,
    gradient_steps=1,
    ent_coef="auto",
    verbose=1,
)
file = pathlib.Path(__file__).parent.joinpath("sac_cartpole").as_posix()

if pathlib.Path(file).exists():
    print("Loading existing model...")
    model.load(file)
else:
    print("Training new model...")
model.learn(total_timesteps=200_000)


model.save(file)
model.load(file)
mean_reward, std_reward = evaluate_policy(model, env, n_eval_episodes=20, deterministic=True)
print(f"\nMean reward: {mean_reward:.1f} +/- {std_reward:.1f}")

# Watch it (console render)
watch_env = ContinuousCartPoleEnv(render_mode="human")
obs, _ = watch_env.reset()
for _ in range(600):
    action, _ = model.predict(obs, deterministic=True)
    obs, reward, terminated, truncated, _ = watch_env.step(action)
    if terminated or truncated:
        print("\nEpisode done\n")
        obs, _ = watch_env.reset()
watch_env.close()