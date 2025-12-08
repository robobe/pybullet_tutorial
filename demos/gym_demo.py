import gymnasium as gym
import pybullet_envs.bullet

print([env.id for env in gym.envs.registry.keys() if "Bullet" in env.id])
