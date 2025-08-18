from environments.reacher_env import ReacherEnv
from stable_baselines3 import PPO

env = ReacherEnv(render_mode="human")
model = PPO.load("models/reacher")

observation, info = env.reset(seed=42)
for _ in range(1000):
    action, _states = model.predict(observation, deterministic=True)
    observation, reward, terminated, truncated, info = env.step(action)

    if terminated or truncated:
        observation, info = env.reset()

env.close()