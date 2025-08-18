from environments.reacher_env import ReacherEnv
from stable_baselines3 import PPO
from stable_baselines3.common.env_checker import check_env

env = ReacherEnv(render_mode="rgb_array")
check_env(env)

model = PPO("MlpPolicy", env, verbose=1)
model.learn(total_timesteps=150_000)
model.save("models/reacher")

# if using render_mode="human"
# env.close()