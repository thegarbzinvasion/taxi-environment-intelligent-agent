import gymnasium as gym
env = gym.make("Taxi-v3", render_mode="human")
obs, info = env.reset()
frame = env.render()

