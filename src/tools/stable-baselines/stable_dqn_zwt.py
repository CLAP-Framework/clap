import gym
import gym_routing_contiuous_2

import numpy as np
import sys
# sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')

from stable_baselines.deepq_veg.policies import MlpPolicy
from stable_baselines.deepq_veg.dqn import DQN

log_path="/home/icv/zwt_rls/log/dqn_1223"
load_path="/home/icv/zwt_rls/models/dqn_1223"
save_path="/home/icv/zwt_rls/models/dqn_1223"

zzz_env = gym.make('zzz-v2')


try:
    model = DQN.load("zwt_deepq_1222",env=zzz_env)
    print("load saved model")
except:
    model = DQN(MlpPolicy, env=zzz_env, save_path=save_path) #, verbose=1
    print("build new model")

model.learn(total_timesteps=1000000)
model.save("zwt_deepq_1222")

del model # remove to demonstrate saving and loading


print("load model to test")
model = DQN.load("zwt_deepq_1222")

obs = zzz_env.reset()


while True:
    action, _states = model.predict(obs)
    obs, rewards, dones, info = zzz_env.step(action)
    #zzz_env.render()
