import gym
import numpy as np
import gym_routing


from stable_baselines.ddpg.policies import MlpPolicy
from stable_baselines.common.noise import NormalActionNoise, OrnsteinUhlenbeckActionNoise, AdaptiveParamNoiseSpec
from stable_baselines import DDPG

log_path="/home/zwt/zzz/data/veg/log/ddpg_0419"
load_path="/home/zwt/zzz/data/veg/models/ddpg_0419"
save_path="/home/zwt/zzz/data/veg/models/ddpg_0419"



env = gym.make('zzz-v1')

# the noise objects for DDPG
n_actions = env.action_space.shape[-1]
param_noise = None
action_noise = NormalActionNoise(mean=np.zeros(n_actions), sigma=float(0.5) * np.ones(n_actions))

try:
    model = DDPG.load(load_path,env=env, tensorboard_log=log_path)
    print("load saved model")

except:
    model = DDPG(MlpPolicy, env, verbose=1, param_noise=param_noise, action_noise=action_noise, gamma=0.99, nb_train_steps=20, nb_rollout_steps=100, nb_eval_steps=50, tensorboard_log=log_path,full_tensorboard_log=True, save_path=save_path)
    print("build new model")

model.learn(total_timesteps=100000)
model.save(save_path)

del model # remove to demonstrate saving and loading

model = DDPG.load(load_path)
print("load model to test")
obs, rule_action = env.reset()


while True:
    #action, _states = model.predict(obs)
    action, q_value, rule_action, rule_q = model.predict_RLS(obs, rule_action)
    print("action=",action)
    print("q_value=",q_value)
    print("rule_action=",rule_action)
    print("rule_q=",rule_q)
    q_value = q_value[0][0].astype(float)
    rule_q = rule_q[0][0].astype(float)
        

    obs, rewards, dones, info, rule_action = env.step(action, q_value, rule_action, rule_q)
    env.render()
