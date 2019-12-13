from gym.envs.registration import register

register(
    id='zzz-v1',
    entry_point='env.zzz_ddpg:ZZZCarlaEnv'
)
