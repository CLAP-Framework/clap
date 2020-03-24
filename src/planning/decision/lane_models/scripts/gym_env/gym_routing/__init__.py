from gym.envs.registration import register

register(
    id='zzz_lane-v0',
    entry_point='gym_routing.envs:ZZZCarlaEnv_lane'
)
