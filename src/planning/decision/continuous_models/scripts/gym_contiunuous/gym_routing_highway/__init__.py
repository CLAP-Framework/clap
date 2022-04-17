from gym.envs.registration import register

register(
    id='zzz-highway-v1',
    entry_point='gym_routing_highway.envs:ZZZCarlaEnv'
    
)
