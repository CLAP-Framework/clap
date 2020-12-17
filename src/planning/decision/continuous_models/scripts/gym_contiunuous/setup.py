from setuptools import setup

setup(name='carla_trainning_continuous',
      version='0.0.2',
      install_requires=['gym'],  # And any other dependencies foo needs
      packages=['gym_routing_contiuous', 'gym_routing_contiuous.envs']
)
