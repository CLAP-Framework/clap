from setuptools import setup

setup(name='zzz-highway-v1',
      version='0.0.1',
      install_requires=['gym'],  # And any other dependencies foo needs
      packages=['gym_routing_highway', 'gym_routing_highway.envs']
)
