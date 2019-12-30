# This script is used for benchmarking with two vehicle following case

import carla
from addict import Dict as edict
import numpy as np
import time
from scipy.io import loadmat

# some global settings
traj_path = "/home/jacobz/ZZZ/ref/v13x23c.mat"
profile_number = 0
speed_direction = np.array([1, 0, 0]) # speed direction vector
init_speed = 13
start_delay = 10

# global variables
global_states = edict(
    vehicle_id = None,
    speed_profile = None,
    init_time = None,
    client = None,
    started = False,
    finished = False
)

def callback(snapshot):
    t = snapshot.timestamp.elapsed_seconds
    if global_states.init_time is None:
        global_states.init_time = t + start_delay
        return
    t -= global_states.init_time

    if t > 0 and not global_states.started:
        print("Start to apply velocity profile")
        global_states.started = True

    if len(global_states.speed_profile) <= 0:
        global_states.finished = True
        return

    # find latest speed value
    latest_time, latest_speed = global_states.speed_profile[-1]
    while t >= latest_time + 0.01: # 0.01 is for tolerance
        global_states.speed_profile.pop()
        if len(global_states.speed_profile) <= 0:
            global_states.finished = True
            return

        latest_time, latest_speed = global_states.speed_profile[-1]

    vel_vec = speed_direction * latest_speed / np.linalg.norm(speed_direction)
    vel = carla.Vector3D(x=vel_vec[0], y=vel_vec[1], z=vel_vec[2])
    command = carla.command.ApplyVelocity(actor_id=global_states.vehicle_id, velocity=vel)
    global_states.client.apply_batch([command])

def main():
    # load speed profile
    data = loadmat(traj_path)
    data = [np.hstack(tns) for tns in zip(data['times'][0], data['speed'][0])]
    global_states.speed_profile = list(reversed(data[profile_number]))

    # set up client
    client = carla.Client("localhost", 2000)
    client.set_timeout(4.0)
    world = client.get_world()
    global_states.client = client
    cbid = world.on_tick(callback)

    # find spawned vehicle
    ego_id = -1
    for actor in world.get_actors():
        role_name = actor.attributes.get("role_name")
        if role_name == "frontv":
            global_states.vehicle_id = actor.id
        elif role_name == "egov":
            ego_id = actor.id

    # apply init speed
    print("Apply initial velocity on front vehicle")
    vel_vec = speed_direction * init_speed / np.linalg.norm(speed_direction)
    vel = carla.Vector3D(x=vel_vec[0], y=vel_vec[1], z=vel_vec[2])
    front_command = carla.command.ApplyVelocity(actor_id=global_states.vehicle_id, velocity=vel)
    client.apply_batch([front_command])

    time.sleep(2) # this value enlarge the distance between ego and front vehicle
    print("Apply initial velocity on ego vehicle")
    ego_command = carla.command.ApplyVelocity(actor_id=ego_id, velocity=vel)
    client.apply_batch([ego_command])

    # loop
    try:
        while not global_states.finished:
            pass
        print("Profile executed")
    finally:
        world.remove_on_tick(cbid)

if __name__ == "__main__":
    main()
