# Copyright (c) Facebook, Inc. and its affiliates.
# All rights reserved.

# This source code is licensed under the license found in the
# LICENSE file in the root directory of this source tree.

import numpy as np
import torch
import argparse
import os
import gym
import gym_routing_highway

import time
import json
import random

import sys
import cv2
sys.path.insert(0,'/opt/ros/melodic/lib/python2.7/dist-packages')

import utils
from logger import Logger
# from video import VideoRecorder

from agent.bisim_agent import BisimAgent
# from agents.navigation.carla_env import CarlaEnv

# import gym_carla
from torchvision import transforms
from Mcts import MCTS




def parse_args():
    parser = argparse.ArgumentParser()
    # environment
    parser.add_argument('--domain_name', default='carla')
    parser.add_argument('--task_name', default='run')
    parser.add_argument('--image_size', default=84, type=int)
    parser.add_argument('--action_repeat', default=1, type=int)
    parser.add_argument('--frame_stack', default=1, type=int) #3
    parser.add_argument('--resource_files', type=str)
    parser.add_argument('--eval_resource_files', type=str)
    parser.add_argument('--img_source', default=None, type=str, choices=['color', 'noise', 'images', 'video', 'none'])
    parser.add_argument('--total_frames', default=1000, type=int)
    # replay buffer
    parser.add_argument('--replay_buffer_capacity', default=10000, type=int)
    # train
    parser.add_argument('--agent', default='bisim', type=str, choices=['baseline', 'bisim', 'deepmdp'])
    parser.add_argument('--init_steps', default=10, type=int)
    parser.add_argument('--num_train_steps', default=1000000, type=int)
    parser.add_argument('--batch_size', default=32, type=int)
    parser.add_argument('--hidden_dim', default=256, type=int)
    parser.add_argument('--k', default=3, type=int, help='number of steps for inverse model')
    parser.add_argument('--bisim_coef', default=0.5, type=float, help='coefficient for bisim terms')
    parser.add_argument('--load_encoder', default=None, type=str)
    # eval
    parser.add_argument('--eval_freq', default=5000, type=int)  # TODO: master had 10000
    parser.add_argument('--num_eval_episodes', default=20, type=int)
    # critic
    parser.add_argument('--critic_lr', default=1e-3, type=float)
    parser.add_argument('--critic_beta', default=0.9, type=float)
    parser.add_argument('--critic_tau', default=0.005, type=float)
    parser.add_argument('--critic_target_update_freq', default=2, type=int)
    # actor
    parser.add_argument('--actor_lr', default=1e-3, type=float)
    parser.add_argument('--actor_beta', default=0.9, type=float)
    parser.add_argument('--actor_log_std_min', default=-10, type=float)
    parser.add_argument('--actor_log_std_max', default=2, type=float)
    parser.add_argument('--actor_update_freq', default=2, type=int)
    # encoder/decoder
    parser.add_argument('--encoder_type', default='pixelCarla098', type=str, choices=['pixel', 'pixelCarla096', 'pixelCarla098', 'identity'])
    parser.add_argument('--encoder_feature_dim', default=50, type=int)
    parser.add_argument('--encoder_lr', default=1e-3, type=float)
    parser.add_argument('--encoder_tau', default=0.005, type=float)
    parser.add_argument('--encoder_stride', default=1, type=int)
    parser.add_argument('--decoder_type', default='pixel', type=str, choices=['pixel', 'identity', 'contrastive', 'reward', 'inverse', 'reconstruction'])
    parser.add_argument('--decoder_lr', default=1e-3, type=float)
    parser.add_argument('--decoder_update_freq', default=1, type=int)
    parser.add_argument('--decoder_weight_lambda', default=0.0, type=float)
    parser.add_argument('--num_layers', default=4, type=int)
    parser.add_argument('--num_filters', default=32, type=int)
    # sac
    parser.add_argument('--discount', default=0.99, type=float)
    parser.add_argument('--init_temperature', default=0.01, type=float)
    parser.add_argument('--alpha_lr', default=1e-3, type=float)
    parser.add_argument('--alpha_beta', default=0.9, type=float)
    # misc
    parser.add_argument('--seed', default=1, type=int)
    parser.add_argument('--work_dir', default='.', type=str)
    parser.add_argument('--save_tb', default=False, action='store_true')
    parser.add_argument('--save_model', default=True, action='store_true')
    parser.add_argument('--save_buffer', default=False, action='store_true')
    parser.add_argument('--save_video', default=False, action='store_true')
    parser.add_argument('--transition_model_type', default='probabilistic', type=str, choices=['', 'deterministic', 'probabilistic', 'ensemble'])
    parser.add_argument('--render', default=False, action='store_true')
    parser.add_argument('--port', default=2000, type=int)
    args = parser.parse_args()
    return args


def make_agent(obs_shape, action_shape, state_space_dim, args, device):
    agent = BisimAgent(
        obs_shape=obs_shape,
        action_shape=action_shape,
        device=device,
        state_space_dim=state_space_dim,
        hidden_dim=args.hidden_dim,
        discount=args.discount,
        init_temperature=args.init_temperature,
        # alpha_lr=args.alpha_lr,
        # alpha_beta=args.alpha_beta,
        # actor_lr=args.actor_lr,
        # actor_beta=args.actor_beta,
        # actor_log_std_min=args.actor_log_std_min,
        # actor_log_std_max=args.actor_log_std_max,
        # actor_update_freq=args.actor_update_freq,
        # critic_lr=args.critic_lr,
        # critic_beta=args.critic_beta,
        # critic_tau=args.critic_tau,
        # critic_target_update_freq=args.critic_target_update_freq,
        # encoder_type=args.encoder_type,
        # encoder_feature_dim=args.encoder_feature_dim,
        # encoder_lr=args.encoder_lr,
        # encoder_tau=args.encoder_tau,
        # encoder_stride=args.encoder_stride,
        # decoder_type=args.decoder_type,
        decoder_lr=args.decoder_lr,
        # decoder_update_freq=args.decoder_update_freq,
        decoder_weight_lambda=args.decoder_weight_lambda,
        transition_model_type=args.transition_model_type,
        # num_layers=args.num_layers,
        # num_filters=args.num_filters,
        bisim_coef=args.bisim_coef
    )

    # if args.load_encoder:
    #     model_dict = agent.actor.encoder.state_dict()
    #     encoder_dict = torch.load(args.load_encoder) 
    #     encoder_dict = {k[8:]: v for k, v in encoder_dict.items() if 'encoder.' in k}  # hack to remove encoder. string
    #     agent.actor.encoder.load_state_dict(encoder_dict)
    #     agent.critic.encoder.load_state_dict(encoder_dict)

    return agent


def main():
    args = parse_args()
    utils.set_seed_everywhere(args.seed)

    env = gym.make('zzz-highway-v1')
    Library = MCTS(obs_dimension=env.state_dimension, action_dimension=env.action_dimension)

    utils.make_dir(args.work_dir)
    model_dir = utils.make_dir(os.path.join(args.work_dir, 'model'))
    buffer_dir = utils.make_dir(os.path.join(args.work_dir, 'buffer'))

    # video = VideoRecorder(video_dir if args.save_video else None)

    with open(os.path.join(args.work_dir, 'args.json'), 'w') as f:
        json.dump(vars(args), f, sort_keys=True, indent=4)

    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')

    replay_buffer = utils.ReplayBuffer(
        obs_shape=env.observation_space.shape,
        action_shape=[1], # discrete, 1 dimension!
        capacity=args.replay_buffer_capacity,
        batch_size=args.batch_size,
        device=device
    )

    agent = make_agent(
        obs_shape=env.observation_space.shape,
        action_shape=[1], # discrete, 1 dimension!
        state_space_dim=env.state_dimension,
        args=args,
        device=device
    )

    try:
        load_step = 20000
        agent.load(model_dir, load_step)
        print("[Bisim] : Load learned model successful, step=",load_step)

    except:
        load_step = 0
        print("[Bisim] : No learned model, Creat new model")


    L = Logger(args.work_dir, use_tb=args.save_tb)

    episode, episode_reward, done = 0, 0, True
    start_time = time.time()
    for step in range(args.num_train_steps):
        if done:
            # if args.decoder_type == 'inverse':
            #     for i in range(1, args.k):  # fill k_obs with 0s if episode is done
            #         replay_buffer.k_obses[replay_buffer.idx - i] = 0
            if step > 0:
                L.log('train/duration', time.time() - start_time, step)
                start_time = time.time()
                L.dump(step)

            L.log('train/episode_reward', episode_reward, step)

            obs, rule_action = env.reset()
            done = False
            episode_reward = 0
            episode_step = 0
            episode += 1
            reward = 0
            L.log('train/episode', episode, step)
  
        
        # evaluate agent periodically
        if step % args.eval_freq == 0:
            L.log('eval/episode', episode, step)
            if args.save_model:
                print("[Bisim] : Saved Model! Step:",step + load_step)
                agent.save(model_dir, step + load_step)
            if args.save_buffer:
                replay_buffer.save(buffer_dir)
                print("[Bisim] : Saved Buffer!")

        # run training update
        if step >= args.init_steps:
            num_updates = args.init_steps if step == args.init_steps else 1
            for _ in range(num_updates):
                agent.update(replay_buffer, L, step) # Updated Transition and Reward Module

        
        curr_reward = reward
        action = Library.act(obs, rule_action, agent, stage=1)

        bisim, max_action, r_dist, transition_dist = agent.calculate_bisimulation_pess(obs, obs, 3)
        print("bisimulation:",bisim, max_action, r_dist, transition_dist)

        print("Predicted Reward:",agent.get_reward_prediction(obs, action))
        next_obs, reward, done, next_rule_action = env.step(action)
        print("Actual Reward:",reward)
        # allow infinit bootstrap
        # done_bool = 0 if episode_step + 1 == env._max_episode_steps else float(
        #     done
        # )
        episode_reward += reward
        if action == rule_action:
            Library.add_data(obs, action, reward, next_obs, done)
        replay_buffer.add(obs, action, curr_reward, reward, next_obs, done)
        # np.copyto(replay_buffer.k_obses[replay_buffer.idx - args.k], next_obs)

        obs = next_obs
        rule_action = next_rule_action
        episode_step += 1



if __name__ == '__main__':
    main()


