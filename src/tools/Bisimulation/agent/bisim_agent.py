# Copyright (c) Facebook, Inc. and its affiliates.
# All rights reserved.

# This source code is licensed under the license found in the
# LICENSE file in the root directory of this source tree.

import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F

import utils
from sac_ae import  Actor, Critic, LOG_FREQ, weight_init
from transition_model import make_transition_model
from decoder import make_decoder           



class BisimAgent(object):
    """Bisimulation metric algorithm."""
    def __init__(
        self,
        obs_shape,
        action_shape,
        device,
        state_space_dim,
        transition_model_type,
        hidden_dim=256,
        discount=0.99,
        init_temperature=0.01,
        # alpha_lr=1e-3,
        # alpha_beta=0.9,
        # actor_lr=1e-3,
        # actor_beta=0.9,
        # actor_log_std_min=-10,
        # actor_log_std_max=2,
        # actor_update_freq=2,
        # encoder_stride=2,
        # critic_lr=1e-3,
        # critic_beta=0.9,
        # critic_tau=0.005,
        # critic_target_update_freq=2,
        # encoder_type='pixel',
        # encoder_feature_dim=50,
        # encoder_lr=1e-3,
        # encoder_tau=0.005,
        # decoder_type='pixel',
        decoder_lr=0.01,
        # decoder_update_freq=1,
        # decoder_latent_lambda=0.0,
        decoder_weight_lambda=0.0,
        # num_layers=4,
        # num_filters=32,
        bisim_coef=0.5
    ):
        self.device = device
        self.discount = discount
        # self.critic_tau = critic_tau
        # self.encoder_tau = encoder_tau
        # self.actor_update_freq = actor_update_freq
        # self.critic_target_update_freq = critic_target_update_freq
        # self.decoder_update_freq = decoder_update_freq
        # self.decoder_latent_lambda = decoder_latent_lambda
        self.transition_model_type = transition_model_type
        self.bisim_coef = bisim_coef
        self.state_space_dim = state_space_dim

        self.action_shape = action_shape
        self.transition_model = make_transition_model(
            transition_model_type, state_space_dim, action_shape
        ).to(device)

        self.reward_decoder = nn.Sequential(
        nn.Linear(state_space_dim + action_shape[0], 256),
            # nn.LayerNorm(128),
            # nn.ReLU(),
            nn.Sigmoid(),
            nn.Linear(256, 512),
            nn.Sigmoid(),
            nn.Linear(512, 128),
            nn.Sigmoid(),
            nn.Linear(128, 1)).to(device)

        # optimizer for decoder
        self.reward_decoder_optimizer = torch.optim.Adam(
            list(self.reward_decoder.parameters()) + list(self.transition_model.parameters()),
            lr=decoder_lr,
            weight_decay=decoder_weight_lambda
        )
        self.train()

    def train(self, training=True):
        self.training = training
        
    # @property
    # def alpha(self):
    #     return self.log_alpha.exp()

    # def select_action(self, obs):
    #     with torch.no_grad():
    #         obs = torch.FloatTensor(obs).to(self.device)
    #         obs = obs.unsqueeze(0)
    #         mu, _, _, _ = self.actor(
    #             obs, compute_pi=False, compute_log_pi=False
    #         )
    #         return mu.cpu().data.numpy().flatten()

    # def sample_action(self, obs):
    #     with torch.no_grad():
    #         obs = torch.FloatTensor(obs).to(self.device)
    #         obs = obs.unsqueeze(0)
    #         mu, pi, _, _ = self.actor(obs, compute_log_pi=False)
    #         return pi.cpu().data.numpy().flatten()


    # def update_encoder(self, obs, action, reward, L, step):
    #     h = self.critic.encoder(obs)            

    #     # Sample random states across episodes at random
    #     batch_size = obs.size(0)
    #     perm = np.random.permutation(batch_size)
    #     h2 = h[perm]

    #     with torch.no_grad():
    #         # action, _, _, _ = self.actor(obs, compute_pi=False, compute_log_pi=False)
    #         pred_next_latent_mu1, pred_next_latent_sigma1 = self.transition_model(torch.cat([h, action], dim=1))
    #         # reward = self.reward_decoder(pred_next_latent_mu1)
    #         reward2 = reward[perm]
    #     if pred_next_latent_sigma1 is None:
    #         pred_next_latent_sigma1 = torch.zeros_like(pred_next_latent_mu1)
    #     if pred_next_latent_mu1.ndim == 2:  # shape (B, Z), no ensemble
    #         pred_next_latent_mu2 = pred_next_latent_mu1[perm]
    #         pred_next_latent_sigma2 = pred_next_latent_sigma1[perm]
    #     elif pred_next_latent_mu1.ndim == 3:  # shape (B, E, Z), using an ensemble
    #         pred_next_latent_mu2 = pred_next_latent_mu1[:, perm]
    #         pred_next_latent_sigma2 = pred_next_latent_sigma1[:, perm]
    #     else:
    #         raise NotImplementedError

    #     z_dist = F.smooth_l1_loss(h, h2, reduction='none')
    #     r_dist = F.smooth_l1_loss(reward, reward2, reduction='none')
    #     if self.transition_model_type == '':
    #         transition_dist = F.smooth_l1_loss(pred_next_latent_mu1, pred_next_latent_mu2, reduction='none')
    #     else:
    #         transition_dist = torch.sqrt(
    #             (pred_next_latent_mu1 - pred_next_latent_mu2).pow(2) +
    #             (pred_next_latent_sigma1 - pred_next_latent_sigma2).pow(2)
    #         )
    #         # transition_dist  = F.smooth_l1_loss(pred_next_latent_mu1, pred_next_latent_mu2, reduction='none') \
    #         #     +  F.smooth_l1_loss(pred_next_latent_sigma1, pred_next_latent_sigma2, reduction='none')

    #     bisimilarity = r_dist + self.discount * transition_dist
    #     loss = (z_dist - bisimilarity).pow(2).mean()
    #     L.log('train_ae/encoder_loss', loss, step)
    #     return loss

    def update_transition_reward_model(self, obs, action, next_obs, reward, L, step):
        obs_with_action = torch.cat([obs, action], dim=1)
        pred_next_latent_mu, pred_next_latent_sigma = self.transition_model(obs_with_action)
        if pred_next_latent_sigma is None:
            pred_next_latent_sigma = torch.ones_like(pred_next_latent_mu)

        diff = (pred_next_latent_mu - next_obs.detach()) / pred_next_latent_sigma
        loss = torch.mean(0.5 * diff.pow(2) + torch.log(pred_next_latent_sigma))
        L.log('train_ae/transition_loss', loss, step)

        pred_next_reward = self.reward_decoder(obs_with_action)
        reward_loss = F.mse_loss(pred_next_reward, reward)
        # print("pred_next_reward",pred_next_reward)
        # print("reward",reward)
        # print("reward_loss",reward_loss)
        total_loss = loss + reward_loss
        return total_loss

    def update(self, replay_buffer, L, step):
        obs, action, _, reward, next_obs, not_done = replay_buffer.sample()
        L.log('train/batch_reward', reward.mean(), step)

        transition_reward_loss = self.update_transition_reward_model(obs, action, next_obs, reward, L, step)
        # encoder_loss = self.update_encoder(obs, action, reward, L, step)
        # total_loss = self.bisim_coef * encoder_loss + transition_reward_loss
        total_loss = transition_reward_loss
        # self.encoder_optimizer.zero_grad()
        self.reward_decoder_optimizer.zero_grad()
        total_loss.backward()
        # self.encoder_optimizer.step()
        self.reward_decoder_optimizer.step()

        print("[Bisim] : Updated all models! Step:",step)

    def get_reward_prediction(self, obs, action):
        np_obs = np.empty((1, self.state_space_dim), dtype=np.float32)
        np.copyto(np_obs[0], obs)
        obs = torch.as_tensor(np_obs, device=self.device).float()
        np_action = np.empty((1, 1), dtype=np.float32)
        np.copyto(np_action[0], action)
        action = torch.as_tensor(np_action, device=self.device)

        with torch.no_grad():
            obs_with_action = torch.cat([obs, action], dim=1)
            return self.reward_decoder(obs_with_action)

    def get_trans_prediction(self, obs, action):
        np_obs = np.empty((1, self.state_space_dim), dtype=np.float32)
        np.copyto(np_obs[0], obs)
        obs = torch.as_tensor(np_obs, device=self.device).float()
        np_action = np.empty((1, 1), dtype=np.float32)
        np.copyto(np_action[0], action)
        action = torch.as_tensor(np_action, device=self.device)
        with torch.no_grad():
            obs_with_action = torch.cat([obs, action], dim=1)
            return self.transition_model(obs_with_action)

    def calculate_bisimulation_pess(self, state_corner, state_normal, action_normal):
        np_obs = np.empty((1, self.state_space_dim), dtype=np.float32)
        np.copyto(np_obs[0], state_corner)
        state_corner = torch.as_tensor(np_obs, device=self.device).float()
        np_obs = np.empty((1, self.state_space_dim), dtype=np.float32)
        np.copyto(np_obs[0], state_normal)
        state_normal = torch.as_tensor(np_obs, device=self.device).float()
        np_action = np.empty((1, 1), dtype=np.float32)
        np.copyto(np_action[0], action_normal)
        action_normal = torch.as_tensor(np_action, device=self.device)
        with torch.no_grad():
            bisim_for_corner_action = []
            for action in self.action_shape:
                np_action = np.empty((1, 1), dtype=np.float32)
                np.copyto(np_action[0], action)
                action = torch.as_tensor(np_action, device=self.device)

                obs_with_action = torch.cat([state_normal, action_normal], dim=1)
                normal_reward = self.reward_decoder(obs_with_action)

                obs_with_action = torch.cat([state_corner, action], dim=1)
                corner_reward = self.reward_decoder(obs_with_action)
                r_dist = F.smooth_l1_loss(normal_reward, corner_reward, reduction='none')

                pred_next_latent_mu1, pred_next_latent_sigma1 = self.transition_model(torch.cat([state_normal, action_normal], dim=1))
                pred_next_latent_mu2, pred_next_latent_sigma2 = self.transition_model(torch.cat([state_corner, action], dim=1))

                transition_dist = torch.sqrt((pred_next_latent_mu1 - pred_next_latent_mu2).pow(2) + (pred_next_latent_sigma1 - pred_next_latent_sigma2).pow(2))
                bisim_for_corner_action.append(r_dist + self.discount * transition_dist)

        
        max_action = bisim_for_corner_action.index(max(bisim_for_corner_action))
        return bisim_for_corner_action[max_action], max_action, r_dist, transition_dist

    def calculate_bisimulation_optimal(self, state_corner, state_normal, action_normal):
        with torch.no_grad():
            bisim_for_corner_action = []
            for action in self.action_shape:
                obs_with_action = state_normal.append(action_normal)
                normal_reward = self.reward_decoder(obs_with_action)

                obs_with_action = state_corner.append(action)
                corner_reward = self.reward_decoder(obs_with_action)
                r_dist = F.smooth_l1_loss(normal_reward, corner_reward, reduction='none')

                pred_next_latent_mu1, pred_next_latent_sigma1 = self.transition_model(torch.cat([state_normal, action_normal], dim=1))
                pred_next_latent_mu2, pred_next_latent_sigma2 = self.transition_model(torch.cat([state_corner, action], dim=1))

                transition_dist = torch.sqrt((pred_next_latent_mu1 - pred_next_latent_mu2).pow(2) + (pred_next_latent_sigma1 - pred_next_latent_sigma2).pow(2))
                bisim_for_corner_action.append(r_dist + self.discount * transition_dist)

        
        min_action = bisim_for_corner_action.index(min(bisim_for_corner_action))
        return bisim_for_corner_action[min_action], min_action

            
    def save(self, model_dir, step):
        # torch.save(
        #     self.actor.state_dict(), '%s/actor_%s.pt' % (model_dir, step)
        # )
        # torch.save(
        #     self.critic.state_dict(), '%s/critic_%s.pt' % (model_dir, step)
        # )
        torch.save(
            self.reward_decoder.state_dict(),
            '%s/reward_decoder_%s.pt' % (model_dir, step)
        )
        torch.save(
            self.transition_model.state_dict(),
            '%s/transition_model%s.pt' % (model_dir, step)
        )


    def load(self, model_dir, step):
        # self.actor.load_state_dict(
        #     torch.load('%s/actor_%s.pt' % (model_dir, step))
        # )
        # self.critic.load_state_dict(
        #     torch.load('%s/critic_%s.pt' % (model_dir, step))
        # )
        self.reward_decoder.load_state_dict(
            torch.load('%s/reward_decoder_%s.pt' % (model_dir, step))
        )
        self.transition_model.load_state_dict(
            torch.load('%s/transition_model%s.pt' % (model_dir, step))
        )


