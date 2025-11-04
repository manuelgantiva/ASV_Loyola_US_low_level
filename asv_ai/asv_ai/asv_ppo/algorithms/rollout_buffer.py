#!/usr/bin/env python3
"""
Rollout buffer implementation for PPO.

This module provides the CustomRolloutBuffer class for storing transitions
and computing advantages using Generalized Advantage Estimation (GAE).
"""

import torch


class CustomRolloutBuffer:
    """
    Custom rollout buffer implementation for PPO.
    
    Stores transitions and computes returns using Generalized Advantage Estimation (GAE).
    """
    
    def __init__(self, buffer_size, obs_dim, action_dim, device='cpu', gamma=0.99, gae_lambda=0.95):
        self.buffer_size = buffer_size
        self.obs_dim = obs_dim
        self.action_dim = action_dim
        self.device = device
        self.gamma = gamma
        self.gae_lambda = gae_lambda
        
        # Storage arrays
        self.observations = torch.zeros((buffer_size, obs_dim), dtype=torch.float32, device=device)
        self.actions = torch.zeros((buffer_size, action_dim), dtype=torch.float32, device=device)
        self.rewards = torch.zeros(buffer_size, dtype=torch.float32, device=device)
        self.values = torch.zeros(buffer_size, dtype=torch.float32, device=device)
        self.log_probs = torch.zeros(buffer_size, dtype=torch.float32, device=device)
        self.episode_starts = torch.zeros(buffer_size, dtype=torch.bool, device=device)
        
        # Computed during GAE
        self.advantages = torch.zeros(buffer_size, dtype=torch.float32, device=device)
        self.returns = torch.zeros(buffer_size, dtype=torch.float32, device=device)
        
        self.pos = 0
        self.full = False
        
    def add(self, obs, actions, rewards, episode_starts, values, log_probs):
        """
        Add a transition to the buffer.
        
        Args:
            obs: Observations [batch_size, obs_dim] or [obs_dim]
            actions: Actions [batch_size, action_dim] or [action_dim]  
            rewards: Rewards [batch_size] or scalar
            episode_starts: Episode start flags [batch_size] or boolean
            values: Value estimates [batch_size] or scalar
            log_probs: Log probabilities [batch_size] or scalar
        """
        # Convert inputs to tensors and ensure correct shape
        obs = torch.as_tensor(obs, dtype=torch.float32, device=self.device)
        actions = torch.as_tensor(actions, dtype=torch.float32, device=self.device)
        rewards = torch.as_tensor(rewards, dtype=torch.float32, device=self.device)
        episode_starts = torch.as_tensor(episode_starts, dtype=torch.bool, device=self.device)
        values = torch.as_tensor(values, dtype=torch.float32, device=self.device)
        log_probs = torch.as_tensor(log_probs, dtype=torch.float32, device=self.device)
        
        # Handle batch dimension
        if obs.dim() == 1:
            obs = obs.unsqueeze(0)
        if actions.dim() == 1:
            actions = actions.unsqueeze(0)
        if rewards.dim() == 0:
            rewards = rewards.unsqueeze(0)
        if episode_starts.dim() == 0:
            episode_starts = episode_starts.unsqueeze(0)
        if values.dim() == 0:
            values = values.unsqueeze(0)
        if log_probs.dim() == 0:
            log_probs = log_probs.unsqueeze(0)
        
        batch_size = obs.shape[0]
        
        # Add to buffer
        for i in range(batch_size):
            if self.pos >= self.buffer_size:
                # Buffer is full, stop adding
                self.full = True
                break
                
            self.observations[self.pos] = obs[i]
            self.actions[self.pos] = actions[i] 
            self.rewards[self.pos] = rewards[i]
            self.episode_starts[self.pos] = episode_starts[i]
            self.values[self.pos] = values[i]
            self.log_probs[self.pos] = log_probs[i]
            
            self.pos += 1
            
        if self.pos >= self.buffer_size:
            self.full = True
    
    def compute_returns_and_advantage(self, last_values, dones):
        """
        Compute returns and advantages using Generalized Advantage Estimation (GAE).
        
        Args:
            last_values: Value estimates for the last observations [n_envs]
            dones: Done flags for the last step [n_envs]
        """
        last_values = torch.as_tensor(last_values, dtype=torch.float32, device=self.device).flatten()
        dones = torch.as_tensor(dones, dtype=torch.bool, device=self.device).flatten()
        
        # GAE computation
        last_gae_lam = 0
        
        # Work backwards through the buffer
        for step in reversed(range(self.pos)):
            if step == self.pos - 1:
                # Last step: use provided last_values and dones
                next_non_terminal = ~dones[0] if len(dones) > 0 else True
                next_values = last_values[0] if len(last_values) > 0 else 0
            else:
                # Use values from next step in buffer
                next_non_terminal = ~self.episode_starts[step + 1] 
                next_values = self.values[step + 1]
            
            # Temporal difference error
            delta = self.rewards[step] + self.gamma * next_values * next_non_terminal - self.values[step]
            
            # GAE advantage
            last_gae_lam = delta + self.gamma * self.gae_lambda * next_non_terminal * last_gae_lam
            self.advantages[step] = last_gae_lam
        
        # Compute returns: R = A + V
        self.returns[:self.pos] = self.advantages[:self.pos] + self.values[:self.pos]
    
    def get(self, batch_size=None):
        """
        Generate random minibatches for training.
        
        Args:
            batch_size: Size of each minibatch. If None, return all data.
            
        Yields:
            Minibatch data as a named tuple
        """
        if batch_size is None:
            batch_size = self.pos
            
        # Create indices for all stored transitions
        indices = torch.randperm(self.pos, device=self.device)
        
        # Yield minibatches
        start_idx = 0
        while start_idx < self.pos:
            end_idx = min(start_idx + batch_size, self.pos)
            batch_indices = indices[start_idx:end_idx]
            
            yield RolloutBufferSamples(
                observations=self.observations[batch_indices],
                actions=self.actions[batch_indices],
                old_values=self.values[batch_indices],
                old_log_prob=self.log_probs[batch_indices],
                advantages=self.advantages[batch_indices],
                returns=self.returns[batch_indices]
            )
            
            start_idx = end_idx
    
    def reset(self):
        """Reset buffer to empty state"""
        self.pos = 0
        self.full = False
        
    def __len__(self):
        """Return number of stored transitions"""
        return self.pos


class RolloutBufferSamples:
    """
    Data class for rollout buffer samples (mimics SB3 interface).
    """
    def __init__(self, observations, actions, old_values, old_log_prob, advantages, returns):
        self.observations = observations
        self.actions = actions  
        self.old_values = old_values
        self.old_log_prob = old_log_prob
        self.advantages = advantages
        self.returns = returns
