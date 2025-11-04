#!/usr/bin/env python3
"""
Custom Actor-Critic network implementation for PPO.

This module provides the CustomActorCritic class that implements both the policy
(actor) and value function (critic) networks used in PPO training.
"""

import numpy as np
import torch
import torch.nn as nn


class CustomActorCritic(nn.Module):
    """
    Custom Actor-Critic network for PPO implementation.
    
    Actor network outputs action distribution parameters (mean and log_std).
    Critic network outputs value estimates V(s).
    """
    
    def __init__(self, obs_dim, action_dim, hidden_sizes=[64, 64], activation=nn.Tanh):
        super(CustomActorCritic, self).__init__()
        
        self.obs_dim = obs_dim
        self.action_dim = action_dim
        
        # Shared feature extraction layers
        layers = []
        prev_size = obs_dim
        for hidden_size in hidden_sizes:
            layers.append(nn.Linear(prev_size, hidden_size))
            layers.append(activation())
            prev_size = hidden_size
        
        self.shared_features = nn.Sequential(*layers)
        
        # Actor head (policy network)
        self.actor_mean = nn.Linear(prev_size, action_dim)
        # Learnable log standard deviation for action distribution
        self.actor_log_std = nn.Parameter(torch.zeros(action_dim))
        
        # Critic head (value network)
        self.critic = nn.Linear(prev_size, 1)
        
        # Initialize weights
        self._initialize_weights()
    
    def _initialize_weights(self):
        """Initialize network weights using orthogonal initialization (common in RL)"""
        for module in self.modules():
            if isinstance(module, nn.Linear):
                nn.init.orthogonal_(module.weight, gain=np.sqrt(2))
                nn.init.constant_(module.bias, 0.0)
        
        # Special initialization for policy output layer (smaller weights)
        nn.init.orthogonal_(self.actor_mean.weight, gain=0.01)
        nn.init.constant_(self.actor_mean.bias, 0.0)
        
        # Initialize log_std to reasonable values
        nn.init.constant_(self.actor_log_std, -0.5)  # std ≈ 0.6
    
    def forward(self, obs):
        """
        Forward pass for action selection.
        
        Args:
            obs: Observation tensor [batch_size, obs_dim]
            
        Returns:
            actions: Sampled actions [batch_size, action_dim]
            values: Value estimates [batch_size, 1]  
            log_probs: Log probabilities of actions [batch_size, action_dim]
        """
        # Extract shared features
        features = self.shared_features(obs)
        
        # Actor: get action distribution parameters
        action_mean = self.actor_mean(features)
        action_std = torch.exp(self.actor_log_std.expand_as(action_mean))
        
        # Create action distribution
        action_dist = torch.distributions.Normal(action_mean, action_std)
        
        # Sample actions
        actions = action_dist.sample()
        
        # Apply tanh squashing to bound actions in [-1, 1]
        actions_tanh = torch.tanh(actions)
        
        # Compute log probabilities with tanh correction
        log_probs = action_dist.log_prob(actions)
        # Tanh correction: log_prob(tanh(x)) = log_prob(x) - log(1 - tanh²(x))
        log_probs -= torch.log(1 - actions_tanh.pow(2) + 1e-6)
        log_probs = log_probs.sum(dim=-1, keepdim=True)
        
        # Critic: get value estimate
        values = self.critic(features)
        
        return actions_tanh, values, log_probs
    
    def evaluate_actions(self, obs, actions):
        """
        Evaluate actions for training (used in policy updates).
        
        Args:
            obs: Observation tensor [batch_size, obs_dim]
            actions: Action tensor [batch_size, action_dim]
            
        Returns:
            values: Value estimates [batch_size, 1]
            log_probs: Log probabilities [batch_size, action_dim] 
            entropy: Action distribution entropy [batch_size, 1]
        """
        # Extract shared features
        features = self.shared_features(obs)
        
        # Actor: get action distribution parameters
        action_mean = self.actor_mean(features)
        action_std = torch.exp(self.actor_log_std.expand_as(action_mean))
        
        # Create action distribution
        action_dist = torch.distributions.Normal(action_mean, action_std)
        
        # Inverse tanh to get pre-tanh actions for log_prob calculation
        # atanh is unstable at boundaries, so clip actions slightly
        actions_clipped = torch.clamp(actions, -1 + 1e-6, 1 - 1e-6)
        actions_pretanh = torch.atanh(actions_clipped)
        
        # Compute log probabilities 
        log_probs = action_dist.log_prob(actions_pretanh)
        # Apply tanh correction
        log_probs -= torch.log(1 - actions.pow(2) + 1e-6)
        log_probs = log_probs.sum(dim=-1, keepdim=True)
        
        # Compute entropy (before tanh transformation)
        entropy = action_dist.entropy().sum(dim=-1, keepdim=True)
        
        # Critic: get value estimate
        values = self.critic(features)
        
        return values, log_probs, entropy
    
    def get_values(self, obs):
        """Get value estimates only (used for bootstrap values)"""
        features = self.shared_features(obs)
        return self.critic(features)
