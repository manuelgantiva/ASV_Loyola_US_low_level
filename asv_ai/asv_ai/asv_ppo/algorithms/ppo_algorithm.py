#!/usr/bin/env python3
"""
Custom PPO algorithm implementation.

This module provides the CustomPPO class that implements the Proximal Policy
Optimization algorithm with compatibility for the stable-baselines3 interface.
"""

import numpy as np
import torch
from gymnasium import spaces

from ..networks import CustomActorCritic


class CustomPPO:
    """
    Custom PPO implementation to replace stable-baselines3 PPO.
    
    Maintains similar interface for easy integration with existing code.
    """
    
    def __init__(self, obs_dim, action_dim, device='cpu', learning_rate=3e-4, 
                 clip_range=0.2, ent_coef=0.0, vf_coef=0.5, max_grad_norm=0.5, target_kl=None):
        self.obs_dim = obs_dim
        self.action_dim = action_dim
        self.device = device
        self.clip_range = clip_range
        self.ent_coef = ent_coef
        self.vf_coef = vf_coef
        self.max_grad_norm = max_grad_norm
        self.target_kl = target_kl
        
        # Create policy network
        self.policy = CustomActorCritic(obs_dim, action_dim).to(device)
        
        # Create optimizer
        self.optimizer = torch.optim.Adam(self.policy.parameters(), lr=learning_rate)
        
        # Learning rate scheduler (for compatibility)
        self.lr_schedule = learning_rate
        self.learning_rate = learning_rate
        
        # Training statistics
        self._n_updates = 0
        
        # For compatibility with existing code
        self.observation_space = spaces.Box(low=-np.inf, high=np.inf, shape=(obs_dim,), dtype=np.float32)
        self.action_space = spaces.Box(low=-1, high=1, shape=(action_dim,), dtype=np.float32)
    
    def predict(self, obs, deterministic=False):
        """
        Predict action for given observation (for compatibility with SB3 interface).
        
        Args:
            obs: Observation array [obs_dim] or [batch_size, obs_dim]
            deterministic: If True, return mean action instead of sampling
            
        Returns:
            actions: Predicted actions [action_dim] or [batch_size, action_dim]
            states: None (for compatibility)
        """
        self.policy.eval()
        
        with torch.no_grad():
            obs_tensor = torch.as_tensor(obs, dtype=torch.float32, device=self.device)
            if obs_tensor.dim() == 1:
                obs_tensor = obs_tensor.unsqueeze(0)
                single_obs = True
            else:
                single_obs = False
            
            if deterministic:
                # Return mean action (deterministic policy)
                features = self.policy.shared_features(obs_tensor)
                actions = torch.tanh(self.policy.actor_mean(features))
            else:
                # Sample from policy
                actions, _, _ = self.policy(obs_tensor)
            
            actions = actions.cpu().numpy()
            
            if single_obs:
                actions = actions[0]
                
        return actions, None
    
    def save(self, path):
        """
        Save the model to disk.
        
        Args:
            path: File path to save the model (should end with .zip for compatibility)
        """
        # Convert .zip to .pth for torch save format
        if path.endswith('.zip'):
            torch_path = path.replace('.zip', '.pth')
        else:
            torch_path = path
            
        save_dict = {
            'policy_state_dict': self.policy.state_dict(),
            'optimizer_state_dict': self.optimizer.state_dict(),
            'obs_dim': self.obs_dim,
            'action_dim': self.action_dim,
            'learning_rate': self.learning_rate,
            'clip_range': self.clip_range,
            'ent_coef': self.ent_coef,
            'vf_coef': self.vf_coef,
            'max_grad_norm': self.max_grad_norm,
            '_n_updates': self._n_updates
        }
        
        torch.save(save_dict, torch_path)
    
    @classmethod
    def load(cls, path, obs_dim=None, action_dim=None, device='cpu'):
        """
        Load a saved model.
        
        Args:
            path: File path to load the model from
            obs_dim: Observation dimension (required if not in saved file)
            action_dim: Action dimension (required if not in saved file) 
            device: Device to load the model on
            
        Returns:
            CustomPPO instance with loaded parameters
        """
        # Convert .zip to .pth for torch load format
        if path.endswith('.zip'):
            torch_path = path.replace('.zip', '.pth')
        else:
            torch_path = path
            
        checkpoint = torch.load(torch_path, map_location=device)
        
        # Get dimensions from checkpoint or use provided values
        obs_dim = checkpoint.get('obs_dim', obs_dim)
        action_dim = checkpoint.get('action_dim', action_dim)
        
        if obs_dim is None or action_dim is None:
            raise ValueError("obs_dim and action_dim must be provided either in checkpoint or as arguments")
        
        # Create new instance
        model = cls(
            obs_dim=obs_dim,
            action_dim=action_dim,
            device=device,
            learning_rate=checkpoint.get('learning_rate', 3e-4),
            clip_range=checkpoint.get('clip_range', 0.2),
            ent_coef=checkpoint.get('ent_coef', 0.0),
            vf_coef=checkpoint.get('vf_coef', 0.5),
            max_grad_norm=checkpoint.get('max_grad_norm', 0.5)
        )
        
        # Load state dicts
        model.policy.load_state_dict(checkpoint['policy_state_dict'])
        model.optimizer.load_state_dict(checkpoint['optimizer_state_dict'])
        model._n_updates = checkpoint.get('_n_updates', 0)
        
        return model
