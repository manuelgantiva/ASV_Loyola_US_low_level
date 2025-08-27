import numpy as np
import torch
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Vector3, Quaternion

class DataConverter:
    """Utilities for converting between ROS messages, NumPy arrays, and PyTorch tensors."""
    
    # Coordinate system bounds
    MIN_X, MAX_X = -10.0, 30.0
    MIN_Y, MAX_Y = -10.0, 30.0
    
    @staticmethod
    def ros_to_numpy(msg):
        """Convert a ROS Float32MultiArray to a NumPy array."""
        return np.array(msg.data, dtype=np.float32)
    
    @staticmethod
    def numpy_to_ros(array, msg_type=Float32MultiArray):
        """Convert a NumPy array to a ROS message."""
        msg = msg_type()
        msg.data = array.flatten().astype(np.float32).tolist()
        return msg
    
    @staticmethod
    def numpy_to_tensor(array, device="cpu"):
        """Convert a NumPy array to a PyTorch tensor."""
        return torch.tensor(array, dtype=torch.float32, device=device)
    
    @staticmethod
    def tensor_to_numpy(tensor):
        """Convert a PyTorch tensor to a NumPy array."""
        return tensor.detach().cpu().numpy()
    
    @staticmethod
    def state_to_ppo_input(state_array, num_agents):
        """Extract agent states from the extended state array and reshape for PPO."""
        agent_states = state_array[:num_agents * 6]
        return agent_states.reshape((num_agents, 6))
    
    @staticmethod
    def ppo_output_to_actions(actions, num_agents):
        """Format PPO output for transmission to agents."""
        actions_np = np.array(actions)
        if actions_np.ndim == 1:
            actions_np = actions_np.reshape((num_agents, -1))
        return actions_np.flatten()
    
    @staticmethod
    def scale_action_to_physical(action_normalized, action_type):
        """Scale normalized action [-1,1] to physical range."""
        if action_type == "vyaw":
            return action_normalized * 0.5  # Scale to [-0.5, 0.5] rad/s
        elif action_type == "acceleration":
            return action_normalized * 0.1  # Scale to [-0.1, 0.1] m/s²
        return action_normalized
    
    @staticmethod
    def scale_physical_to_normalized(action_physical, action_type):
        """Scale physical action to normalized range [-1,1]."""
        if action_type == "vyaw":
            return action_physical / 0.5
        elif action_type == "acceleration":
            return action_physical / 0.1
        return action_physical
    
    @staticmethod
    def flatten_observations(observations):
        """Flatten observations for training buffer."""
        if isinstance(observations, np.ndarray):
            return observations.reshape(observations.shape[0], -1)
        return observations

    @staticmethod
    def prepare_sb3_training_data(state_array, action_array, reward_value, done_value):
        """Format data in the way SB3 expects for training."""
        obs = DataConverter.numpy_to_tensor(state_array)
        actions = DataConverter.numpy_to_tensor(action_array)
        rewards = np.array([float(reward_value)])
        dones = np.array([bool(done_value)])
        
        return obs, actions, rewards, dones

    @staticmethod
    def validate_position(position):
        """Ensure position is within reasonable bounds"""
        x, y = position[:2]
        x_clipped = np.clip(x, DataConverter.MIN_X, DataConverter.MAX_X)
        y_clipped = np.clip(y, DataConverter.MIN_Y, DataConverter.MAX_Y)
        return np.array([x_clipped, y_clipped])