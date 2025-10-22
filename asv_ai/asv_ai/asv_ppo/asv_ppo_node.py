#!/usr/bin/env python3
import json
import os
import time

import numpy as np
import rclpy
import torch as th
from gymnasium import spaces
from rclpy.node import Node
from stable_baselines3 import PPO
from stable_baselines3.common.buffers import RolloutBuffer
from std_msgs.msg import Bool, Float32, Float32MultiArray
from std_srvs.srv import Trigger

from ..utils.data_conversion import DataConverter


class ASVPPONode(Node):
    def __init__(self):
        super().__init__('asv_ppo_node')

        # Parameters - Environment Configuration
        self.declare_parameter('num_agents', 2)
        self.declare_parameter('model_path', '')
        self.declare_parameter('rollout_dir', os.path.expanduser('~/Desktop/ASV_Rollouts'))
        self.declare_parameter('rollout_save_every', 50)
        self.declare_parameter('rollout_collection_enabled', True)
        
        # Parameters - State/Action Space Dimensions (generalizable for any robot/environment)
        self.declare_parameter('obs_dim_per_agent', 6)  # Default: [x, y, yaw, vx, vy, vyaw]
        self.declare_parameter('action_dim_per_agent', 2)  # Default: [vyaw_rate, acceleration]

        # Resolve parameters
        self.num_agents = self.get_parameter('num_agents').value
        self.model_path = self.get_parameter('model_path').value
        self.rollout_dir = os.path.expanduser(self.get_parameter('rollout_dir').value)
        self.rollout_save_every = int(self.get_parameter('rollout_save_every').value)
        self.rollout_collection_enabled = self.get_parameter('rollout_collection_enabled').value
        
        # Resolve space dimensions (generalizable)
        self.obs_dim_per_agent = self.get_parameter('obs_dim_per_agent').value
        self.action_dim_per_agent = self.get_parameter('action_dim_per_agent').value
        
        os.makedirs(self.rollout_dir, exist_ok=True)

        # Runtime buffers/state - Temporal sequence for RL training
        # For RL we need transition tuples: (s_t-1, a_t-1, r_t, s_t)
        self.rollout_data = []  # Collected rollout data for logging
        self.last_state = None  # Full raw state from previous step (for rollout logging)
        self.prev_obs = None    # Processed observation from PREVIOUS step (s_t-1) - where action was taken
        self.last_action = None # Action taken at previous step (a_t-1)
        self.last_reward = None # Reward received at current step (r_t)
        self.last_values = None # Value estimate from previous step (V(s_t-1))
        self.last_log_probs = None # Log probability of action from previous step
        self.current_obs = None # CURRENT observation (s_t) - result after taking action
        self.episode_start = np.ones(1, dtype=bool)  # Start of first episode
        self.model_ready = False
        self._pending_state = None  # buffer a state if it arrives before model is ready
        self.episode_id = 0
        self.step_idx = 0

        # Training statistics
        self.training_stats = {
            'total_training_sessions': 0,
            'total_transitions_trained': 0,
            'average_buffer_size_at_training': 0,
            'memory_resets': 0
        }
        self.episode_rewards = []
        self.current_episode_reward = 0.0

        # Add training parameters
        self.declare_parameter('training_enabled', False)
        self.declare_parameter('train_frequency', 1000)  # How many steps before training
        self.declare_parameter('n_epochs', 10)  # PPO training epochs
        self.declare_parameter('batch_size', 64)  # PPO batch size
        self.declare_parameter('gamma', 0.99)  # Discount factor
        self.declare_parameter('gae_lambda', 0.95)  # GAE lambda
        self.declare_parameter('max_episodes', 1000)

        # Get training parameters
        self.training_enabled = self.get_parameter('training_enabled').value
        self.train_frequency = self.get_parameter('train_frequency').value
        self.n_epochs = self.get_parameter('n_epochs').value
        self.batch_size = self.get_parameter('batch_size').value
        self.gamma = self.get_parameter('gamma').value
        self.gae_lambda = self.get_parameter('gae_lambda').value
        self.max_episodes = self.get_parameter('max_episodes').value

        # Publishers and Subscribers FIRST to avoid missing early messages
        self.state_sub = self.create_subscription(Float32MultiArray, '/environment/state', self.state_callback, 10)
        self.reward_sub = self.create_subscription(Float32, '/environment/reward', self.reward_callback, 10)
        self.done_sub = self.create_subscription(Bool, '/environment/done', self.done_callback, 10)

        self.action_pub = self.create_publisher(Float32MultiArray, '/ppo/action', 10)

        self.reset_client = self.create_client(Trigger, '/environment/reset')

        # Define observation and action spaces for PPO model (generalizable via parameters)
        # These stay in PPO node as they are model-specific, not environment-specific
        obs_dim = self.num_agents * self.obs_dim_per_agent
        action_dim = self.num_agents * self.action_dim_per_agent

        self.observation_space = spaces.Box(
            low=-np.inf, high=np.inf,
            shape=(obs_dim,),
            dtype=np.float32
        )
        self.action_space = spaces.Box(
            low=-1, high=1,
            shape=(action_dim,),
            dtype=np.float32
        )

        # Smart model loading: try multiple sources in order of preference
        loaded_model_path = self._find_and_load_best_model()
        if loaded_model_path:
            self.get_logger().info(f'Successfully loaded model from {loaded_model_path}')
        else:
            # Create new PPO model with explicit spaces (no environment needed)
            self.model = PPO(
                "MlpPolicy",
                env=None,
                verbose=0,
                _init_setup_model=False  # We'll set spaces manually
            )
            # Manually set the spaces
            self.model.observation_space = self.observation_space
            self.model.action_space = self.action_space
            # Now initialize the model
            self.model._setup_model()
            self.get_logger().info('No existing model found, starting with new PPO model.')

        self.model_ready = True
        # If we received a state while loading, process it now
        if self._pending_state is not None:
            self.get_logger().info('Processing buffered environment state after model init.')
            self._predict_and_publish(self._pending_state)
            self._pending_state = None

        # Add after model initialization:
        if self.training_enabled:
            # observation_space and action_space already defined above
            
            # Create training buffer with optimized configuration
            self.buffer_size = 200  # 2x larger for better experience diversity
            self.min_training_size = 50  # Train when 50 transitions available
            self.memory_limit = 500  # Reset buffer when it hits memory limit

            self.training_buffer = RolloutBuffer(
                buffer_size=self.buffer_size,
                observation_space=self.observation_space,
                action_space=self.action_space,
                device=self.model.device,
                gamma=self.gamma,
                gae_lambda=self.gae_lambda,
                n_envs=1
            )

            # Training state variables (note: current_obs already defined above)
            self.values = []  # Store value estimates
            self.log_probs = []  # Store log probabilities
            # self.current_obs already initialized above in Runtime buffers section
            self.dones = np.zeros(1, dtype=bool)  # Episode termination flags
            self.episode_start = np.zeros(1, dtype=bool)  # Episode start flags

            # Create optimized training timer (every 10 seconds instead of 30)
            self.training_timer = self.create_timer(10.0, self.train_model)

            self.reset_timer = self.create_timer(20.0, self.check_and_reset)  # Reduced frequency

            self.get_logger().info("PPO training mode enabled")

            self.get_logger().info(f'ASV PPO Node started with {self.num_agents} agents')

        # New - create session timestamp at startup for consistent file naming
        self.session_start_time = self.get_clock().now()
        self.formatted_timestamp = self.session_start_time.to_msg()
        self.session_id = time.strftime(
            "%Y-%m-%d_%H-%M-%S",
            time.localtime(self.formatted_timestamp.sec)
        )

        # Single file for the entire session
        self.rollout_filename = os.path.join(
            self.rollout_dir,
            f"asv_formation_{self.session_id}.json"
        )

        # Initialize the file with metadata
        self._initialize_rollout_file()

    def state_callback(self, msg):
        try:
            # Convert ROS message to NumPy array
            flat_state = DataConverter.ros_to_numpy(msg)
            self.get_logger().info(f'PPO received state with shape {flat_state.shape}', throttle_duration_sec=1)

            # If state buffer is full, we need to reset it before adding more
            if hasattr(self, 'training_buffer') and self.training_buffer.full:
                self.get_logger().info("Training buffer full, resetting to allow more training data")
                self.training_buffer.reset()

            # Check if agents are out of bounds and log the problematic positions
            if self._agents_out_of_bounds(flat_state):
                problematic_positions = []
                for i in range(self.num_agents):
                    # Use parameterized dimensions instead of hardcoded values
                    agent_x = flat_state[i * self.obs_dim_per_agent]
                    agent_y = flat_state[i * self.obs_dim_per_agent + 1]
                    problematic_positions.append([agent_x, agent_y])

                self.get_logger().warn(f"Agents detected out of bounds at positions: {problematic_positions}")
                self._finalize_episode()  # Save current episode data
                self.reset_environment()  # Request reset
                return

            if not self.model_ready:
                # Buffer the latest state until model is ready
                self._pending_state = flat_state
                self.get_logger().info('Model not ready yet; buffering environment state.', throttle_duration_sec=1)
                return

            self._predict_and_publish(flat_state)
        except Exception as e:
            self.get_logger().error(f'Error in state_callback: {str(e)}')

    def _predict_and_publish(self, flat_state: np.ndarray):
        """Process environment state, predict actions, and publish to ROS topics."""
        try:
            # Format state for the model
            agent_states = DataConverter.state_to_ppo_input(flat_state, self.num_agents)
            self.current_obs = agent_states.reshape(-1)  # Store current observation (s_t)

            # Get values and log_probs for rollout buffer
            with th.no_grad():
                # Reshape to (1, -1) instead of keeping 2D
                obs_tensor = DataConverter.numpy_to_tensor(self.current_obs)
                obs_tensor = obs_tensor.reshape(1, -1)  # Reshape to (1, 12) - batch of 1 with 12 features

                # Call policy and get results
                actions, values, log_probs = self.model.policy(obs_tensor)

                # Convert to numpy first
                actions_np = actions.cpu().numpy()

                # Calculate exploration factor that decreases over time
                exploration_factor = max(0.1, 1.0 - (self.episode_id / 200.0))

                # Add exploration noise that decreases over time
                if self.training_enabled:
                    noise = np.random.normal(0, exploration_factor * 0.3, actions_np.shape)
                    actions_np = np.clip(actions_np + noise, -1, 1)

            # If training is enabled, add to buffer with smart memory management
            # We create transition (s_t-1, a_t-1, r_t, s_t) using prev_obs from last step
            if self.training_enabled and self.prev_obs is not None and self.last_action is not None:
                try:
                    buffer_size = len(self.training_buffer.observations)

                    # Check if we need to reset buffer due to memory limit
                    if buffer_size >= self.memory_limit:
                        self.training_stats['memory_resets'] += 1
                        self.get_logger().info(f"Buffer reached memory limit ({buffer_size}/{self.memory_limit}), resetting for memory management - Reset #{self.training_stats['memory_resets']}")
                        self.training_buffer.reset()
                        buffer_size = 0

                    # Only add to buffer if there's space
                    if buffer_size < self.training_buffer.buffer_size:
                        with th.no_grad():
                            # Add transition to rollout buffer using prev_obs (s_t-1)
                            self.training_buffer.add(
                                obs=self.prev_obs.reshape(1, -1),
                                action=self.last_action.reshape(1, -1),
                                reward=np.array([self.last_reward or 0.0]),
                                episode_start=self.episode_start,
                                value=self.last_values,
                                log_prob=self.last_log_probs
                            )
                except Exception as e:
                    self.get_logger().error(f"Error adding to training buffer: {e}")

            # Store current values for next iteration (shift current -> previous)
            self.prev_obs = self.current_obs  # Current becomes previous for next step
            self.last_action = actions_np
            self.last_values = values
            self.last_log_probs = log_probs

            # Add to rollout data if collection is enabled
            if self.rollout_collection_enabled:
                self._append_rollout_step(flat_state, actions_np)

            # Publish action to environment
            action_flat = DataConverter.ppo_output_to_actions(actions_np, self.num_agents)
            action_msg = Float32MultiArray(data=action_flat.tolist())
            self.action_pub.publish(action_msg)

            self.get_logger().info(f"Published PPO action: {actions_np.tolist()}")

        except Exception as e:
            self.get_logger().error(f"Error in _predict_and_publish: {str(e)}")

    def _append_transition(self, next_state: np.ndarray):
        """Append a (state, action, reward, next_state) transition to the rollout buffer.
        Uses defaults if some fields are not yet available (e.g., reward)."""
        if self.last_state is None or self.last_action is None:
            return  # Need at least previous state and action to form a transition

        reward = float(self.last_reward) if self.last_reward is not None else 0.0

        # Format the state and next_state arrays into structured dictionaries
        structured_state = self._format_state_for_logging(self.last_state)
        structured_next_state = self._format_state_for_logging(next_state)

        # Format actions into a structured dictionary
        structured_action = self._format_action_for_logging(self.last_action)

        transition = {
            "episode": int(self.episode_id),
            "step": int(self.step_idx),
            "state": structured_state,
            "action": structured_action,
            "reward": reward,
            "next_state": structured_next_state,
        }
        self.rollout_data.append(transition)
        self.step_idx += 1

        # Optional periodic save
        if self.rollout_save_every and (self.step_idx % self.rollout_save_every == 0):
            self._save_rollout(final=False)

    def _format_state_for_logging(self, state_array):
        """Convert flat state array to structured dictionary."""
        try:
            # Calculate indices for different parts of the state
            agent_state_size = 6  # [x, y, yaw, vx, vy, vyaw]
            agent_desired_pos_size = 2  # [desired_x, desired_y]

            # Start and end indices
            agents_end = self.num_agents * agent_state_size
            desired_pos_end = agents_end + (self.num_agents * agent_desired_pos_size)

            # Format agent states
            agents = []
            for i in range(self.num_agents):
                start_idx = i * agent_state_size
                agent_data = {
                    "id": i,
                    "position": {
                        "x": float(state_array[start_idx]),
                        "y": float(state_array[start_idx + 1]),
                        "yaw": float(state_array[start_idx + 2])
                    },
                    "velocity": {
                        "vx": float(state_array[start_idx + 3]),
                        "vy": float(state_array[start_idx + 4]),
                        "vyaw": float(state_array[start_idx + 5])
                    }
                }

                # Add desired position
                desired_start = agents_end + (i * agent_desired_pos_size)
                agent_data["desired_position"] = {
                    "x": float(state_array[desired_start]),
                    "y": float(state_array[desired_start + 1])
                }
                agents.append(agent_data)

            # Format formation data
            formation_data = {
                "centroid": {
                    "x": float(state_array[desired_pos_end]),
                    "y": float(state_array[desired_pos_end + 1])
                },
                "error": float(state_array[desired_pos_end + 2]),
                "cross_track_error": float(state_array[desired_pos_end + 3]),
                "virtual_leader": {
                    "x": float(state_array[desired_pos_end + 4]),
                    "y": float(state_array[desired_pos_end + 5])
                }
            }

            return {
                "agents": agents,
                "formation": formation_data,
                "raw": state_array.tolist()  # Keep raw data for backward compatibility
            }
        except Exception as e:
            self.get_logger().error(f"Error formatting state for logging: {e}")
            # Return raw array if formatting fails
            return state_array.tolist()

    def _format_action_for_logging(self, action_array):
        """Convert flat action array to structured dictionary."""
        try:
            actions = []
            # The issue is that action_array has extra dimensions - flatten it
            action_array = action_array.flatten()

            for i in range(self.num_agents):
                start_idx = i * 2
                actions.append({
                    "agent_id": i,
                    "vyaw_rate": float(action_array[start_idx]),
                    "forward_acceleration": float(action_array[start_idx + 1])
                })
            return {
                "actions": actions,
                "raw": action_array.tolist()  # Keep raw data for backward compatibility
            }
        except Exception as e:
            self.get_logger().error(f"Error formatting action for logging: {e}")
            # Return raw array if formatting fails
            return action_array.tolist()

    def _save_rollout(self, final: bool):
        """Update the rollout file with current episode data."""
        if not self.rollout_data:
            return

        try:
            # Load existing file content
            with open(self.rollout_filename) as f:
                data = json.load(f)

            # Find or create episode entry
            if len(data["episodes"]) <= self.episode_id:
                # Add new episode
                episode_data = {
                    "episode_id": self.episode_id,
                    "start_time": time.time(),
                    "start_time_formatted": time.strftime("%H:%M:%S"),
                    "is_complete": final,
                    "transitions": self.rollout_data
                }
                data["episodes"].append(episode_data)
            else:
                # Update existing episode
                data["episodes"][self.episode_id]["transitions"] = self.rollout_data
                data["episodes"][self.episode_id]["is_complete"] = final

            # Write back to file
            with open(self.rollout_filename, 'w') as f:
                json.dump(data, f, indent=2)

            status = "complete" if final else f"in progress ({len(self.rollout_data)} steps)"
            self.get_logger().info(f"Updated rollout file with episode {self.episode_id} - {status}")
        except Exception as e:
            self.get_logger().error(f"Failed to update rollout file: {e}")

    def _finalize_episode(self):
        """Save and reset buffers for the next episode."""
        self._save_rollout(final=True)
        # Reset episodic buffers
        self.rollout_data = []
        self.last_state = None
        self.last_action = None
        self.last_reward = None
        self.step_idx = 0
        self.episode_id += 1
        self.get_logger().info(f"Starting episode {self.episode_id}")

    def collect_rollouts(self, rollout_dir: str | None = None, save_every_steps: int | None = None):
        """Configure where and how often to save rollouts collected from ROS topics.

        This adapts SB3's `collect_rollouts` to our ROS-driven, callback-based flow.
        It does not block; callbacks fill the buffer and saves occur periodically and on episode end.

        Args:
            rollout_dir: Directory to write JSON files; defaults to the `rollout_dir` parameter.
            save_every_steps: Save partial files every N steps; 0/None disables periodic saves.
        """
        if rollout_dir is not None:
            self.rollout_dir = os.path.expanduser(rollout_dir)
            os.makedirs(self.rollout_dir, exist_ok=True)
        if save_every_steps is not None:
            self.rollout_save_every = int(save_every_steps)
        self.get_logger().info(
            f"Rollout collection configured: dir={self.rollout_dir}, save_every={self.rollout_save_every}"
        )

    def reward_callback(self, msg):
        self.last_reward = msg.data
        self.current_episode_reward += msg.data

    def done_callback(self, msg):
        """Handle episode completion for training and rollout collection."""
        if msg.data:
            self.episode_rewards.append(self.current_episode_reward)
            self.get_logger().info(f"Episode {self.episode_id} finished with total reward: {self.current_episode_reward:.4f}")
            self.current_episode_reward = 0.0

            if self.training_enabled:
                # Set done flag for current step
                self.dones = np.ones(1, dtype=bool)

                # Add final transition with terminal state (current_obs is the terminal state)
                if self.current_obs is not None:
                    with th.no_grad():
                        obs_tensor = DataConverter.numpy_to_tensor(self.current_obs)
                        obs_tensor = obs_tensor.reshape(1, -1)
                        _, values, _ = self.model.policy.forward(obs_tensor)

                        # Add final transition to buffer
                        self.training_buffer.add(
                            obs=self.current_obs.reshape(1, -1),
                            action=self.last_action.reshape(1, -1) if self.last_action is not None
                                else np.zeros((1, self.action_space.shape[0])),
                            reward=np.array([self.last_reward or 0.0]),
                            episode_start=self.episode_start,  # Use episode_start, not dones
                            value=values,
                            log_prob=self.last_log_probs if self.last_log_probs is not None
                                else th.zeros(self.action_space.shape[0], device=self.model.device)
                        )

                # Set episode_start flag for the NEXT step
                self.episode_start = np.ones(1, dtype=bool)

                # Reset done flag for next step
                self.dones = np.zeros(1, dtype=bool)

            # Finalize episode for rollout collection
            self._finalize_episode()
            self.get_logger().info(f'Episode {self.episode_id-1} finished')

            if self.training_enabled and self.episode_id > 0 and self.episode_id % 3 == 0:
                # Try to train every 3 episodes
                self.train_model()

            # Limit number of episodes if needed
            if self.episode_id >= self.max_episodes:
                self.get_logger().info(f"Reached maximum episodes ({self.max_episodes}). Saving final model.")
                model_path = os.path.join(self.rollout_dir, "ppo_model_final.zip")
                self.model.save(model_path)
                # Could add code to shut down gracefully here

    def _initialize_rollout_file(self):
        """Create the rollout file with metadata section."""
        metadata = {
            "session_id": self.session_id,
            "start_time": self.session_start_time.to_msg().sec,
            "start_time_formatted": time.strftime(
                "%Y-%m-%d %H:%M:%S",
                time.localtime(self.session_start_time.to_msg().sec)
            ),
            "num_agents": self.num_agents,
            "model_path": self.model_path if self.model_path else "new_model",
            "episodes": []
        }

        try:
            with open(self.rollout_filename, 'w') as f:
                json.dump(metadata, f, indent=2)
            self.get_logger().info(f"Initialized rollout file: {self.rollout_filename}")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize rollout file: {e}")

    def train_model(self):
        """Train the PPO model on collected transitions with progressive approach."""
        if not self.training_enabled:
            return

        # Check how many valid transitions we have
        buffer_size = len(self.training_buffer.observations)

        # Log buffer status with memory information
        memory_usage_kb = (buffer_size * 76) / 1024  # Approximate memory usage
        self.get_logger().info(f"Buffer status: {buffer_size}/{self.buffer_size} transitions ({memory_usage_kb:.1f} KB)")

        # Progressive training: train when we have minimum viable batch (not when full)
        if buffer_size < self.min_training_size:
            self.get_logger().info(f"Not enough data for training: {buffer_size}/{self.min_training_size} minimum required")
            return

        self.get_logger().info(f"Training PPO model on {buffer_size} transitions (progressive training)")

        try:
            # Compute returns and advantages
            last_values = th.zeros(1, device=self.model.device)
            self.training_buffer.compute_returns_and_advantage(last_values=last_values, dones=self.dones)

            # Set training mode
            self.model.policy.set_training_mode(True)

            # Learning rate schedule
            progress_remaining = max(0.0, 1.0 - (self.episode_id / 1000.0))

            if hasattr(self.model.lr_schedule, "__call__"):
                current_lr = self.model.lr_schedule(progress_remaining)
            else:
                current_lr = self.model.learning_rate

            # Update optimizer learning rate
            for param_group in self.model.policy.optimizer.param_groups:
                param_group["lr"] = current_lr

            # Train for multiple epochs
            clip_range = self.model.clip_range(progress_remaining)
            clip_range_vf = self.model.clip_range_vf(progress_remaining) if self.model.clip_range_vf is not None else None

            for epoch in range(self.n_epochs):
                approx_kl_divs = []

                # Process minibatches
                for rollout_data in self.training_buffer.get(self.batch_size):
                    actions = rollout_data.actions

                    # Evaluate actions
                    values, log_probs, entropy = self.model.policy.evaluate_actions(
                        rollout_data.observations, actions
                    )
                    values = values.flatten()

                    # Normalize advantage
                    advantages = rollout_data.advantages
                    if self.model.normalize_advantage and len(advantages) > 1:
                        advantages = (advantages - advantages.mean()) / (advantages.std() + 1e-8)

                    # PPO loss
                    ratio = th.exp(log_probs - rollout_data.old_log_prob)
                    policy_loss_1 = advantages * ratio
                    policy_loss_2 = advantages * th.clamp(ratio, 1.0 - clip_range, 1.0 + clip_range)
                    policy_loss = -th.min(policy_loss_1, policy_loss_2).mean()

                    # Value loss
                    if clip_range_vf is None:
                        values_pred = values
                    else:
                        values_pred = rollout_data.old_values + th.clamp(
                            values - rollout_data.old_values, -clip_range_vf, clip_range_vf
                        )
                    value_loss = th.nn.functional.mse_loss(rollout_data.returns, values_pred)

                    # Entropy loss
                    if entropy is None:
                        entropy_loss = -th.mean(-log_probs)
                    else:
                        entropy_loss = -th.mean(entropy)

                    # Total loss
                    loss = policy_loss + self.model.ent_coef * entropy_loss + self.model.vf_coef * value_loss

                    # Gradient step
                    self.model.policy.optimizer.zero_grad()
                    loss.backward()
                    # Clip grad norm
                    th.nn.utils.clip_grad_norm_(self.model.policy.parameters(), self.model.max_grad_norm)
                    self.model.policy.optimizer.step()

                    # Log statistics
                    with th.no_grad():
                        log_ratio = log_probs - rollout_data.old_log_prob
                        approx_kl_div = th.mean((th.exp(log_ratio) - 1) - log_ratio).cpu().numpy()
                        approx_kl_divs.append(approx_kl_div)

                mean_kl = np.mean(approx_kl_divs)
                self.get_logger().info(
                    f"Epoch {epoch+1}/{self.n_epochs}, approx_kl={mean_kl:.6f}, lr={current_lr:.6f}"
                )

                # Early stopping
                if self.model.target_kl is not None and mean_kl > 1.5 * self.model.target_kl:
                    self.get_logger().info(f"Early stopping at epoch {epoch+1} due to reaching max KL: {mean_kl:.6f}")
                    break

            # Update training statistics
            self.training_stats['total_training_sessions'] += 1
            self.training_stats['total_transitions_trained'] += buffer_size
            self.training_stats['average_buffer_size_at_training'] = (
                self.training_stats['total_transitions_trained'] /
                self.training_stats['total_training_sessions']
            )

            # Log training statistics
            self.get_logger().info(
                f"Training complete. Sessions: {self.training_stats['total_training_sessions']}, "
                f"Total transitions: {self.training_stats['total_transitions_trained']}, "
                f"Avg buffer size: {self.training_stats['average_buffer_size_at_training']:.1f}"
            )

            # Save model after training
            model_path = os.path.join(self.rollout_dir, f"ppo_model_ep{self.episode_id}.zip")
            self.model.save(model_path)
            self.get_logger().info(f"Saved trained model to {model_path}")

            # Also save as latest_model.zip for easy discovery
            latest_model_path = os.path.join(self.rollout_dir, "latest_model.zip")
            self.model.save(latest_model_path)
            self.get_logger().info(f"Saved latest model to {latest_model_path}")

            # Smart buffer management: reset strategically
            current_buffer_size = len(self.training_buffer.observations)
            if current_buffer_size >= self.memory_limit * 0.8:  # Reset when 80% of memory limit
                self.training_stats['memory_resets'] += 1
                self.get_logger().info(f"Resetting buffer for memory management ({current_buffer_size}/{self.memory_limit}) - Reset #{self.training_stats['memory_resets']}")
                self.training_buffer.reset()
            else:
                self.get_logger().info(f"Keeping buffer for continuous learning ({current_buffer_size}/{self.memory_limit})")
        except Exception as e:
            self.get_logger().error(f"Error during training: {e}")

    def _append_rollout_step(self, state: np.ndarray, action: np.ndarray):
        """Add the current state and action to the rollout data.
        
        This method is called from _predict_and_publish to collect state-action 
        pairs for later analysis.
        """
        # Store the current state for later use
        if self.last_state is not None:
            # If we have a previous state, add a full transition
            self._append_transition(state)

        # Update for next time
        self.last_state = state.copy()
        self.last_action = action.copy()

    def reset_environment(self):
        """Request environment reset when agents are out of bounds."""
        if not self.reset_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Reset service not available, continuing without reset')
            return False

        request = Trigger.Request()
        future = self.reset_client.call_async(request)

        # Setup callback for when reset is complete
        future.add_done_callback(self._reset_done_callback)
        return True

    def _reset_done_callback(self, future):
        """Handle completion of reset service call."""
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('Environment reset successful')
                # Reset internal state variables
                self.last_state = None
                self.last_action = None
                self.last_reward = None
                self.episode_start = np.ones(1, dtype=bool)
            else:
                self.get_logger().warn(f'Environment reset failed: {response.message}')
        except Exception as e:
            self.get_logger().error(f'Error in reset callback: {e}')

    def _agents_out_of_bounds(self, flat_state: np.ndarray) -> bool:
        """Check if any agent is out of bounds."""
        # Define boundaries - updated to match our new coordinate system
        MIN_X, MAX_X = -15.0, 35.0  # Wider than our clipping bounds
        MIN_Y, MAX_Y = -15.0, 35.0  # Wider than our clipping bounds

        for i in range(self.num_agents):
            # Extract agent position (x, y) - uses parameterized dimensions
            agent_x = flat_state[i * self.obs_dim_per_agent]
            agent_y = flat_state[i * self.obs_dim_per_agent + 1]

            # Check if out of bounds
            if (agent_x < MIN_X or agent_x > MAX_X or
                agent_y < MIN_Y or agent_y > MAX_Y):
                return True

    def check_and_reset(self):
        """Periodically check if agents are stuck and need reset."""
        if not hasattr(self, 'last_position'):
            self.last_position = None
            return

        if self.last_position is not None and self.current_obs is not None:
            # Check if agents haven't moved (using parameterized dimensions)
            # Only check position data for all agents (first obs_dim_per_agent * num_agents elements)
            obs_size = self.obs_dim_per_agent * self.num_agents
            if np.allclose(self.last_position, self.current_obs[:obs_size], atol=0.1):
                self.get_logger().warn("Agents appear stuck. Requesting environment reset")
                self.reset_environment()

        if self.current_obs is not None:
            # Store only the position data for comparison
            obs_size = self.obs_dim_per_agent * self.num_agents
            self.last_position = self.current_obs[:obs_size].copy()

    def _find_and_load_best_model(self):
        """Smart model loading: try multiple sources in order of preference.
        
        Returns the path of the loaded model, or None if no model was loaded.
        """
        model_candidates = []

        # 1. First priority: explicit model_path parameter
        if self.model_path and os.path.exists(self.model_path):
            model_candidates.append((self.model_path, "explicit parameter"))

        # 2. Second priority: latest_model.zip in rollout directory
        latest_model_path = os.path.join(self.rollout_dir, "latest_model.zip")
        if os.path.exists(latest_model_path):
            model_candidates.append((latest_model_path, "latest model"))

        # 3. Third priority: latest episode model in rollout directory
        if os.path.exists(self.rollout_dir):
            # Look for episode-specific models (ppo_model_ep*.zip)
            episode_models = []
            for filename in os.listdir(self.rollout_dir):
                if filename.startswith('ppo_model_ep') and filename.endswith('.zip'):
                    try:
                        # Extract episode number from filename
                        episode_num = int(filename.split('ep')[1].split('.')[0])
                        full_path = os.path.join(self.rollout_dir, filename)
                        episode_models.append((episode_num, full_path))
                    except (ValueError, IndexError):
                        continue

            # Sort by episode number (latest first)
            if episode_models:
                episode_models.sort(key=lambda x: x[0], reverse=True)
                latest_episode_model = episode_models[0][1]
                model_candidates.append((latest_episode_model, f"latest episode model (ep {episode_models[0][0]})"))

            # 4. Fourth priority: final model
            final_model_path = os.path.join(self.rollout_dir, "ppo_model_final.zip")
            if os.path.exists(final_model_path):
                model_candidates.append((final_model_path, "final model"))

        # 5. Fifth priority: look in common locations
        common_paths = [
            os.path.expanduser("~/Desktop/ASV_Rollouts/latest_model.zip"),
            os.path.expanduser("~/Desktop/ASV_Rollouts/ppo_model_final.zip"),
            "./latest_model.zip",
            "./ppo_model.zip",
            "./models/latest_model.zip",
            "./models/ppo_model.zip"
        ]

        for path in common_paths:
            if os.path.exists(path):
                model_candidates.append((path, f"common location: {path}"))

        # Try to load the first available model
        for model_path, description in model_candidates:
            try:
                self.get_logger().info(f"Attempting to load model from {description}: {model_path}")
                # Load model without environment - we'll validate spaces separately
                self.model = PPO.load(model_path, env=None)

                # Verify the model loaded correctly and spaces match
                if hasattr(self.model, 'policy') and self.model.policy is not None:
                    # Validate that loaded model spaces match our expected spaces
                    if (self.model.observation_space.shape == self.observation_space.shape and
                        self.model.action_space.shape == self.action_space.shape):
                        self.get_logger().info(f"✓ Successfully loaded and verified model from {description}")
                        return model_path
                    else:
                        self.get_logger().warn(f"✗ Model spaces don't match: obs={self.model.observation_space.shape} vs {self.observation_space.shape}, action={self.model.action_space.shape} vs {self.action_space.shape}")
                else:
                    self.get_logger().warn(f"✗ Model loaded but appears invalid from {description}")

            except Exception as e:
                self.get_logger().warn(f"✗ Failed to load model from {description}: {str(e)}")
                continue

        # No model could be loaded
        return None

    def get_training_insights(self):
        """Get current training performance insights."""
        if not self.training_enabled:
            return "Training disabled"

        buffer_size = len(self.training_buffer.observations) if hasattr(self, 'training_buffer') else 0
        memory_usage_kb = (buffer_size * 76) / 1024

        insights = {
            'buffer_status': f"{buffer_size}/{self.buffer_size if hasattr(self, 'buffer_size') else 100}",
            'memory_usage_kb': f"{memory_usage_kb:.1f} KB",
            'training_sessions': self.training_stats['total_training_sessions'],
            'total_transitions': self.training_stats['total_transitions_trained'],
            'memory_resets': self.training_stats['memory_resets'],
            'ready_for_training': buffer_size >= (self.min_training_size if hasattr(self, 'min_training_size') else 50)
        }

        return insights

def main(args=None):
    rclpy.init(args=args)
    node = ASVPPONode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
