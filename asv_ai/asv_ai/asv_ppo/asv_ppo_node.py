#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Float32, Bool
import numpy as np
import os
import json
from stable_baselines3 import PPO
from ..asv_env.asv_env import Environment as SimEnv

class ASVPPONode(Node):
    def __init__(self):
        super().__init__('asv_ppo_node')

        # Parameters
        self.declare_parameter('num_agents', 2)
        self.declare_parameter('model_path', '')
        self.declare_parameter('rollout_dir', os.path.expanduser('~/Desktop/ASV_Rollouts'))
        self.declare_parameter('rollout_save_every', 50)

        # Resolve parameters
        self.num_agents = self.get_parameter('num_agents').value
        self.model_path = self.get_parameter('model_path').value
        self.rollout_dir = os.path.expanduser(self.get_parameter('rollout_dir').value)
        self.rollout_save_every = int(self.get_parameter('rollout_save_every').value)
        os.makedirs(self.rollout_dir, exist_ok=True)

        # Runtime buffers/state
        self.rollout_data = []
        self.last_state = None
        self.last_action = None
        self.last_reward = None
        self.model_ready = False
        self._pending_state = None  # buffer a state if it arrives before model is ready
        self.episode_id = 0
        self.step_idx = 0

        # Publishers and Subscribers FIRST to avoid missing early messages
        self.state_sub = self.create_subscription(Float32MultiArray, '/environment/state', self.state_callback, 10)
        self.reward_sub = self.create_subscription(Float32, '/environment/reward', self.reward_callback, 10)
        self.done_sub = self.create_subscription(Bool, '/environment/done', self.done_callback, 10)
        self.action_pub = self.create_publisher(Float32MultiArray, '/ppo/action', 10)

        # Create a dummy environment to get space information
        dummy_env = SimEnv(num_agents=self.num_agents)

        if self.model_path and os.path.exists(self.model_path):
            self.model = PPO.load(self.model_path, env=dummy_env)
            self.get_logger().info(f'Loaded model from {self.model_path}')
        else:
            self.model = PPO("MlpPolicy", dummy_env, verbose=0)
            self.get_logger().info('No model found, using new PPO model.')

        self.model_ready = True
        # If we received a state while loading, process it now
        if self._pending_state is not None:
            self.get_logger().info('Processing buffered environment state after model init.')
            self._predict_and_publish(self._pending_state)
            self._pending_state = None

        self.get_logger().info(f'ASV PPO Node started with {self.num_agents} agents')

    def state_callback(self, msg):
        try:
            # The observation from the topic is a flat array
            flat_state = np.array(msg.data)
            self.get_logger().info(f'PPO received state with shape {flat_state.shape} and values {flat_state[:12]}...', throttle_duration_sec=1)

            if not self.model_ready:
                # Buffer the latest state until model is ready
                self._pending_state = flat_state
                self.get_logger().info('Model not ready yet; buffering environment state.', throttle_duration_sec=1)
                return

            self._predict_and_publish(flat_state)
        except Exception as e:
            self.get_logger().error(f'Error in state_callback: {str(e)}')

    def _predict_and_publish(self, flat_state: np.ndarray):
        """Internal helper to compute an action from flat state and publish it."""
        # Extract just the agent states from the extended state
        try:
            # The original state has 6 values per agent at the beginning of the array
            agent_states = flat_state[:self.num_agents * 6]
            reshaped_state = agent_states.reshape((self.num_agents, 6)).astype(np.float32)
            
            self.get_logger().info(f"Received state with shape: {flat_state.shape}, using first {self.num_agents * 6} elements for agent states")
        except ValueError as e:
            self.get_logger().error(f"Could not reshape observation: {e}. Received shape: {flat_state.shape}")
            return

        # Store previous transition (s,a,r,s') using the latest observation as next_state
        self._append_transition(next_state=flat_state)

        # Compute and publish action
        try:
            # IMPORTANT: pass (num_agents, 6), not flattened
            action, _ = self.model.predict(reshaped_state, deterministic=True)
        except Exception as e:
            self.get_logger().error(f"Model predict failed: {e}")
            return
    
        # Ensure action shape is (num_agents, 2)
        action = np.array(action)
        if action.ndim == 1:
            if action.size % self.num_agents != 0:
                self.get_logger().error(f"Action size {action.size} not divisible by num_agents {self.num_agents}.")
                return
            action = action.reshape((self.num_agents, -1))
        elif action.ndim == 2:
            if action.shape[0] != self.num_agents:
                self.get_logger().error(f"Action first dim {action.shape[0]} != num_agents {self.num_agents}.")
                return
        else:
            self.get_logger().error(f"Unexpected action shape {action.shape}")
            return

        self.action_pub.publish(Float32MultiArray(data=action.flatten().astype(np.float32).tolist()))
        self.get_logger().info(f'Published PPO action: {action.tolist()}', throttle_duration_sec=1)

        # Cache for next transition
        self.last_state = flat_state
        self.last_action = action.flatten()
        # Note: reward will be filled by reward_callback before next step; defaulted if missing

    def _append_transition(self, next_state: np.ndarray):
        """Append a (state, action, reward, next_state) transition to the rollout buffer.
        Uses defaults if some fields are not yet available (e.g., reward)."""
        if self.last_state is None or self.last_action is None:
            return  # Need at least previous state and action to form a transition

        reward = float(self.last_reward) if self.last_reward is not None else 0.0
        transition = {
            "episode": int(self.episode_id),
            "step": int(self.step_idx),
            "state": self.last_state.astype(float).tolist(),
            "action": self.last_action.astype(float).tolist(),
            "reward": reward,
            "next_state": np.array(next_state, dtype=float).tolist(),
        }
        self.rollout_data.append(transition)
        self.step_idx += 1

        # Optional periodic save
        if self.rollout_save_every and (self.step_idx % self.rollout_save_every == 0):
            self._save_rollout(final=False)

    def _save_rollout(self, final: bool):
        """Save the current rollout buffer to a JSON file in rollout_dir."""
        if not self.rollout_data:
            return
        ts = self.get_clock().now().to_msg()
        stamp = f"{ts.sec}-{ts.nanosec}"
        suffix = "final" if final else "partial"
        filename = os.path.join(self.rollout_dir, f"rollout_ep{self.episode_id}_{suffix}_{stamp}.json")
        try:
            with open(filename, 'w') as f:
                json.dump(self.rollout_data, f, indent=2)
            self.get_logger().info(f"Saved rollout ({suffix}) to {filename}")
        except Exception as e:
            self.get_logger().error(f"Failed to save rollout to {filename}: {e}")

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

    def done_callback(self, msg):
        if msg.data:
            self.get_logger().info(
                f'Episode finished. Saving rollout data with {len(self.rollout_data)} transitions.'
            )
            self._finalize_episode()

def main(args=None):
    rclpy.init(args=args)
    node = ASVPPONode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()