#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Float32, Bool
import numpy as np

class ASVEnvNode(Node):
    def __init__(self):
        super().__init__('asv_env_node')
        
        self.declare_parameter('num_agents', 2)
        self.num_agents = self.get_parameter('num_agents').value
        
        self.agent_states = [None] * self.num_agents
        self.received_updates_this_step = [False] * self.num_agents
        self.loop_started = False

        # Publishers
        self.state_pub = self.create_publisher(Float32MultiArray, '/environment/state', 10)
        self.reward_pub = self.create_publisher(Float32, '/environment/reward', 10)
        self.done_pub = self.create_publisher(Bool, '/environment/done', 10)
        
        self.agent_action_pubs = []
        self.agent_reset_pubs = []
        for i in range(self.num_agents):
            self.agent_action_pubs.append(self.create_publisher(Float32MultiArray, f'/agent_{i}/action', 10))
            self.agent_reset_pubs.append(self.create_publisher(Float32MultiArray, f'/agent_{i}/reset', 10))

        # Subscribers
        self.action_sub = self.create_subscription(
            Float32MultiArray, '/ppo/action', self.action_callback, 10
        )
        self.agent_state_subs = [
            self.create_subscription(
                Float32MultiArray,
                f'/agent_{i}/state_update',
                self.create_state_callback(i),
                10,
            )
            for i in range(self.num_agents)
        ]

        # This timer will periodically send reset signals until the loop starts
        self.reset_timer = self.create_timer(0.5, self.trigger_reset)
        # After loop start, keep publishing environment state at a low rate until first action arrives
        self._awaiting_first_action = True
        self._keepalive_timer = None
        self.get_logger().info('ASV Environment Coordinator started. Waiting for all agents to reset...')

    def create_state_callback(self, agent_id):
        def state_callback(msg):
            # This callback is triggered when an agent sends a state update
            if self.agent_states[agent_id] is None:
                self.get_logger().info(f"Received first state from agent {agent_id}.")
            self.agent_states[agent_id] = np.array(msg.data)
            self.received_updates_this_step[agent_id] = True

            # Start the loop once all agents responded initially
            if not self.loop_started and all(self.received_updates_this_step):
                self.get_logger().info('All agents have responded. Starting RL loop.')
                self.loop_started = True
                self.reset_timer.cancel()
                self.initial_state_timer = self.create_timer(0.5, self.initial_state_publish_callback)
                # start keepalive publisher until we get the first action from PPO
                if self._keepalive_timer is None:
                    self._keepalive_timer = self.create_timer(0.5, self._keepalive_publish)
                return

            # During the loop: once all agents have updated, publish env state
            if self.loop_started and all(self.received_updates_this_step):
                self.get_logger().info('All agents updated this step. Publishing next observation.', throttle_duration_sec=1)
                self.publish_environment_state()
                # Reset per-step flags to wait for the next round of agent updates
                self.received_updates_this_step = [False] * self.num_agents

        return state_callback

    def initial_state_publish_callback(self):
        """
        This is a one-shot callback that publishes the initial state and then
        cancels the timer so it doesn't run again.
        """
        self.get_logger().info("One-shot timer triggered: publishing initial state.")
        self.publish_environment_state()
        self.initial_state_timer.cancel()

    def action_callback(self, msg):
        if not self.loop_started:
            self.get_logger().warn('Action received before RL loop started. Ignoring.')
            return

        self.get_logger().info(f'Received actions from PPO: {list(msg.data)}. Distributing to agents.', throttle_duration_sec=1)
        self.received_updates_this_step = [False] * self.num_agents
        # we got the first action, stop keepalive
        if self._keepalive_timer is not None:
            self._keepalive_timer.cancel()
            self._keepalive_timer = None
            self._awaiting_first_action = False
        
        actions = np.array(msg.data).reshape((self.num_agents, -1))
        for i in range(self.num_agents):
            self.agent_action_pubs[i].publish(Float32MultiArray(data=actions[i].tolist()))
        self.get_logger().info('Actions dispatched to all agents; waiting for their state updates.', throttle_duration_sec=1)

    def publish_environment_state(self):
        # Only publish if all agents have reported at least once
        if any(s is None for s in self.agent_states):
            self.get_logger().warn('Attempted to publish state, but not all agents have reported.')
            return

        global_state = np.array(self.agent_states).flatten()
        state_msg = Float32MultiArray(data=global_state.tolist())
        self.state_pub.publish(state_msg)
        self.get_logger().info(f"Published environment state: {state_msg.data}", throttle_duration_sec=1)
        
        reward = self.calculate_reward()
        done = self.check_done()
        
        self.reward_pub.publish(Float32(data=float(reward)))
        self.done_pub.publish(Bool(data=bool(done)))
        
        if done:
            self.get_logger().info('Episode finished. Restarting reset timer.')
            self.loop_started = False
            self._awaiting_first_action = True
            if self._keepalive_timer is not None:
                self._keepalive_timer.cancel()
                self._keepalive_timer = None
            self.reset_timer.reset()
            self.trigger_reset()

    def _keepalive_publish(self):
        """Publish environment state regularly while waiting for first PPO action.
        Helps subscriber pipelines and RViz TF listeners start up deterministically."""
        if self.loop_started and self._awaiting_first_action:
            self.get_logger().info('Keepalive: publishing environment state while awaiting first action.', throttle_duration_sec=1)
            self.publish_environment_state()

    def trigger_reset(self):
        if not self.loop_started:
            self.get_logger().info('Broadcasting reset signals...')
        
        self.received_updates_this_step = [False] * self.num_agents
        for i in range(self.num_agents):
            # Initial state: [x, y, heading, speed, turn_rate, unused]
            start_x = i * 20.0 - (self.num_agents - 1) * 10.0
            initial_state = [start_x, 0.0, np.pi / 2, 1.0, 0.0, 0.0]
            self.agent_reset_pubs[i].publish(Float32MultiArray(data=initial_state))

    def calculate_reward(self):
        # Placeholder reward logic
        total_reward = 0.0
        if all(s is not None for s in self.agent_states):
            total_reward = sum(state[3] for state in self.agent_states) # Reward for forward speed
            if self.check_collision():
                total_reward -= 100
        return total_reward

    def check_done(self):
        return self.check_collision()

    def check_collision(self):
        if self.num_agents < 2 or any(s is None for s in self.agent_states):
            return False
        pos1 = self.agent_states[0][:2]
        pos2 = self.agent_states[1][:2]
        # Check if distance between boats is less than a threshold (e.g., 5 meters)
        return np.linalg.norm(pos1 - pos2) < 5.0

def main(args=None):
    rclpy.init(args=args)
    node = ASVEnvNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()