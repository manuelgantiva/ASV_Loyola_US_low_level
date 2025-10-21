#!/usr/bin/env python3

import numpy as np
import rclpy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

from ..utils.data_conversion import DataConverter
from .asv_agent import ASVAgent


class ASVAgentNode(Node):
    def __init__(self):
        super().__init__('asv_agent_node')

        self.declare_parameter('agent_id', 0)
        self.agent_id = self.get_parameter('agent_id').value

        # Initialize physical ASV model with proper dt
        initial_state = np.array([[2.0], [2.0], [0.0], [0.0], [0.0], [0.0]], dtype=np.float32)
        self.asv_model = ASVAgent(id=self.agent_id, x_ini=initial_state, dt=0.025)  # dt/4 for sub-steps

        # State for ROS compatibility: [x, y, yaw, vx, vy, vyaw]
        self.state = np.array([2.0, 2.0, 0.0, 0.0, 0.0, 0.0])
        self.dt = 0.1  # ROS publishing time step

        # TF publishing rate limiting for performance
        self.last_tf_publish_time = 0.0
        self.tf_publish_interval = 0.05  # 20Hz max TF rate (instead of unlimited)

        # State publishing optimization (but allow frequent updates for responsiveness)
        self._last_state_publish = 0.0
        self._state_publish_interval = 0.02  # 50Hz state updates for responsiveness

        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Publisher for this agent's state
        self.state_pub = self.create_publisher(Float32MultiArray, f'/agent_{self.agent_id}/state_update', 10)

        # Subscriber for this agent's action
        self.action_sub = self.create_subscription(Float32MultiArray, f'/agent_{self.agent_id}/action', self.action_callback, 10)

        # Subscriber to this agent's specific reset signal
        self.reset_sub = self.create_subscription(Float32MultiArray, f'/agent_{self.agent_id}/reset', self.reset_callback, 10)

        self.get_logger().info(f'ASV Agent Node {self.agent_id} started and waiting for reset.')

    def reset_callback(self, msg):
        """Resets the agent to an initial state provided by the environment."""
        initial_state = np.array(msg.data)
        if initial_state.size == 6:
            self.state = initial_state
            # Convert ROS state to ASV model format and reset physical model
            asv_state = self._convert_ros_to_asv_state(initial_state)
            self.asv_model.x = asv_state
            self.get_logger().info(f'Agent {self.agent_id} reset to {self.state}')
            self.publish_state_and_tf()

    def action_callback(self, msg):
        """Applies an action using physical model, updates physics, and publishes the new state."""
        # Convert ROS message to NumPy array
        action = DataConverter.ros_to_numpy(msg)

        # Throttled logging for performance
        self.get_logger().info(f'Agent {self.agent_id} received action: {action.tolist()}', throttle_duration_sec=5.0)

        # Convert normalized action to physical action format expected by ASVAgent
        physical_action = self._convert_action_to_physical(action)

        # Use physical model to evolve the system
        self.asv_model.evolve(physical_action)

        # Convert ASV model state back to ROS format
        self.state = self._convert_asv_to_ros_state(self.asv_model.x)

        # Publish immediately for responsive visualization
        self.publish_state_and_tf()

    def publish_state_and_tf(self):
        """Publishes the current state and broadcasts the TF with optimized timing."""
        current_time = self.get_clock().now().nanoseconds / 1e9

        # Always publish state immediately for responsive visualization
        state_msg = DataConverter.numpy_to_ros(self.state)
        self.state_pub.publish(state_msg)
        self.get_logger().info(f'Agent {self.agent_id} published state: {self.state.tolist()}', throttle_duration_sec=5.0)

        # Rate-limited TF publishing for performance (but still responsive)
        if current_time - self.last_tf_publish_time >= self.tf_publish_interval:
            self.last_tf_publish_time = current_time
            self._publish_tf()

    def _publish_tf(self):
        """Publishes TF transform at controlled rate"""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = f'ASV{self.agent_id}/base_link'

        # Explicitly cast numpy floats to python floats
        t.transform.translation.x = float(self.state[0])
        t.transform.translation.y = float(self.state[1])
        t.transform.translation.z = 0.0

        q = self.get_quaternion_from_euler(0, 0, self.state[2])
        t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = q
        self.tf_broadcaster.sendTransform(t)

    def get_quaternion_from_euler(self, roll, pitch, yaw):
        cy, sy = np.cos(yaw * 0.5), np.sin(yaw * 0.5)
        cp, sp = np.cos(pitch * 0.5), np.sin(pitch * 0.5)
        cr, sr = np.cos(roll * 0.5), np.sin(roll * 0.5)
        return [sr*cp*cy-cr*sp*sy, cr*sp*cy+sr*cp*sy, cr*cp*sy-sr*sp*cy, cr*cp*cy+sr*sp*sy]

    def _convert_ros_to_asv_state(self, ros_state):
        """Convert ROS state [x, y, yaw, vx, vy, vyaw] to ASV state [x, y, psi, u, v, r]"""
        x, y, yaw, vx, vy, vyaw = ros_state

        # Convert global velocities to body-frame velocities
        cos_yaw = np.cos(yaw)
        sin_yaw = np.sin(yaw)
        u = vx * cos_yaw + vy * sin_yaw  # surge (forward velocity in body frame)
        v = -vx * sin_yaw + vy * cos_yaw  # sway (lateral velocity in body frame)
        r = vyaw  # yaw rate (same in both frames)

        return np.array([[x], [y], [yaw], [u], [v], [r]], dtype=np.float32)

    def _convert_asv_to_ros_state(self, asv_state):
        """Convert ASV state [x, y, psi, u, v, r] to ROS state [x, y, yaw, vx, vy, vyaw]"""
        x, y, psi, u, v, r = asv_state.flatten()

        # Convert body-frame velocities to global velocities
        cos_psi = np.cos(psi)
        sin_psi = np.sin(psi)
        vx = u * cos_psi - v * sin_psi  # global x velocity
        vy = u * sin_psi + v * cos_psi  # global y velocity
        vyaw = r  # yaw rate (same in both frames)

        return np.array([x, y, psi, vx, vy, vyaw])

    def _convert_action_to_physical(self, normalized_action):
        """Convert normalized action [-1,1] to physical forces for ASVAgent"""
        # normalized_action[0] -> yaw moment, normalized_action[1] -> surge force
        # Scale to match the ranges expected by ASVAgent.get_force_tau()
        surge_force = normalized_action[1] * 2.0  # Scale to [0, 2] range expected by get_force_tau
        yaw_moment = normalized_action[0] * 0.5 * 2.3  # Scale to [-0.5, 0.5] * 2.3 range

        return np.array([surge_force, yaw_moment])

def main(args=None):
    rclpy.init(args=args)
    node = ASVAgentNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
