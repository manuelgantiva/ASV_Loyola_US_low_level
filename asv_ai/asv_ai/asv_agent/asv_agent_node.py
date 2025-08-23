#!/usr/bin/env python3
from ..utils.data_conversion import DataConverter
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import numpy as np
import tf2_ros
from geometry_msgs.msg import TransformStamped

class ASVAgentNode(Node):
    def __init__(self):
        super().__init__('asv_agent_node')
        
        self.declare_parameter('agent_id', 0)
        self.agent_id = self.get_parameter('agent_id').value
        
        # State: [x, y, yaw, vx, vy, vyaw]
        self.state = np.zeros(6)
        self.dt = 0.1  # Simulation time step

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
            self.get_logger().info(f'Agent {self.agent_id} reset to {self.state}')
            self.publish_state_and_tf()

    def action_callback(self, msg):
        """Applies an action, updates physics, and publishes the new state."""
        # Convert ROS message to NumPy array
        action = DataConverter.ros_to_numpy(msg)
        self.get_logger().info(f'Agent {self.agent_id} received action: {action.tolist()}', throttle_duration_sec=1)
        
        # Apply scaled actions
        self.state[5] = DataConverter.scale_action_to_physical(action[0], "vyaw")  # vyaw
        self.state[3] += DataConverter.scale_action_to_physical(action[1], "acceleration")  # vx
        self.state[3] = np.clip(self.state[3], 0.0, 5.0)  # Clamp speed
    
        # Update physics
        x, y, yaw, vx, vy, vyaw = self.state
        
        # Correctly update yaw and position
        new_yaw = yaw + vyaw * self.dt
        new_x = x + (vx * np.cos(new_yaw) - vy * np.sin(new_yaw)) * self.dt
        new_y = y + (vx * np.sin(new_yaw) + vy * np.cos(new_yaw)) * self.dt
        
        # The state update was incorrect. It should use the new values.
        self.state = np.array([new_x, new_y, new_yaw, self.state[3], self.state[4], self.state[5]])
        
        self.publish_state_and_tf()

    def publish_state_and_tf(self):
        """Publishes the current state and broadcasts the TF."""
        # Publish state
        # Convert NumPy array to ROS message
        state_msg = DataConverter.numpy_to_ros(self.state)
        self.state_pub.publish(state_msg)        
        self.get_logger().info(f'Agent {self.agent_id} published state: {self.state.tolist()}', throttle_duration_sec=1)
        
        # Publish TF for RViz
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

def main(args=None):
    rclpy.init(args=args)
    node = ASVAgentNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()