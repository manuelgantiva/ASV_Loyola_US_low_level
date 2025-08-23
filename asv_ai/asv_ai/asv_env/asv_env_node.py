#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Float32, Bool, ColorRGBA
from std_srvs.srv import Trigger
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point, PoseArray, Pose
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import numpy as np
from ..asv_path.asv_path import ParametrizedPath
from ..utils.data_conversion import DataConverter
import math

class ASVEnvNode(Node):
    def __init__(self):
        super().__init__('asv_env_node')
        
        self.declare_parameter('num_agents', 2)
        self.num_agents = self.get_parameter('num_agents').value
        
        self.agent_states = [None] * self.num_agents
        self.received_updates_this_step = [False] * self.num_agents
        self.loop_started = False
        
        # Create a parametrized path for formation calculations
        self.param_path = ParametrizedPath()
        self.param_path.theta = 50.0  # Initial parameter value
        
        # Assign formation angles to each agent (distributed around the circle)
        self.agent_betas = [2 * np.pi * i / self.num_agents for i in range(self.num_agents)]
        self.formation_distance = 5.0  # Distance from formation center (lm in the original code)

        # Initialize empty publisher lists first
        self.agent_action_pubs = []
        self.agent_reset_pubs = []
        
        # Publishers
        self.state_pub = self.create_publisher(Float32MultiArray, '/environment/state', 10)
        self.reward_pub = self.create_publisher(Float32, '/environment/reward', 10)
        self.done_pub = self.create_publisher(Bool, '/environment/done', 10)
        self.reset_service = self.create_service(Trigger, '/environment/reset', self.reset_callback)

        # Create a callback for each agent
        for i in range(self.num_agents):
            self.agent_action_pubs.append(self.create_publisher(Float32MultiArray, f'/agent_{i}/action', 10))
            self.agent_reset_pubs.append(self.create_publisher(Float32MultiArray, f'/agent_{i}/reset', 10))

        # Subscribers
        self.action_sub = self.create_subscription(
            Float32MultiArray, '/ppo/action', self.action_callback, 10
        )
        self.agent_state_subs = []
        for i in range(self.num_agents):
            self.agent_state_subs.append(
                self.create_subscription(
                    Float32MultiArray, 
                    f'/agent_{i}/state_update',
                    self.create_state_callback(i),
                    10
                )
            )
        
        # Initial state timer
        self.initial_timer = self.create_timer(1.0, self.initial_state_publish_callback)
        
        # Keepalive timer (ensures environment state is published regularly)
        self.keepalive_timer = self.create_timer(0.5, self._keepalive_publish)
        
        self.get_logger().info(f'ASV Environment Node started with {self.num_agents} agents')
        
        # Create QoS profile for visualization (reliable, keep last 10)
        viz_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Visualization publishers
        self.viz_pub = self.create_publisher(MarkerArray, '/asv_env/visualization', viz_qos)

        # Visualization timer (update every 0.5 seconds)
        self.viz_timer = self.create_timer(0.5, self.publish_visualization)
    
        self.poses_pub = self.create_publisher(PoseArray, '/asv_env/asv_poses', 10)

    def create_state_callback(self, agent_id):
        def callback(msg):
            try:
                state = np.array(msg.data, dtype=np.float32)
                if state.size != 6:
                    self.get_logger().warn(f'Agent {agent_id} state has wrong size: {state.size}')
                    return
                    
                self.agent_states[agent_id] = state
                self.received_updates_this_step[agent_id] = True
                
                self.get_logger().info(f'Received state from agent {agent_id}: {state.tolist()}', throttle_duration_sec=1.0)
                
                # Debug current state of agent updates
                self.get_logger().info(f'Agent updates: {self.received_updates_this_step}, loop_started: {self.loop_started}', throttle_duration_sec=1.0)
                
                # If all agents have reported, update the environment state
                if all(self.received_updates_this_step) and self.loop_started:
                    self.publish_environment_state()
                    self.calculate_reward()
                    self.received_updates_this_step = [False] * self.num_agents
                else:
                    self.get_logger().info(f'Not publishing yet: all_reported={all(self.received_updates_this_step)}, loop_started={self.loop_started}', throttle_duration_sec=1.0)
            except Exception as e:
                self.get_logger().error(f'Error in state callback for agent {agent_id}: {str(e)}')
                
        return callback

    def initial_state_publish_callback(self):
        # Reset all agents and publish initial state
        self.trigger_reset()
        
        # IMPORTANT: Force the loop to start and publish first state
        self.loop_started = True
        
        # Wait a short time for agents to report their initial states
        self.create_timer(0.5, self._force_first_publish)
        
        self.initial_timer.cancel()  # Only run once
        
    def action_callback(self, msg):
        # Convert ROS message to NumPy array
        actions = DataConverter.ros_to_numpy(msg)
        
        if actions.size != 2 * self.num_agents:
            self.get_logger().error(f'Action message has wrong size: {actions.size}, expected {2 * self.num_agents}')
            return
            
        self.loop_started = True
        
        for i in range(self.num_agents):
            action_slice = actions[i*2:(i+1)*2]
            action_msg = DataConverter.numpy_to_ros(action_slice)
            self.agent_action_pubs[i].publish(action_msg)
            self.get_logger().info(f'Published action to agent {i}: {action_slice.tolist()}')

    def publish_environment_state(self):
        try:
            # Only publish if all agents have reported at least once
            if any(s is None for s in self.agent_states):
                self.get_logger().warn('Attempted to publish state, but not all agents have reported.')
                return

            # Basic agent states array (original data)
            agent_states_array = np.array(self.agent_states)
            self.get_logger().info(f'Agent states for publishing: {[s.tolist() for s in self.agent_states]}')
            
            # Calculate virtual leader position and path derivative
            pos_v, deriv = self.param_path.path(self.param_path.theta, True)
            self.get_logger().info(f'Virtual leader position: {pos_v.tolist()}, derivative: {deriv}')
        
            # Calculate formation centroid from mean of agent positions
            centroid = np.mean(np.array([state[:2] for state in self.agent_states]), axis=0)
            
            # Calculate desired positions for each agent
            desired_positions = []
            formation_error = 0.0
            
            for i, state in enumerate(self.agent_states):
                # Calculate expected position based on virtual leader and formation angle
                expected_pos = pos_v.flatten() + self.formation_distance * np.array([
                    np.cos(deriv.item() + self.agent_betas[i]), 
                    np.sin(deriv.item() + self.agent_betas[i])
                ])
                desired_positions.append(expected_pos)
                
                # Calculate formation error for this agent
                agent_error = np.linalg.norm(expected_pos - state[:2])
                formation_error += agent_error
            
            # Calculate average formation error
            formation_error = formation_error / self.num_agents
            
            # Calculate cross-track error from centroid
            cross_track_error = self.param_path.cross_track_error(centroid)
            
            # Create extended state array:
            # 1. Original agent states: [x, y, yaw, vx, vy, vyaw] for each agent
            # 2. Desired positions: [desired_x, desired_y] for each agent
            # 3. Formation info: [centroid_x, centroid_y, formation_error, cross_track_error]
            # 4. Virtual leader: [pos_v_x, pos_v_y]
            
            # Flatten original agent states
            global_state = agent_states_array.flatten()
            
            # Add desired positions for each agent
            for pos in desired_positions:
                global_state = np.append(global_state, pos)
            
            # Add formation information
            global_state = np.append(global_state, [
                centroid[0],           # Formation centroid x
                centroid[1],           # Formation centroid y
                formation_error,       # Average formation error
                cross_track_error,     # Cross track error
                pos_v.item(0),         # Virtual leader x
                pos_v.item(1)          # Virtual leader y
            ])
            
            # Publish the extended state
            state_msg = Float32MultiArray(data=global_state.tolist())
            self.state_pub.publish(state_msg)
            
            self.get_logger().info(
                f'Published extended environment state with {len(global_state)} elements',
                throttle_duration_sec=1.0
            )
        except Exception as e:
            self.get_logger().error(f'Error in publish_environment_state: {str(e)}')

    def _keepalive_publish(self):
        # Ensure environment state is published regularly
        if all(s is not None for s in self.agent_states) and self.loop_started:
            self.publish_environment_state()

    def trigger_reset(self):
        # Reset the path parameter - this sets where the virtual leader will be
        # Change to be well within valid bounds (50-90)
        self.param_path.theta = np.random.uniform(60, 80)
        
        # Get position and derivative at this parameter
        pos_v, deriv = self.param_path.path(self.param_path.theta, True)
        
        # Reset all agents to initial positions
        for i in range(self.num_agents):
            # Calculate expected position with SMALLER random offset
            expected_pos = pos_v.flatten() + self.formation_distance * np.array([
                np.cos(deriv.item() + self.agent_betas[i]), 
                np.sin(deriv.item() + self.agent_betas[i])
            ])
            
            # Add smaller random offset and ensure within bounds
            offset = np.random.uniform(-5, 5, size=2)  # Smaller offset (was -10, 10)
            
            # Position before boundary check
            x_pos = expected_pos[0] + offset[0]
            y_pos = expected_pos[1] + offset[1]
            
            # Ensure position is within bounds (50-90)
            x_pos = np.clip(x_pos, 55.0, 85.0)  # Add extra margin from edge
            y_pos = np.clip(y_pos, 55.0, 85.0)  # Add extra margin from edge
            
            reset_state = np.array([
                x_pos,   # x - clipped to bounds
                y_pos,   # y - clipped to bounds
                np.random.uniform(-np.pi, np.pi),  # yaw
                np.random.uniform(0, 1),       # vx
                0.0,                           # vy
                0.0                            # vyaw
            ])
            
            # Use pre-initialized publishers
            reset_msg = Float32MultiArray(data=reset_state.tolist())
            self.agent_reset_pubs[i].publish(reset_msg)
            
        self.received_updates_this_step = [False] * self.num_agents
        self.get_logger().info('Reset all agents to initial positions')

    def calculate_reward(self):
        # Only calculate if all agents have reported
        if any(s is None for s in self.agent_states):
            return
            
        # Calculate virtual leader position and path derivative
        pos_v, deriv = self.param_path.path(self.param_path.theta, True)
        
        # Average velocity reward components (Rv) and distance reward components (Rd)
        total_rv = 0.0
        total_rd = 0.0
        
        for i, state in enumerate(self.agent_states):
            # Calculate expected position
            expected_pos = pos_v.flatten() + self.formation_distance * np.array([
                np.cos(deriv.item() + self.agent_betas[i]), 
                np.sin(deriv.item() + self.agent_betas[i])
            ])
            
            # Calculate angle to desired position
            x_p1 = expected_pos - state[:2]
            angle = np.arctan2(x_p1[1], x_p1[0]) - state[2]
            
            # Velocity reward (similar to ASVAgent.Rv)
            k_v = 2.75
            rv = k_v * (state[3] * np.cos(angle) - (np.abs(state[4]) + np.abs(state[5])) * np.abs(np.sin(angle)))
            
            # Distance reward (similar to ASVAgent.Rd)
            k_d = 2.0
            err_max = 10.0
            error = np.linalg.norm(expected_pos - state[:2])
            rd = k_d * (-error / err_max)
            
            total_rv += rv
            total_rd += rd
        
        # Average rewards
        total_rv /= self.num_agents
        total_rd /= self.num_agents
        
        # Final reward
        reward = total_rv + total_rd
        
        # Publish reward
        self.reward_pub.publish(Float32(data=float(reward)))
        
        # Check if done and publish - use explicit bool conversion
        done = bool(self.check_done())
        self.done_pub.publish(Bool(data=done))

    def check_done(self):
        # Episode is done if:
        # 1. There's a collision between agents
        # 2. Formation has reached a successful state
        
        # Check for collisions
        if self.check_collision():
            return True
            
        # Check for successful formation
        centroid = np.mean(np.array([state[:2] for state in self.agent_states]), axis=0)
        along_track_error = self.param_path.along_track_error(centroid)
        
        # Success if along track error is small
        return along_track_error < 5.0

    def check_collision(self):
        if self.num_agents < 2:
            return False
            
        # Check distances between all pairs of agents
        for i in range(self.num_agents):
            for j in range(i+1, self.num_agents):
                pos_i = self.agent_states[i][:2]
                pos_j = self.agent_states[j][:2]
                distance = np.linalg.norm(pos_i - pos_j)
                
                # Collision threshold
                if distance < 2.0:
                    return True
                    
        return False

    def _force_first_publish(self):
        self.get_logger().info(f'AGENT STATES: {[s is not None for s in self.agent_states]}')
        if all(s is not None for s in self.agent_states):
            self.get_logger().info('First state publish forced to break action-state deadlock')
            self.publish_environment_state()
            return True
        else:
            self.get_logger().info('Waiting for all agents to report before forcing first state')
            return False
        
    def reset_callback(self, request, response):
        """Service to reset all agents to initial positions."""
        try:
            # Reset all agents to their initial positions
            self.reset_all_agents()
            
            response.success = True
            response.message = "All agents reset successfully"
            return response
        except Exception as e:
            self.get_logger().error(f"Failed to reset agents: {e}")
            response.success = False
            response.message = f"Failed to reset: {str(e)}"
            return response
    
    def reset_all_agents(self):
        """Reset all agents to initial positions."""
        # Instead of random positions or hard-coded positions at the edge,
        # use positions closer to the center of the map:
        for i in range(self.num_agents):
            if i == 0:
                # First boat - place at bottom left of center
                pos = [65.0, 65.0, 0.0, 0.0, 0.0, 0.0]
            else:
                # Second boat - place at top right of center
                pos = [75.0, 75.0, 0.0, 0.0, 0.0, 0.0]
            
            # Publish reset message to agent
            self.reset_agent(i, pos)
        
        self.get_logger().info("Reset all agents to initial positions")

    def reset_agent(self, agent_id, position):
        """
        Resets a specific agent to a given position.
        
        Args:
            agent_id: The ID of the agent to reset
            position: Array containing [x, y, yaw, vx, vy, vyaw]
        """
        if agent_id < 0 or agent_id >= self.num_agents:
            self.get_logger().error(f"Invalid agent ID for reset: {agent_id}")
            return
        
        # Convert position array to Float32MultiArray message
        reset_msg = Float32MultiArray(data=position)
        
        # Publish reset message to this agent
        self.agent_reset_pubs[agent_id].publish(reset_msg)
        self.get_logger().info(f"Reset agent {agent_id} to position {position}")
        
        # Clear any cached state for this agent
        self.agent_states[agent_id] = None
        self.received_updates_this_step[agent_id] = False

    # In order to see the parametrized path
    def publish_visualization(self):
        """Publish visualization markers for RViz2"""
        try:
            # Create a single MarkerArray for all visualizations
            all_markers = MarkerArray()
            
            # 1. Visualize the parametrized path
            path_marker = Marker()
            path_marker.header.frame_id = "map"
            path_marker.header.stamp = self.get_clock().now().to_msg()
            path_marker.ns = "path"
            path_marker.id = 0
            path_marker.type = Marker.LINE_STRIP
            path_marker.action = Marker.ADD
            path_marker.scale.x = 0.2  # Line width
            path_marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)  # Green
            path_marker.pose.orientation.w = 1.0
            
            # Sample points along the path
            thetas = np.linspace(0, 100, 200)  # Sample 200 points
            for theta in thetas:
                try:
                    pos, _ = self.param_path.path(theta, False)
                    # Handle different return formats from path function
                    if isinstance(pos, np.ndarray):
                        if pos.size == 1:
                            # If it's a single value array, create a point at (pos, pos)
                            point = Point(x=float(pos.item()), y=float(pos.item()), z=0.1)
                        else:
                            # It's an array with at least 2 values
                            point = Point(x=float(pos.item(0)), y=float(pos.item(0)), z=0.1)
                    else:
                        # Fallback for other types
                        point = Point(x=float(theta), y=float(theta), z=0.1)
                    path_marker.points.append(point)
                except Exception as e:
                    self.get_logger().warn(f"Error creating path point at theta={theta}: {e}")
                    continue
            
            all_markers.markers.append(path_marker)
            
            # 2. Visualize the current position on the path
            current_pos_marker = Marker()
            current_pos_marker.header.frame_id = "map"
            current_pos_marker.header.stamp = self.get_clock().now().to_msg()
            current_pos_marker.ns = "current_position"
            current_pos_marker.id = 1
            current_pos_marker.type = Marker.SPHERE
            current_pos_marker.action = Marker.ADD
            current_pos_marker.scale.x = 1.0
            current_pos_marker.scale.y = 1.0
            current_pos_marker.scale.z = 1.0
            current_pos_marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)  # Red
            
            pos_v, _ = self.param_path.path(self.param_path.theta, False)
            # Make sure pos_v is properly formatted for indexing
            if hasattr(pos_v, 'flatten'):
                pos_v = pos_v.flatten()
            elif not isinstance(pos_v, (list, np.ndarray)) or len(pos_v) < 2:
                # If pos_v is not properly formatted, use a default position
                self.get_logger().warn(f"Invalid pos_v format: {type(pos_v)}, value: {pos_v}")
                pos_v = np.array([70.0, 70.0])  # Default position in the center

            current_pos_marker.pose.position.x = float(pos_v[0])
            current_pos_marker.pose.position.y = float(pos_v[1])
            current_pos_marker.pose.position.z = 0.1
            current_pos_marker.pose.orientation.w = 1.0
            
            all_markers.markers.append(current_pos_marker)
            
            # 3. Visualize the formation (if agents have reported)
            if all(s is not None for s in self.agent_states):
                # Virtual leader position and derivative
                pos_v, deriv = self.param_path.path(self.param_path.theta, True)
                # Make sure pos_v is properly formatted for indexing
                if hasattr(pos_v, 'flatten'):
                    pos_v = pos_v.flatten()
                elif not isinstance(pos_v, (list, np.ndarray)) or len(pos_v) < 2:
                    self.get_logger().warn(f"Invalid pos_v format: {type(pos_v)}, value: {pos_v}")
                    pos_v = np.array([70.0, 70.0])
                
                # Create marker for virtual leader
                leader_marker = Marker()
                leader_marker.header.frame_id = "map"
                leader_marker.header.stamp = self.get_clock().now().to_msg()
                leader_marker.ns = "virtual_leader"
                leader_marker.id = 2
                leader_marker.type = Marker.SPHERE
                leader_marker.action = Marker.ADD
                leader_marker.scale.x = 0.8
                leader_marker.scale.y = 0.8
                leader_marker.scale.z = 0.8
                leader_marker.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=1.0)  # Yellow
                leader_marker.pose.position.x = float(pos_v[0])
                leader_marker.pose.position.y = float(pos_v[1])
                leader_marker.pose.position.z = 0.1
                leader_marker.pose.orientation.w = 1.0
                
                all_markers.markers.append(leader_marker)
                
                # Create markers for desired positions
                for i in range(self.num_agents):
                    # Calculate expected position
                    if hasattr(deriv, 'item'):
                        deriv_val = deriv.item()
                    else:
                        deriv_val = float(deriv)

                    expected_pos = pos_v + self.formation_distance * np.array([
                        np.cos(deriv_val + self.agent_betas[i]), 
                        np.sin(deriv_val + self.agent_betas[i])
                    ])
                    
                    marker_id = 3 + i
                    desired_marker = Marker()
                    desired_marker.header.frame_id = "map"
                    desired_marker.header.stamp = self.get_clock().now().to_msg()
                    desired_marker.ns = f"desired_position_{i}"
                    desired_marker.id = marker_id
                    desired_marker.type = Marker.SPHERE
                    desired_marker.action = Marker.ADD
                    desired_marker.scale.x = 0.5
                    desired_marker.scale.y = 0.5
                    desired_marker.scale.z = 0.5
                    desired_marker.color = ColorRGBA(r=0.0, g=0.0, b=1.0, a=0.7)  # Blue (semi-transparent)
                    desired_marker.pose.position.x = float(expected_pos[0])
                    desired_marker.pose.position.y = float(expected_pos[1])
                    desired_marker.pose.position.z = 0.1
                    desired_marker.pose.orientation.w = 1.0
                    
                    all_markers.markers.append(desired_marker)
                    
                    # Create a line between desired and actual position
                    line_marker = Marker()
                    line_marker.header.frame_id = "map"
                    line_marker.header.stamp = self.get_clock().now().to_msg()
                    line_marker.ns = f"error_line_{i}"
                    line_marker.id = marker_id + self.num_agents
                    line_marker.type = Marker.LINE_STRIP
                    line_marker.action = Marker.ADD
                    line_marker.scale.x = 0.1  # Line width
                    line_marker.color = ColorRGBA(r=1.0, g=0.5, b=0.0, a=0.7)  # Orange
                    line_marker.pose.orientation.w = 1.0
                    
                    # Add desired position point
                    p1 = Point(x=float(expected_pos[0]), y=float(expected_pos[1]), z=0.1)
                    line_marker.points.append(p1)
                    
                    # Add actual position point
                    p2 = Point(x=float(self.agent_states[i][0]), y=float(self.agent_states[i][1]), z=0.1)
                    line_marker.points.append(p2)
                    
                    all_markers.markers.append(line_marker)
            
            # Publish all markers as a single MarkerArray
            self.viz_pub.publish(all_markers)

            if all(s is not None for s in self.agent_states):
                pose_array = PoseArray()
                pose_array.header.frame_id = "map"
                pose_array.header.stamp = self.get_clock().now().to_msg()
                
                for state in self.agent_states:
                    pose = Pose()
                    pose.position.x = float(state[0])
                    pose.position.y = float(state[1])
                    pose.position.z = 0.0
                    
                    # Convert yaw to quaternion
                    yaw = float(state[2])
                    pose.orientation.x = 0.0
                    pose.orientation.y = 0.0
                    pose.orientation.z = math.sin(yaw/2)
                    pose.orientation.w = math.cos(yaw/2)
                    
                    pose_array.poses.append(pose)
                
                self.poses_pub.publish(pose_array)
            
        except Exception as e:
            self.get_logger().error(f"Error in publish_visualization: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = ASVEnvNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()