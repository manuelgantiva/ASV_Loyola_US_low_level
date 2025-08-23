import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # Declare all the launch arguments
    num_agents_arg = DeclareLaunchArgument(
        'num_agents', default_value='2', 
        description='Number of ASV agents'
    )
    model_path_arg = DeclareLaunchArgument(
        'model_path', default_value='', 
        description='Path to pre-trained PPO model'
    )
    train_freq_arg = DeclareLaunchArgument(
        'train_frequency', default_value='1000',
        description='Steps between training updates'
    )
    batch_size_arg = DeclareLaunchArgument(
        'batch_size', default_value='64',
        description='PPO batch size'
    )
    n_epochs_arg = DeclareLaunchArgument(
        'n_epochs', default_value='10',
        description='PPO training epochs'
    )
    rollout_dir_arg = DeclareLaunchArgument(
        'rollout_dir', default_value='~/Desktop/ASV_Rollouts',
        description='Directory to save rollouts and models'
    )
    
    # Get the launch configurations
    num_agents = LaunchConfiguration('num_agents')
    model_path = LaunchConfiguration('model_path')
    train_frequency = LaunchConfiguration('train_frequency')
    batch_size = LaunchConfiguration('batch_size')
    n_epochs = LaunchConfiguration('n_epochs')
    rollout_dir = LaunchConfiguration('rollout_dir')
    
    # Define nodes that don't depend on agent count
    env_node = Node(
        package='asv_ai',
        executable='asv_env_node',
        name='asv_env_node',
        parameters=[{'num_agents': num_agents}],
        output='screen'
    )
    
    ppo_node = Node(
        package='asv_ai',
        executable='asv_ppo_node',
        name='asv_ppo_node',
        parameters=[{
            'num_agents': num_agents, 
            'model_path': model_path,
            'training_enabled': True,
            'train_frequency': train_frequency,
            'batch_size': batch_size,
            'n_epochs': n_epochs,
            'rollout_dir': rollout_dir
        }],
        output='screen'
    )
    
    rviz_config_path = PathJoinSubstitution([
        FindPackageShare('asv_ai'), 'rviz', 'asv.rviz'
    ])
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen'
    )
    
    # Function to create agent nodes - this matches your system launch file
    def launch_setup(context):
        num_agents_str = LaunchConfiguration('num_agents').perform(context)
        num_agents_value = int(num_agents_str)
        
        urdf_path = PathJoinSubstitution([
            FindPackageShare('yf_description'),  # Changed from asv_description to yf_description
            'urdf',
            'asv_loyola.urdf.xacro'  # Using the same URDF as in system launch
        ])

        nodes_to_launch = []
        for i in range(num_agents_value):
            robot_description = ParameterValue(
                Command(['xacro ', urdf_path, f' id:={i}', ' own:=true']),
                value_type=str
            )

            agent_node = Node(
                package='asv_ai',
                executable='asv_agent_node',
                name=f'asv_agent_node_{i}',
                namespace=f'agent_{i}',
                parameters=[{'agent_id': i}],
                output='screen'
            )
            nodes_to_launch.append(agent_node)

            robot_state_publisher_node = Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name=f'robot_state_publisher_{i}',
                namespace=f'agent_{i}',
                parameters=[{
                    'robot_description': robot_description,
                }],
                remappings=[
                    ('/robot_description', '/robot_description')
                ],
                output='screen'
            )
            nodes_to_launch.append(robot_state_publisher_node)
        
        return nodes_to_launch
    
    # Add all to launch description
    ld = LaunchDescription()
    
    # Add all arguments
    ld.add_action(num_agents_arg)
    ld.add_action(model_path_arg)
    ld.add_action(train_freq_arg)
    ld.add_action(batch_size_arg)
    ld.add_action(n_epochs_arg)
    ld.add_action(rollout_dir_arg)
    
    # Add all nodes
    ld.add_action(env_node)
    ld.add_action(ppo_node)
    ld.add_action(rviz_node)
    ld.add_action(OpaqueFunction(function=launch_setup))
    
    return ld