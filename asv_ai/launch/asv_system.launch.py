import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def launch_setup(context, *args, **kwargs):
    """
    This function is executed at launch time to resolve the `num_agents`
    launch argument before creating the agent-specific nodes.
    """
    num_agents_str = LaunchConfiguration('num_agents').perform(context)
    num_agents = int(num_agents_str)
    
    urdf_path = PathJoinSubstitution([
        FindPackageShare('yf_description'),
        'urdf',
        'asv_loyola.urdf.xacro'
    ])

    nodes_to_launch = []
    for i in range(num_agents):
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
                # The frame_prefix is not needed because the xacro file handles it
                # 'frame_prefix': f'agent_{i}/' 
            }],
            remappings=[
                ('/robot_description', '/robot_description')
            ],
            output='screen'
        )
        nodes_to_launch.append(robot_state_publisher_node)
    
    return nodes_to_launch

def generate_launch_description():
    num_agents_arg = DeclareLaunchArgument(
        'num_agents', default_value='2', description='Number of ASV agents'
    )
    model_path_arg = DeclareLaunchArgument(
        'model_path', default_value='', description='Path to pre-trained PPO model'
    )
    
    num_agents = LaunchConfiguration('num_agents')
    model_path = LaunchConfiguration('model_path')
    
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
        parameters=[{'num_agents': num_agents, 'model_path': model_path}],
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

    rqt_graph_node = ExecuteProcess(
        cmd=['rqt_graph'],
        shell=True
    )
    
    ld = LaunchDescription()
    
    ld.add_action(num_agents_arg)
    ld.add_action(model_path_arg)
    
    ld.add_action(env_node)
    ld.add_action(ppo_node)
    ld.add_action(rviz_node)
    ld.add_action(rqt_graph_node)
    
    ld.add_action(OpaqueFunction(function=launch_setup))
    
    # This return statement was missing
    return ld