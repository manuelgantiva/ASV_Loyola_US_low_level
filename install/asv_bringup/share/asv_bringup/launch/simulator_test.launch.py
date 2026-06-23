from launch import LaunchDescription
from launch_ros.actions import Node

# Exec robot description node with xacro
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
from launch.actions import ExecuteProcess

# Retrieving path information
import os
from ament_index_python.packages import get_package_share_directory

# Add Arguments Launch
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression

# Add Arguments IfCondition Launch
from launch.conditions import IfCondition

# Exec other Launch
from launch.actions import IncludeLaunchDescription
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource


def generate_launch_description():
    """
    Launch file for the ASV simulator test

    args:
        my_id: Vehicle ID, is a single character string
        rec: Record bag file
        pub2neigh: Publish to neighbor
    """

    # TODO: Add log level
    # DeclareLaunchArgument(name='log_level', default_value='info'),
    # On each node:
    # arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')]
    # Modify INFO to DEBUG in the nodes

    arg_my_id = DeclareLaunchArgument(
        'my_id',
        default_value="0",
        description='Vehicle ID, is a single character string'
    )
    arg_rec = DeclareLaunchArgument(
        'rec',
        default_value="false",
        description='Record bag file'
    )
    arg_worker_mode = DeclareLaunchArgument(
        'worker_mode',
        default_value="1",
        description='Distributed: -1, Coordinator: 0, Worker: 1, 2, ...'
    )
    my_id = LaunchConfiguration('my_id')
    rec = LaunchConfiguration('rec')
    worker_mode = LaunchConfiguration('worker_mode')
    my_namespace = PythonExpression(["'ASV' + str(", my_id, ")"])
    namespace_control = PythonExpression(
        ["'ASV' + str(", my_id, ") + '/control'"])
    namespace_comunication = PythonExpression(
        ["'ASV' + str(", my_id, ") + '/comunication'"])

    namespace_observer = PythonExpression(
        ["'ASV' + str(", my_id, ") + '/observer'"])
    namespace_simulator = PythonExpression(
        ["'ASV' + str(", my_id, ") + '/simulator'"])

    nodes = []
    ###################################################################
    ## -------------------Robot description rviz2--------------------##
    ###################################################################

    yf_pkg = get_package_share_directory("yf_description")
    urdf_path = os.path.join(yf_pkg, 'urdf', 'asv_loyola.urdf.xacro')
    own_description = ParameterValue(Command(['xacro ', urdf_path, ' id:=', my_id, ' own:=true']),
                                     value_type=str)

    own_robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="own_robot_state_publisher",
        namespace=my_namespace,
        parameters=[{'robot_description': own_description},
                    {'publish_frequency': 10.0}],
        remappings=[
            ("/robot_description", "/own_description")
        ]
    )

    tf_map_node = ExecuteProcess(
        cmd=[
            "ros2", "run", "tf2_ros", "static_transform_publisher",
            "0", "0", "0",  # x, y, z
            "1.5708", "0", "3.1415",  # roll, pitch, yaw
            "map", "map_ned"  # Frame de origen y destino
        ],
        output="screen"
    )

    ###################################################################
    ##--------------------Get Config id File ------------------------##
    ################################################################### 

    param_id = DeclareLaunchArgument('param_id', default_value=[
                                     'params_', my_id, '.yaml'])
    config = PathJoinSubstitution([
        get_package_share_directory('asv_bringup'),
        'config',
        LaunchConfiguration('param_id')
    ])
    
    config_gen = os.path.join(get_package_share_directory('asv_bringup'),
        'config',
        'params_gen.yaml'
    )

    ###################################################################
    ##--------------------Simulation Nodes---------------------------##
    ################################################################### 
   
    asv_simulator_node = Node(
        package="asv_simulator",
        executable="simulator",
        namespace= namespace_simulator,
        parameters=[{'my_id': my_id},
                    config],
    )

    rvz = Node(
            package='rviz2',
            namespace='',
            executable='rviz2',
            name='rviz2',
            arguments=['-d' + os.path.join(get_package_share_directory('asv_bringup'), 'rviz2', 'mlc_config.rviz')],
        )
    
    
    rqt_node = ExecuteProcess(
        cmd=["rqt", "--perspective-file", os.path.join(get_package_share_directory('asv_bringup'), 'rqt', 'mlc_test.perspective')],
        output="screen"
    )

    ###################################################################
    ##--------------------Comunication Nodes-------------------------##
    ################################################################### 

    record = Node(
        package="asv_comunication",
        executable="bag_record",
        namespace= namespace_comunication,
        parameters = [
            {'my_id': my_id}
        ],
        condition=IfCondition(rec)
    )

    rc_handler_node = Node(
        package="asv_comunication",
        executable="rc_handler",
        namespace= namespace_comunication,
        parameters = [
            {'my_id': my_namespace},
            config_gen]
    )

    ref_llc_node = Node(
        package="asv_comunication",
        executable="ref_llc",
        namespace= namespace_comunication,
        parameters = [
            {'my_id': my_namespace},
            config_gen]
    )

    ref_mlc_node = Node(
        package="asv_comunication",
        executable="ref_mlc",
        namespace= namespace_comunication,
        parameters = [
            {'my_id': my_namespace},
            config_gen]
    )


    
    transceiver_sim_node = Node(
        package="asv_comunication",
        executable="transceiver_simulator.py",
        namespace= namespace_comunication,
        parameters = [
            {'my_id': my_namespace,
            'worker_mode': worker_mode},
        ]
    )

    ###################################################################
    ##-----------------------Control Nodes---------------------------##
    ################################################################### 
  
    asv_tf_broadcast_node = Node(
        package="asv_control",
        executable="asv_tf2_broadcaster",
        namespace= namespace_control,
        parameters = [
            {'my_id': my_namespace}
        ]
    )

    pwm_mapper_node = Node(
        package="asv_control",
        executable="pwm_mapper",
        namespace= namespace_control,
        parameters = [
            {'my_id': my_namespace},
        ]
    )
    
    mux_llc_node = Node(
        package="asv_control",
        executable="mux_llc",
        namespace= namespace_control,
        parameters = [
            {'my_id': my_namespace},
        ]
    )

    ifac_llc_node = Node(
        package="asv_control",
        executable="ifac_llc",
        namespace= namespace_control,
        parameters = [{'my_id': my_namespace},
            config],
    )

    mpc_llc_rt_node = Node(
        package="asv_acados",
        executable="mpc_llc_rt",
        name="mpc_llc",
        namespace= namespace_control,
        parameters = [{'my_id': my_namespace},
            config],
    )

    wang_mlc_node = Node(
        package="asv_control",
        executable="wang_mlc",
        namespace= namespace_control,
        parameters = [{'my_id': my_namespace},
                config],
    )
    
    mpc_mlc_pf_node = Node(
        package="asv_control",
        executable="mpc_mlc_pf",
        namespace= namespace_control,
        parameters = [{'my_id': my_namespace},
            config],
    )

    ###################################################################
    ## -----------------------Observer Nodes--------------------------##
    ###################################################################

    mux_obs_node = Node(
        package="asv_observer",
        executable="mux_obs",
        namespace=namespace_observer,
        parameters=[
            {'my_id': my_namespace},
        ]
    )

    observer_bejarano = Node(
        package="asv_observer",
        executable="observer_bejarano",
        name="observer_bejarano",
        namespace=namespace_observer,
        parameters=[
            {'my_id': my_namespace},
            config
        ]
    )

    observer_liu = Node(
        package="asv_observer",
        executable="observer_liu",
        name="observer_liu",
        namespace=namespace_observer,
        parameters=[
            {'my_id': my_namespace},
            config
        ],
    )

    observer_zono = Node(
        package="asv_observer",
        executable="observer_zono",
        name="observer_zono",
        namespace=namespace_observer,
        parameters=[
            {'my_id': my_namespace},
            config
        ]
    )

    observer_param = Node(
        package="asv_observer",
        executable="observer_param",
        name="observer_param",
        namespace=namespace_observer,
        parameters=[
            {'my_id': my_namespace},
            config
        ]
    )

    observer_core = Node(
        package="asv_observer",
        executable="observer_core",
        name="observer_core",
        namespace=namespace_observer,
        parameters=[
            {'my_id': my_namespace},
            config
        ]
    )

    ###################################################################
    ##-------------------------ASVs Nodes----------------------------##
    ################################################################### 
    nodes.append(asv_simulator_node)
    nodes.append(own_robot_state_publisher_node)
    nodes.append(tf_map_node)
    # nodes.append(rqt_node)
    nodes.append(rvz)

    ###################################################################
    ##--------------------Comunication Nodes-------------------------##
    ################################################################### 
    nodes.append(ref_mlc_node)
    # nodes.append(ref_llc_node)
    nodes.append(rc_handler_node)
    # nodes.append(record)
    # nodes.append(transceiver_sim_node)


    ###################################################################
    ##-----------------------Observer Nodes--------------------------##
    ################################################################### 
    nodes.append(mux_obs_node)
    #nodes.append(observer_bejarano)
    #nodes.append(observer_liu)
    nodes.append(observer_zono)
    # nodes.append(observer_param)
    nodes.append(observer_core)

    ###################################################################
    ##-----------------------Control Nodes---------------------------##
    ################################################################### 
    nodes.append(ifac_llc_node)
    nodes.append(mux_llc_node)
    nodes.append(pwm_mapper_node)
    #nodes.append(wang_mlc_node)
    # nodes.append(mpc_llc_rt_node)
    nodes.append(mpc_mlc_pf_node)

    nodes.append(asv_tf_broadcast_node)

    return LaunchDescription(
        [arg_my_id, arg_rec, param_id, arg_worker_mode,
            *nodes]
    )
