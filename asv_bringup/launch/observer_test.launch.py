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
    """

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
    my_id = LaunchConfiguration('my_id')
    rec = LaunchConfiguration('rec')
    my_namespace = PythonExpression(["'ASV' + str(", my_id, ")"])
    namespace_control = PythonExpression(
        ["'ASV' + str(", my_id, ") + '/control'"])
    namespace_comunication = PythonExpression(
        ["'ASV' + str(", my_id, ") + '/comunication'"])
    namespace_observer = PythonExpression(
        ["'ASV' + str(", my_id, ") + '/observer'"])

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
                                     'params_test.yaml'])
    config = PathJoinSubstitution([
        get_package_share_directory('asv_bringup'),
        'config',
        LaunchConfiguration('param_id')
    ])
    
    ###################################################################
    ##--------------------Comunication Nodes-------------------------##
    ################################################################### 

    # Ejecuta rosbag record automáticamente al lanzar el launch file
    record= ExecuteProcess(
        cmd=["ros2", "bag", "record", "--all"],  
        output="screen",
        condition=IfCondition(rec)
    )

    rvz = Node(
            package='rviz2',
            namespace='',
            executable='rviz2',
            name='rviz2',
            arguments=['-d' + os.path.join(get_package_share_directory('asv_bringup'), 'config', 'test_obs.rviz')],
        )
    
    
    rqt_node = ExecuteProcess(
        cmd=["rqt", "--perspective-file", os.path.join(get_package_share_directory('asv_bringup'), 'config', 'test_obs.perspective')],
        output="screen"
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

    ###################################################################
    ## -----------------------Observer Nodes--------------------------##
    ###################################################################

    observer_zono_1 = Node(
        package="asv_observer",
        executable="observer_zono",
        name="observer_zono_1",
        namespace=namespace_observer,
        parameters=[
            {'my_id': my_namespace},
            config
        ],
        remappings=[
            ("state_observer_zono", "state_observer_zono_1"),
            ("state_observer_zono_min", "state_observer_zono_min_1"),
            ("state_observer_zono_max", "state_observer_zono_max_1"),
            ("pose_zono", "pose_zono_1"),
            ("sigmas_zono", "sigmas_zono_1")
        ]
    )

    observer_zono_2 = Node(
        package="asv_observer",
        executable="observer_zono",
        name="observer_zono_2",
        namespace=namespace_observer,
        parameters=[
            {'my_id': my_namespace},
            config
        ],
        remappings=[
            ("state_observer_zono", "state_observer_zono_2"),
            ("state_observer_zono_min", "state_observer_zono_min_2"),
            ("state_observer_zono_max", "state_observer_zono_max_2"),
            ("pose_zono", "pose_zono_2"),
            ("sigmas_zono", "sigmas_zono_2")
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
    # nodes.append(own_robot_state_publisher_node)
    # nodes.append(tf_map_node)
    

    ###################################################################
    ##--------------------Comunication Nodes-------------------------##
    ################################################################### 
    nodes.append(record)
    # nodes.append(rvz)
    # nodes.append(rqt_node)
    
    ###################################################################
    ##-----------------------Observer Nodes--------------------------##
    ################################################################### 
    # nodes.append(observer_zono_1)
    nodes.append(observer_zono_2)
    #nodes.append(observer_param)
    # nodes.append(observer_core)
    

    ###################################################################
    ##-----------------------Control Nodes---------------------------##
    ################################################################### 
    # nodes.append(asv_tf_broadcast_node)

    

    return LaunchDescription(
        [arg_my_id, arg_rec, param_id,
            *nodes]
    )