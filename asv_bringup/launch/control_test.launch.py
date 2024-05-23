from launch import LaunchDescription
from launch_ros.actions import Node

# Exec robot description node with xacro
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command

# Retrieving path information 
import os
from ament_index_python.packages import get_package_share_directory

# Exec other Launch
from launch.actions import IncludeLaunchDescription
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource

# Add Arguments Launch
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression

# Add Arguments IfCondition Launch
from launch.conditions import IfCondition

def generate_launch_description():
    ld= LaunchDescription()

    arg_Id=DeclareLaunchArgument(
        'my_id',
        default_value="0",
        description='Vehicle ID, is a single character string'
    )
    
    # Obtiene el valor del argumento
    my_id = LaunchConfiguration('my_id')

    record = Node(
        package="asv_comunication",
        executable="bag_record",
        namespace= 'comunication',
        parameters = [
            {'my_id': LaunchConfiguration('my_id')}
        ]
    )

    config_1 = os.path.join(
        get_package_share_directory('asv_bringup'),
        'config',
        'params_1.yaml'
    )
    
    config_3 = os.path.join(
        get_package_share_directory('asv_bringup'),
        'config',
        'params_3.yaml'
    )
    
    config_4 = os.path.join(
        get_package_share_directory('asv_bringup'),
        'config',
        'params_4.yaml'
    )
        
    yf_pkg = get_package_share_directory("yf_description")
    urdf_path = os.path.join(yf_pkg, 'urdf', 'asv_loyola.urdf.xacro')
    own_description = ParameterValue(Command(['xacro ', urdf_path, ' id:=1', ' own:=true']), 
                                            value_type=str)

    own_robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="own_robot_state_publisher",
        parameters=[{'robot_description': own_description},
                    {'publish_frequency': 10.0}],
        remappings=[
            ("/robot_description", "/own_description")
        ]
    )

    neighbor_description = ParameterValue(Command(['xacro ', urdf_path, ' id:=0', ' own:=false']),
                                            value_type=str)

    neighbor_robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="neighbor_robot_state_publisher",
        parameters=[{'robot_description': neighbor_description},
                    {'publish_frequency': 10.0}],
        remappings=[
            ("/robot_description", "/neighbor_description")
        ]
    )

    Mavros_launch = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            os.path.join(get_package_share_directory('asv_bringup'),
                         'launch/apm.launch.xml')
        )
    )

    asv_tf_broadcast_node = Node (
        package= "asv_control",
        executable= "asv_tf2_broadcaster",
        namespace= 'control'
    )

    rc_handler_node = Node (
        package= "asv_control",
        executable= "rc_handler",
        namespace= 'control',
        parameters = [config_3]
    )

    ref_llc_node = Node (
        package= "asv_control",
        executable= "ref_llc",
        namespace= 'control',
        parameters = [config_3]
    )

    ref_mlc_node = Node (
        package= "asv_control",
        executable= "ref_mlc",
        namespace= 'control',
        parameters = [config_3]
    )

    apm_llc_node = Node (
        package= "asv_control",
        executable= "apm_llc",
        namespace= 'control'
    )
    
    ifac_llc_node1 = Node (
        package= "asv_control",
        executable= "ifac_llc",
        namespace= 'control',
        parameters = [config_1],
        condition=IfCondition(
            PythonExpression(
                [my_id, ' == 1']
            )
        )
    )

    ifac_llc_node3 = Node (
        package= "asv_control",
        executable= "ifac_llc",
        namespace= 'control',
        parameters = [config_3],
        condition=IfCondition(
            PythonExpression(
                [my_id, ' == 3']
            )
        )
    )
    
    ifac_llc_node4 = Node (
        package= "asv_control",
        executable= "ifac_llc",
        namespace= 'control',
        parameters = [config_4],
        condition=IfCondition(
            PythonExpression(
                [my_id, ' == 4']
            )
        )
    )

    wang_mlc_node1 = Node (
        package= "asv_control",
        executable= "wang_mlc",
        namespace= 'control',
        parameters = [config_1],
        condition=IfCondition(
            PythonExpression(
                [my_id, ' == 1']
            )
        )
    )
    
    wang_mlc_node3 = Node (
        package= "asv_control",
        executable= "wang_mlc",
        namespace= 'control',
        parameters = [config_3],
        condition=IfCondition(
            PythonExpression(
                [my_id, ' == 3']
            )
        )
    )
    
    wang_mlc_node4 = Node (
        package= "asv_control",
        executable= "wang_mlc",
        namespace= 'control',
        parameters = [config_4],
        condition=IfCondition(
            PythonExpression(
                [my_id, ' == 4']
            )
        )
    )

    mux_llc_node = Node (
        package= "asv_control",
        executable= "mux_llc",
        namespace= 'control'
    )

    mux_obs_node = Node (
        package= "asv_control",
        executable= "mux_obs",
        namespace= 'control'
    )
    
    observer_guille1 = Node (
        package= "asv_control",
        executable= "observer_guille",
        name= "observer_guille",
        namespace= 'control',
        parameters = [
            {'my_id': LaunchConfiguration('my_id')},
            config_1
        ],
        condition=IfCondition(
            PythonExpression(
                [my_id, ' == 1']
            )
        )
    )

    observer_guille3 = Node (
        package= "asv_control",
        executable= "observer_guille",
        name= "observer_guille",
        namespace= 'control',
        parameters = [
            {'my_id': LaunchConfiguration('my_id')},
            config_3
        ],
        condition=IfCondition(
            PythonExpression(
                [my_id, ' == 3']
            )
        )
    )
    
    observer_guille4 = Node (
        package= "asv_control",
        executable= "observer_guille",
        name= "observer_guille",
        namespace= 'control',
        parameters = [
            {'my_id': LaunchConfiguration('my_id')},
            config_4
        ],
        condition=IfCondition(
            PythonExpression(
                [my_id, ' == 4']
            )
        )
    )

    observer_liu1 = Node (
        package= "asv_control",
        executable= "observer_liu",
        name= "observer_liu",
        namespace= 'control',
        parameters = [
            {'my_id': LaunchConfiguration('my_id')},
            config_1
        ],
        condition=IfCondition(
            PythonExpression(
                [my_id, ' == 1']
            )
        )
    )
    
    observer_liu3 = Node (
        package= "asv_control",
        executable= "observer_liu",
        name= "observer_liu",
        namespace= 'control',
        parameters = [
            {'my_id': LaunchConfiguration('my_id')},
            config_3
        ],
        condition=IfCondition(
            PythonExpression(
                [my_id, ' == 3']
            )
        )
    )
    
    observer_liu4 = Node (
        package= "asv_control",
        executable= "observer_liu",
        name= "observer_liu",
        namespace= 'control',
        parameters = [
            {'my_id': LaunchConfiguration('my_id')},
            config_4
        ],
        condition=IfCondition(
            PythonExpression(
                [my_id, ' == 4']
            )
        )
    )

    pwm_mapper_node = Node (
        package= "asv_control",
        executable= "pwm_mapper",
        namespace= 'control'
    )

    transceiver_xbee_node = Node(
        package="asv_comunication",
        executable="transceiver_xbee.py",
        namespace= 'comunication'
    )

    ld.add_action(Mavros_launch)
    ld.add_action(own_robot_state_publisher_node)
    ld.add_action(neighbor_robot_state_publisher_node)
    ld.add_action(asv_tf_broadcast_node)
    ld.add_action(rc_handler_node)
    # ld.add_action(ref_llc_node)
    ld.add_action(ref_mlc_node)
    ld.add_action(mux_llc_node)
    ld.add_action(mux_obs_node)
    ld.add_action(pwm_mapper_node)
    ld.add_action(apm_llc_node)
    # ASV_nodes
    ld.add_action(observer_guille1)
    ld.add_action(observer_guille3)
    ld.add_action(observer_guille4)
    ld.add_action(observer_liu1)
    ld.add_action(observer_liu3)
    ld.add_action(observer_liu4)
    ld.add_action(ifac_llc_node1)
    ld.add_action(ifac_llc_node3)
    ld.add_action(ifac_llc_node4)
    ld.add_action(wang_mlc_node1)
    ld.add_action(wang_mlc_node3)
    ld.add_action(wang_mlc_node4)
    ld.add_action(record)
    ld.add_action(transceiver_xbee_node)

    return ld