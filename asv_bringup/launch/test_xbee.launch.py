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
from launch.substitutions import LaunchConfiguration

from launch.actions import ExecuteProcess

def generate_launch_description():
    ld= LaunchDescription()

    arg_Id=DeclareLaunchArgument(
        'my_id',
        default_value="0",
        description='Vehicle ID, is a single character string'
    )

    record = Node(
        package="asv_comunication",
        executable="bag_record",
        namespace= 'comunication',
        parameters = [
            {'my_id': LaunchConfiguration('my_id')}
        ]
    )

    config = os.path.join(
        get_package_share_directory('asv_bringup'),
        'config',
        'params.yaml'
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

    # mavros_node = Node(
    #     package="mavros",
    #     executable="mavros_node",
    #     parameters=[
    #         {"fcu_url": "udp://:14550@"},
    #     ]
    # )

    Mavros_launch = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            os.path.join(get_package_share_directory('asv_bringup'),
                         'launch/apm.launch.xml')
        )
    )

    asv_tf_broadcast_node = Node (
        package= "asv_control",
        executable= "asv_tf2_broadcaster",
        namespace= 'control',
        parameters = [config]
    )

    rc_handler_node = Node (
        package= "asv_control",
        executable= "rc_handler",
        namespace= 'control',
        parameters = [config]
    )

    ref_llc_node = Node (
        package= "asv_control",
        executable= "ref_llc",
        namespace= 'control',
        parameters = [config]
    )

    apm_llc_node = Node (
        package= "asv_control",
        executable= "apm_llc",
        namespace= 'control',
        parameters = [config]
    )

    ifac_llc_node = Node (
        package= "asv_control",
        executable= "ifac_llc",
        namespace= 'control',
        parameters = [config],
        remappings=[
            ("/control/pwm_value_ifac", "/control/pwm_value_ifac_2")
        ]
    )

    mux_llc_node = Node (
        package= "asv_control",
        executable= "mux_llc",
        namespace= 'control',
        parameters = [config]
    )

    mux_obs_node = Node (
        package= "asv_control",
        executable= "mux_obs",
        namespace= 'control',
        parameters = [config]
    )

    observer_guille = Node (
        package= "asv_control",
        executable= "observer_guille",
        name= "observer_guille",
        namespace= 'control',
        parameters = [
            {'my_id': LaunchConfiguration('my_id')},
            config
        ],
        remappings=[
            ("/control/state_observer_guille", "/control/state_observer_guille_2")
        ]
    )

    observer_liu = Node (
        package= "asv_control",
        executable= "observer_liu",
        name= "observer_liu",
        namespace= 'control',
        parameters = [
            {'my_id': LaunchConfiguration('my_id')},
            config
        ],
        remappings=[
            ("/control/state_observer_liu", "/control/state_observer_liu_2")
        ]
    )

    pwm_mapper_node = Node (
        package= "asv_control",
        executable= "pwm_mapper",
        namespace= 'control',
        parameters = [config]
    )

    transceiver_xbee_node = Node(
        package="asv_comunication",
        executable="transceiver_xbee.py",
        namespace= 'comunication'
    )

    # ld.add_action(Mavros_launch)
    # ld.add_action(own_robot_state_publisher_node)
    # ld.add_action(neighbor_robot_state_publisher_node)
    # ld.add_action(asv_tf_broadcast_node)
    # ld.add_action(rc_handler_node)
    # ld.add_action(ref_llc_node)
    # ld.add_action(mux_llc_node)
    ld.add_action(mux_obs_node)
    # ld.add_action(observer_guille)
    # ld.add_action(observer_liu)
    # ld.add_action(pwm_mapper_node)
    # ld.add_action(apm_llc_node)
    # ld.add_action(ifac_llc_node)
    # ld.add_action(record)
    # ld.add_action(transceiver_xbee_node)

    return ld