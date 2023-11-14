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
    robot_description = ParameterValue(Command(['xacro ', urdf_path]), value_type=str)

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{'robot_description': robot_description},
                    {'publish_frequency': 10.0}]
    )
    
    # mavros_node = Node(
    #     package="mavros",
    #     executable="mavros_node",
    #     parameters=[
    #         {"fcu_url": "udp://:14550@"},
    #     ]
    # )

    mavros_launch = IncludeLaunchDescription(
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

    listner_rc_node = Node (
        package= "asv_control",
        executable= "listener_rc",
        namespace= 'control',
        parameters = [config]
    )

    change_mode_node = Node (
        package= "asv_control",
        executable= "change_mode",
        namespace= 'control',
        parameters = [config]
    )

    observer_guille = Node (
        package= "asv_control",
        executable= "observer",
        name= "observer_guille",
        namespace= 'control',
        parameters = [
            {'my_id': LaunchConfiguration('my_id')},
            config
        ]
    )

    observer_liu = Node (
        package= "asv_control",
        executable= "observer_liu",
        name= "observer_liu",
        namespace= 'control',
        remappings=[
            ("/control/state_observer", "/control/state_observe_liu")
        ],
        parameters = [
            {'my_id': LaunchConfiguration('my_id')},
            config
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

    ld.add_action(mavros_launch)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(asv_tf_broadcast_node)
    ld.add_action(listner_rc_node)
    ld.add_action(change_mode_node)
    ld.add_action(observer_guille)
    ld.add_action(observer_liu)
    ld.add_action(pwm_mapper_node)
    ld.add_action(transceiver_xbee_node)
    ld.add_action(record)


    return ld