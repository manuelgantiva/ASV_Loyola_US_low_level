from launch import LaunchDescription
from launch_ros.actions import Node

# Retrieving path information 
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld= LaunchDescription()

    config = os.path.join(
        get_package_share_directory('asv_bringup'),
        'config',
        'params.yaml'
        )

    #mavros_node = Node(
    #    package="mavros",
    #    executable="mavros_node",
    #    parameters=[
    #        {"fcu_url": "udp://:14550@"},
    #    ]
    #)

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

    observer_node = Node (
        package= "asv_control",
        executable= "observer",
        namespace= 'control',
        parameters = [config]
    )

    pwm_mapper_node = Node (
        package= "asv_control",
        executable= "pwm_mapper",
        namespace= 'control',
        parameters = [config]
    )

    #ld.add_action(mavros_node)
    #ld.add_action(listner_rc_node)
    #ld.add_action(change_mode_node)
    ld.add_action(observer_node)
    #ld.add_action(pwm_mapper_node)


    return ld