from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld= LaunchDescription()

    remap_number_topic= ("alive_turtles", "my_alive_turtles")

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
    )

    change_mode_node = Node (
        package= "asv_control",
        executable= "change_mode",
    )

    observer_node = Node (
        package= "asv_control",
        executable= "observer",
    )

    pwm_mapper_node = Node (
        package= "asv_control",
        executable= "pwm_mapper",
    )

    #ld.add_action(mavros_node)
    ld.add_action(listner_rc_node)
    ld.add_action(change_mode_node)
    ld.add_action(observer_node)
    ld.add_action(pwm_mapper_node)


    return ld