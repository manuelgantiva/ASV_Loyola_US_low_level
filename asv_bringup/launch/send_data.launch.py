from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld= LaunchDescription()


    send_xbee_node = Node(
        package="asv_comunication",
        executable="send_xbee.py"
    )


    test_observer_node = Node (
        package= "asv_comunication",
        executable= "test_observer.py",
    )

    
    ld.add_action(send_xbee_node)
    ld.add_action(test_observer_node)


    return ld