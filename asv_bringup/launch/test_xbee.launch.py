from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld= LaunchDescription()


    transceiver_xbee_node = Node(
        package="asv_comunication",
        executable="transceiver_xbee.py",
        namespace= 'comunication'
    )


    test_observer_node = Node (
        package= "asv_comunication",
        executable= "test_observer.py",
        namespace= 'comunication'
    )

    
    ld.add_action(transceiver_xbee_node)
    ld.add_action(test_observer_node)


    return ld