from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    ld= LaunchDescription()

    arg_Id=DeclareLaunchArgument(
        'my_id',
        default_value="0",
        description='Vehicle ID, is a single character string'
    )

    transceiver_xbee_node = Node(
        package="asv_comunication",
        executable="transceiver_xbee.py",
        namespace= 'comunication'
    )


    test_observer_node = Node (
        package= "asv_comunication",
        executable= "test_observer.py",
        namespace= 'comunication',
        parameters=[{'my_id': LaunchConfiguration('my_id')}]
    )

    ld.add_action(arg_Id)
    ld.add_action(transceiver_xbee_node)
    ld.add_action(test_observer_node)


    return ld