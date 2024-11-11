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
    my_namespace = PythonExpression(["'ASV' + str(", my_id, ")"])
    namespace_control= PythonExpression(["'ASV' + str(", my_id, ") + '/control'"])
    namespace_comunication= PythonExpression(["'ASV' + str(", my_id, ") + '/comunication'"])
    namespace_mavros = PythonExpression(["'ASV' + str(", my_id, ") + '/mavros'"])
    namespace_observer= PythonExpression(["'ASV' + str(", my_id, ") + '/observer'"])

    config_gen = os.path.join(get_package_share_directory('asv_bringup'),
        'config',
        'params_gen.yaml'
    )

    config_0 = os.path.join(get_package_share_directory('asv_bringup'),
        'config',
        'params_0.yaml'
    )

    config_1 = os.path.join(get_package_share_directory('asv_bringup'),
        'config',
        'params_1.yaml'
    )
    
    config_3 = os.path.join(get_package_share_directory('asv_bringup'),
        'config',
        'params_3.yaml'
    )
    
    config_4 = os.path.join(get_package_share_directory('asv_bringup'),
        'config',
        'params_4.yaml'
    )

    ###################################################################
    ##--------------------Robot description rviz2--------------------##
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
            ("robot_description", "own_description")
        ]
    )

    neighbor_description = ParameterValue(Command(['xacro ', urdf_path, ' id:=n', ' own:=false']),
                                            value_type=str)

    neighbor_robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="neighbor_robot_state_publisher",
        namespace=my_namespace,
        parameters=[{'robot_description': neighbor_description},
                    {'publish_frequency': 10.0}],
        remappings=[
            ("robot_description", "neighbor_description")
        ]
    )

    ###################################################################
    ##------------------------Mavros Launch--------------------------##
    ################################################################### 

    Mavros_launch = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(os.path.join(get_package_share_directory('asv_bringup'),
                         'launch/apm.launch.xml')),
        launch_arguments={
            'namespace': namespace_mavros 
        }.items()
    )

    #Mavros_launch = IncludeLaunchDescription(
    #    XMLLaunchDescriptionSource(
    #        os.path.join(get_package_share_directory('asv_bringup'),
    #                     'launch/apm.launch.xml')
    #    )
    #)

    ###################################################################
    ##--------------------Comunication Nodes-------------------------##
    ################################################################### 

    record = Node(
        package="asv_comunication",
        executable="bag_record",
        namespace= namespace_comunication,
        parameters = [
            {'my_id': LaunchConfiguration('my_id')}
        ]
    )

    rc_handler_node = Node (
        package= "asv_comunication",
        executable= "rc_handler",
        namespace= namespace_comunication,
        parameters = [
            {'my_id': my_namespace},
            config_gen]
    )

    ref_llc_node = Node (
        package= "asv_comunication",
        executable= "ref_llc",
        namespace= namespace_comunication,
        parameters = [
            {'my_id': my_namespace},
            config_gen]
    )

    ref_mlc_node = Node (
        package= "asv_comunication",
        executable= "ref_mlc",
        namespace= namespace_comunication,
        parameters = [
            {'my_id': my_namespace},
            config_gen]
    )

    apm_llc_node = Node (
        package= "asv_comunication",
        executable= "apm_llc",
        namespace= namespace_comunication,
        parameters = [
            {'my_id': my_namespace}
        ]
    )
    
    imu_fix_node = Node (
        package= "asv_comunication",
        executable= "imu_fix",
        namespace= namespace_comunication,
        parameters = [
            {'my_id': my_namespace}
        ]
    )

    imu_fix_ext_node = Node (
        package= "asv_comunication",
        executable= "imu_fix",
        name= "imu_fix_ext",
        namespace= namespace_comunication,
        remappings=[("mavros/imu/data", "comunication/imu_ext/data"),
                    ("control/accel_imu", "control/accel_imu_ext")],
        parameters = [
            {'my_id': my_namespace}
        ],
        condition=IfCondition(
            PythonExpression(
                [my_id, ' == 4']
            )
        )
    )

    imu_ext_node = Node (
        package= "asv_comunication",
        executable= "imu_driver.py",
        namespace= namespace_comunication,
        parameters = [
            {'my_id': my_namespace}
        ],
        condition=IfCondition(
            PythonExpression(
                [my_id, ' == 4']
            )
        )
    )

    transceiver_xbee_node = Node(
        package="asv_comunication",
        executable="transceiver_xbee.py",
        namespace= namespace_comunication,
        parameters = [
            {'my_id': my_namespace},
        ]
    )

    ###################################################################
    ##-----------------------Control Nodes---------------------------##
    ################################################################### 
  
    asv_tf_broadcast_node = Node (
        package= "asv_control",
        executable= "asv_tf2_broadcaster",
        namespace= namespace_control,
        parameters = [
            {'my_id': my_namespace}
        ]
    )

    pwm_mapper_node = Node (
        package= "asv_control",
        executable= "pwm_mapper",
        namespace= namespace_control,
        parameters = [
            {'my_id': my_namespace},
        ]
    )

    mux_llc_node = Node (
        package= "asv_control",
        executable= "mux_llc",
        namespace= namespace_control,
        parameters = [
            {'my_id': my_namespace},
        ]
    )

    ifac_llc_node0 = Node (
        package= "asv_control",
        executable= "ifac_llc",
        namespace= namespace_control,
        parameters = [{'my_id': my_namespace},
            config_0],
        condition=IfCondition(
            PythonExpression(
                [my_id, ' == 0']
            )
        )
    )
    
    ifac_llc_node1 = Node (
        package= "asv_control",
        executable= "ifac_llc",
        namespace= namespace_control,
        parameters = [{'my_id': my_namespace},
            config_1],
        condition=IfCondition(
            PythonExpression(
                [my_id, ' == 1']
            )
        )
    )

    ifac_llc_node3 = Node (
        package= "asv_control",
        executable= "ifac_llc",
        namespace= namespace_control,
        parameters = [{'my_id': my_namespace},
            config_3],
        condition=IfCondition(
            PythonExpression(
                [my_id, ' == 3']
            )
        )
    )
    
    ifac_llc_node4 = Node (
        package= "asv_control",
        executable= "ifac_llc",
        namespace= namespace_control,
        parameters = [{'my_id': my_namespace},
            config_4],
        condition=IfCondition(
            PythonExpression(
                [my_id, ' == 4']
            )
        )
    )

    mpc_llc_node0 = Node (
        package= "asv_control",
        executable= "mpc_llc",
        namespace= namespace_control,
        parameters = [{'my_id': my_namespace},
            config_0],
        condition=IfCondition(
            PythonExpression(
                [my_id, ' == 0']
            )
        )
    )
    
    mpc_llc_node1 = Node (
        package= "asv_control",
        executable= "mpc_llc",
        namespace= namespace_control,
        parameters = [{'my_id': my_namespace},
            config_1],
        condition=IfCondition(
            PythonExpression(
                [my_id, ' == 1']
            )
        )
    )
    
    mpc_llc_node3 = Node (
        package= "asv_control",
        executable= "mpc_llc",
        namespace= namespace_control,
        parameters = [{'my_id': my_namespace},
            config_3],
        condition=IfCondition(
            PythonExpression(
                [my_id, ' == 3']
            )
        )
    )
    
    mpc_llc_node4 = Node (
        package= "asv_control",
        executable= "mpc_llc",
        namespace= namespace_control,
        parameters = [{'my_id': my_namespace},
            config_4],
        condition=IfCondition(
            PythonExpression(
                [my_id, ' == 4']
            )
        )
    )

    wang_mlc_node0 = Node (
        package= "asv_control",
        executable= "wang_mlc",
        namespace= namespace_control,
        parameters = [{'my_id': my_namespace},
                config_0],
        condition=IfCondition(
            PythonExpression(
                [my_id, ' == 0']
            )
        )
    )

    wang_mlc_node1 = Node (
        package= "asv_control",
        executable= "wang_mlc",
        namespace= namespace_control,
        parameters = [{'my_id': my_namespace},
                config_1],
        condition=IfCondition(
            PythonExpression(
                [my_id, ' == 1']
            )
        )
    )
    
    wang_mlc_node3 = Node (
        package= "asv_control",
        executable= "wang_mlc",
        namespace= namespace_control,
        parameters = [{'my_id': my_namespace},
                config_3],
        condition=IfCondition(
            PythonExpression(
                [my_id, ' == 3']
            )
        )
    )
    
    wang_mlc_node4 = Node (
        package= "asv_control",
        executable= "wang_mlc",
        namespace= namespace_control,
        parameters = [{'my_id': my_namespace},
            config_4],
        condition=IfCondition(
            PythonExpression(
                [my_id, ' == 4']
            )
        )
    )

    ###################################################################
    ##-----------------------Observer Nodes--------------------------##
    ################################################################### 


    mux_obs_node = Node (
        package= "asv_observer",
        executable= "mux_obs",
        namespace= namespace_observer,
        parameters = [
            {'my_id': my_namespace},
        ]
    )
    
    observer_guille0 = Node (
        package= "asv_observer",
        executable= "observer_guille",
        name= "observer_guille",
        namespace= namespace_observer,
        parameters = [
            {'my_id': my_namespace},
            config_0
        ],
        condition=IfCondition(
            PythonExpression(
                [my_id, ' == 0']
            )
        )
    )

    observer_guille1 = Node (
        package= "asv_observer",
        executable= "observer_guille",
        name= "observer_guille",
        namespace= namespace_observer,
        parameters = [
            {'my_id': my_namespace},
            config_1
        ],
        condition=IfCondition(
            PythonExpression(
                [my_id, ' == 1']
            )
        )
    )

    observer_guille3 = Node (
        package= "asv_observer",
        executable= "observer_guille",
        name= "observer_guille",
        namespace= namespace_observer,
        parameters = [
            {'my_id': my_namespace},
            config_3
        ],
        condition=IfCondition(
            PythonExpression(
                [my_id, ' == 3']
            )
        )
    )
    
    observer_guille4 = Node (
        package= "asv_observer",
        executable= "observer_guille",
        name= "observer_guille",
        namespace= namespace_observer,
        parameters = [
            {'my_id': my_namespace},
            config_4
        ],
        condition=IfCondition(
            PythonExpression(
                [my_id, ' == 4']
            )
        )
    )

    observer_liu0 = Node (
        package= "asv_observer",
        executable= "observer_liu",
        name= "observer_liu",
        namespace= namespace_observer,
        parameters = [
            {'my_id': my_namespace},
            config_0
        ],
        condition=IfCondition(
            PythonExpression(
                [my_id, ' == 0']
            )
        )
    )

    observer_liu1 = Node (
        package= "asv_observer",
        executable= "observer_liu",
        name= "observer_liu",
        namespace= namespace_observer,
        parameters = [
            {'my_id': my_namespace},
            config_1
        ],
        condition=IfCondition(
            PythonExpression(
                [my_id, ' == 1']
            )
        )
    )
    
    observer_liu3 = Node (
        package= "asv_observer",
        executable= "observer_liu",
        name= "observer_liu",
        namespace= namespace_observer,
        parameters = [
            {'my_id': my_namespace},
            config_3
        ],
        condition=IfCondition(
            PythonExpression(
                [my_id, ' == 3']
            )
        )
    )
    
    observer_liu4 = Node (
        package= "asv_observer",
        executable= "observer_liu",
        name= "observer_liu",
        namespace= namespace_observer,
        parameters = [
            {'my_id': my_namespace},
            config_4
        ],
        condition=IfCondition(
            PythonExpression(
                [my_id, ' == 4']
            )
        )
    )

    observer_zono0 = Node (
        package= "asv_observer",
        executable= "observer_zono",
        name= "observer_zono",
        namespace= namespace_observer,
        parameters = [
            {'my_id': my_namespace},
            config_0
        ],
        condition=IfCondition(
            PythonExpression(
                [my_id, ' == 0']
            )
        )
    )

    observer_zono1 = Node (
        package= "asv_observer",
        executable= "observer_zono",
        name= "observer_zono",
        namespace= namespace_observer,
        parameters = [
            {'my_id': my_namespace},
            config_1
        ],
        condition=IfCondition(
            PythonExpression(
                [my_id, ' == 1']
            )
        )
    )

    observer_zono3 = Node (
        package= "asv_observer",
        executable= "observer_zono",
        name= "observer_zono",
        namespace= namespace_observer,
        parameters = [
            {'my_id': my_namespace},
            config_3
        ],
        condition=IfCondition(
            PythonExpression(
                [my_id, ' == 3']
            )
        )
    )

    observer_zono4 = Node (
        package= "asv_observer",
        executable= "observer_zono",
        name= "observer_zono",
        namespace= namespace_observer,
        parameters = [
            {'my_id': my_namespace},
            config_4
        ],
        condition=IfCondition(
            PythonExpression(
                [my_id, ' == 4']
            )
        )
    )

    ###################################################################
    ##-------------------------Mavros Nodes--------------------------##
    ################################################################### 
    # ld.add_action(Mavros_launch)
    # ld.add_action(own_robot_state_publisher_node)
    # ld.add_action(neighbor_robot_state_publisher_node)

    ###################################################################
    ##--------------------Comunication Nodes-------------------------##
    ################################################################### 
    ld.add_action(rc_handler_node)
    # ld.add_action(ref_llc_node)
    ld.add_action(ref_mlc_node)
    # ld.add_action(apm_llc_node)
    # ld.add_action(imu_fix_node)
    # ld.add_action(imu_ext_node)
    # ld.add_action(imu_fix_ext_node)
    # ld.add_action(record)
    # ld.add_action(transceiver_xbee_node)

    ###################################################################
    ##-----------------------Observer Nodes--------------------------##
    ################################################################### 
    ld.add_action(mux_obs_node)
    # ld.add_action(observer_guille0)
    # ld.add_action(observer_guille1)
    # ld.add_action(observer_guille3)
    # ld.add_action(observer_guille4)
    # ld.add_action(observer_liu0)
    # ld.add_action(observer_liu1)
    # ld.add_action(observer_liu3)
    # ld.add_action(observer_liu4)
    ld.add_action(observer_zono0)
    # ld.add_action(observer_zono1)
    # ld.add_action(observer_zono3)
    # ld.add_action(observer_zono4)

    ###################################################################
    ##-----------------------Control Nodes---------------------------##
    ################################################################### 
    ld.add_action(asv_tf_broadcast_node)
    ld.add_action(pwm_mapper_node)
    ld.add_action(mux_llc_node)
    ld.add_action(ifac_llc_node0)
    # ld.add_action(ifac_llc_node1)
    # ld.add_action(ifac_llc_node3)
    # ld.add_action(ifac_llc_node4)
    # ld.add_action(mpc_llc_node0)
    # ld.add_action(mpc_llc_node1)
    # ld.add_action(mpc_llc_node3)
    # ld.add_action(mpc_llc_node4)
    ld.add_action(wang_mlc_node0)
    # ld.add_action(wang_mlc_node1)
    # ld.add_action(wang_mlc_node3)
    # ld.add_action(wang_mlc_node4)

    return ld