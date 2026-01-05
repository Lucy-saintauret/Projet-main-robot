import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Paramètres
    use_sim_time = False
    
    # Chemins des fichiers
    pkg_share = get_package_share_directory('hand_description')
    xacro_file = os.path.join(pkg_share, 'urdf', 'hand.urdf.xacro')
    rviz_config = os.path.join(pkg_share, 'rviz', 'config.rviz')
    controllers_config = os.path.join(pkg_share, 'config', 'controllers.yaml')
    # world_file = os.path.join(pkg_share, 'worlds', 'luna_world.sdf')
    
    # Traiter le fichier xacro pour générer l'URDF
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file]),
        value_type=str
    )

    # Générer le fichier URDF temporaire avec résolution des chemins package://
    urdf_file = '/tmp/hand.urdf'
    stl_source = os.path.join(pkg_share, 'meshes', 'middle_base_left.stl')
    stl_dest = '/tmp/middle_base_left.stl'
    
    generate_urdf = ExecuteProcess(
        cmd=['bash', '-c', f'cp "{stl_source}" "{stl_dest}" && xacro {xacro_file} | sed "s|package://hand_description/meshes/index_base_left.stl|file://{stl_dest}|g" > {urdf_file}'],
        output='screen'
    )

    # Lancement de Gazebo Sim avec le monde luna_world
    # gazebo_server = ExecuteProcess(
    #     cmd=['gz', 'sim', '-r', world_file],
    #     output='screen'
    # )

    # Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time
        }]
    )

    # Controller Manager
    # controller_manager_node = Node(
    #     package='controller_manager',
    #     executable='ros2_control_node',
    #     parameters=[
    #         {'robot_description': robot_description},
    #         controllers_config
    #     ],
    #     output='screen'
    # )

    # Joint State Broadcaster Spawner
    # joint_state_broadcaster_spawner = Node(
    #     package='controller_manager',
    #     executable='spawner',
    #     arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
    #     output='screen'
    # )

    # Servo Controller Spawner
    # servo_controller_spawner = Node(
    #     package='controller_manager',
    #     executable='spawner',
    #     arguments=['servo_controller', '--controller-manager', '/controller_manager'],
    #     output='screen'
    # )

    # Joint State Publisher GUI (pour contrôler les joints du bras)
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )
    
    # RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config]

    )


    # # Spawn du robot complet avec gz service
    # spawn_robot = ExecuteProcess(
    #     cmd=['bash', '-c', f'sleep 5 && gz service -s /world/luna_world/create --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean --timeout 1000 --req "sdf_filename: \\"{urdf_file}\\" name: \\"luna\\" pose: {{position: {{x: 0 y: 0 z: 0.2}}}}"'],
    #     output='screen'
    # )

    # bridge = Node(
    # package='ros_gz_bridge',
    # executable='parameter_bridge',
    # arguments=[
    #     '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
    #     '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
    # ],
    # output='screen'
    # )


    return LaunchDescription([
        generate_urdf,
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        # controller_manager_node,
        # joint_state_broadcaster_spawner,
        # servo_controller_spawner,
        rviz_node
    ])
