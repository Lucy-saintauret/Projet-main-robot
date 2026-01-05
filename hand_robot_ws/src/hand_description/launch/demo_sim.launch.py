import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, IncludeLaunchDescription, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Paramètres
    use_sim_time = True
    
    # Chemins des fichiers
    pkg_share = get_package_share_directory('hand_description')
    xacro_file = os.path.join(pkg_share, 'urdf', 'hand.xacro')
    rviz_config = os.path.join(pkg_share, 'rviz', 'config.rviz')
    controllers_config = os.path.join(pkg_share, 'config', 'controllers.yaml')
    
    # Configurer les chemins de ressources Gazebo pour trouver les meshes
    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.path.join(pkg_share, 'meshes') + ':' + 
              os.path.dirname(pkg_share)
    )
    
    # Traiter le fichier xacro pour générer l'URDF
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file]),
        value_type=str
    )

    # Lancement de Gazebo
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ]),
        launch_arguments={'gz_args': '-r -v 4 empty.sdf --physics-engine gz-physics-bullet-featherstone-plugin'}.items()
    )

    # Spawn du robot dans Gazebo
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'hand_robot',
            '-topic', 'robot_description',
            '-z', '0'
        ],
        output='screen'
    )

    # Bridge ROS-Gazebo pour les joints
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
        ],
        output='screen'
    )

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

    # Joint State Publisher GUI (pour contrôler les joints du bras)
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        gz_resource_path,
        gazebo_launch,
        robot_state_publisher_node,
        spawn_robot,
        bridge,
        joint_state_publisher_gui_node,
        rviz_node
    ])
