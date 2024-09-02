from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument
import os
import yaml
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()

    # Declare launch arguments
    ld.add_action(DeclareLaunchArgument(
        'num_agents',
        default_value='1',
        description='Number of agents'
    ))

    ld.add_action(DeclareLaunchArgument(
        'kb_teleop',
        default_value='false',
        description='Enable keyboard teleoperation'
    ))

    # Load configuration
    config = os.path.join(
        get_package_share_directory('f1tenth_gym_ros'),
        'config',
        'sim.yaml'
    )
    config_dict = yaml.safe_load(open(config, 'r'))
    has_opp = config_dict['bridge']['ros__parameters']['num_agent'] > 1
    teleop = config_dict['bridge']['ros__parameters']['kb_teleop']

    # Define nodes
    bridge_node = Node(
        package='f1tenth_gym_ros',
        executable='gym_bridge',
        name='bridge',
        parameters=[config],
        output='screen'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', os.path.join(get_package_share_directory('f1tenth_gym_ros'), 'launch', 'gym_bridge.rviz')],
        output='screen'
    )

    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        parameters=[{
            'yaml_filename': config_dict['bridge']['ros__parameters']['map_path'] + '.yaml',
            'topic': 'map',
            'frame_id': 'map',
            'output': 'screen',
            'use_sim_time': True
        }],
        output='screen'
    )

    nav_lifecycle_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'node_names': ['map_server']
        }],
        output='screen'
    )

    ego_robot_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='ego_robot_state_publisher',
        parameters=[{
            'robot_description': Command(['xacro ', os.path.join(get_package_share_directory('f1tenth_gym_ros'), 'launch', 'ego_racecar.xacro')])
        }],
        remappings=[('/robot_description', 'ego_robot_description')],
        output='screen'
    )

    opp_robot_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='opp_robot_state_publisher',
        parameters=[{
            'robot_description': Command(['xacro ', os.path.join(get_package_share_directory('f1tenth_gym_ros'), 'launch', 'opp_racecar.xacro')])
        }],
        remappings=[('/robot_description', 'opp_robot_description')],
        output='screen'
    )
    
    stanley_controller_node = Node(
    	package='stanley_controller',
    	executable='stanley_controller_node',
        name='stanley_controller',
        parameters=[{
        }],
        output='screen'
    )

    # Add nodes to launch description
    ld.add_action(bridge_node)
    ld.add_action(rviz_node)
    ld.add_action(map_server_node)
    ld.add_action(nav_lifecycle_node)
    ld.add_action(ego_robot_publisher)
    ld.add_action(stanley_controller_node)
    if has_opp:
        ld.add_action(opp_robot_publisher)

    return ld
