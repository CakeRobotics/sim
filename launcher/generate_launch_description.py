import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros

from .config import collect_config
from .find_default_asset_path import find_robot_path, find_world_path


def generate_launch_description():
    config = collect_config()
    if config['world'][0] == '/':
        world_file = config['world']
    else:
        world_file = find_world_path(config['world'])
    urdf_file = find_robot_path(config['robot'])
    initial_pose = config['start']

    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}]
    )
    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        condition=launch.conditions.UnlessCondition(LaunchConfiguration('gui'))
    )
    spawn_entity = launch_ros.actions.Node(
    	package='gazebo_ros',
    	executable='spawn_entity.py',
        arguments=['-entity', 'sam_bot', '-topic', 'robot_description',
            '-x', str(initial_pose['x']),
            '-y', str(initial_pose['y']),
            '-z', str(initial_pose['z']),
            '-R', str(initial_pose['R']),
            '-P', str(initial_pose['P']),
            '-Y', str(initial_pose['Y']),
        ],
        output='screen'
    )

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='gui',
            default_value='True',
            description='Flag to enable joint_state_publisher_gui'
        ),
        launch.actions.DeclareLaunchArgument(
            name='model',
            default_value=urdf_file,
            description='Absolute path to robot urdf file'
        ),
        launch.actions.DeclareLaunchArgument(
            name='use_sim_time',
            default_value='True',
            description='Flag to enable use_sim_time'
        ),
        launch.actions.ExecuteProcess(
            cmd=[
                'gzserver',
                '--pause',
                '--verbose',
                '-s', 'libgazebo_ros_init.so',
                '-s', 'libgazebo_ros_factory.so',
                world_file
            ],
            output='screen'
        ),
        joint_state_publisher_node,
        robot_state_publisher_node,
        spawn_entity,
    ])
