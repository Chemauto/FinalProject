import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from xacro import process_file


def generate_launch_description():
    # 获取包路径
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_go2 = get_package_share_directory('go2_gazebo_description')

    # 声明启动参数
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')

    # Gazebo 启动文件
    gazebo_launch_file = PathJoinSubstitution([
        pkg_gazebo_ros,
        'launch',
        'gazebo.launch.py'
    ])

    # Robot State Publisher
    # Process xacro file
    xacro_file = os.path.join(pkg_go2, 'urdf', 'go2.urdf.xacro')
    robot_description = process_file(xacro_file).toxml()

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description
        }]
    )

    # Spawn robot in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'go2',
            '-topic', '/robot_description',
            '-x', x_pose,
            '-y', y_pose,
            '-z', '0.3'
        ],
        output='screen'
    )

    # Joint State Broadcaster
    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )

    # Diff Drive Controller
    diff_drive_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller'],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument(
            'x_pose',
            default_value='0.0',
            description='X position of robot spawn'),
        DeclareLaunchArgument(
            'y_pose',
            default_value='0.0',
            description='Y position of robot spawn'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([gazebo_launch_file]),
            launch_arguments={'use_sim_time': use_sim_time}.items()
        ),

        robot_state_publisher,
        spawn_entity,
        # joint_state_broadcaster,  # Not needed for diff_drive plugin
        # diff_drive_spawner,  # Not needed for diff_drive plugin
    ])
