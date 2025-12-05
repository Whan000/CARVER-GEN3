#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.actions import TimerAction

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    desc_pkg = get_package_share_directory('carver_description')
    sim_pkg = get_package_share_directory('carver_simulation')

    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GAZEBO_MODEL_PATH'] += os.pathsep + os.path.join(desc_pkg, 'share')
    else:
        os.environ['GAZEBO_MODEL_PATH'] = os.path.join(desc_pkg, 'share')

    world_file = os.path.join(sim_pkg, 'worlds', 'custom_world.sdf')

    xacro_file = PathJoinSubstitution([FindPackageShare('carver_description'), 'urdf', 'carver_core.xacro'])
    robot_description_content = Command([
        FindExecutable(name='xacro'), ' ', xacro_file,
        ' use_sim_time:=', use_sim_time
    ])
    robot_description = {'robot_description': robot_description_content}

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]
        ),
        launch_arguments={'gz_args': f'-r -v 1 {world_file}'}.items()
    )

    state_pub = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        name='robot_state_publisher', output='screen',
        parameters=[robot_description]
    )

    spawn_entity = Node(
        package='ros_gz_sim', executable='create', name='spawn_entity', output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'carver',
            '-allow_renaming', 'true',
            '-x', '-3.0', '-y', '0.0', '-z', '0.0'
        ]
    )

    jsb_spawner = Node(
        package='controller_manager', executable='spawner', name='spawner_joint_state_broadcaster',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
    )

    steering_spawner = Node(
        package='controller_manager', executable='spawner', name='spawner_steering_controller',
        arguments=['steering_controller', '--controller-manager', '/controller_manager'],
    )

    wheel_spawner = Node(
        package='controller_manager', executable='spawner', name='spawner_wheel_velocity_controller',
        arguments=['wheel_velocity_controller', '--controller-manager', '/controller_manager'],
    )

    bridge = Node(
        package='ros_gz_bridge', 
        executable='parameter_bridge', 
        output='screen',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            '/lidar/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked',
            '/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU',
            '/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V'
        ]
    )
    rviz_config = os.path.join(sim_pkg, 'rviz', 'display.rviz')
    rviz_delayed = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='rviz2', executable='rviz2', name='rviz', output='screen',
                arguments=['-d', rviz_config],
                parameters=[{'use_sim_time': use_sim_time}]
            )
        ]
    )

    teleop_xterm = ExecuteProcess(
        cmd=['xterm', '-e', 'ros2', 'run', 'carver_simulation', 'simulation_teleop.py'],
        name='carver_teleop_xterm',
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use simulation (Gazebo) clock'
        ),
        gz_sim,
        state_pub,
        spawn_entity,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[jsb_spawner]
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=jsb_spawner,
                on_exit=[steering_spawner]
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=steering_spawner,
                on_exit=[wheel_spawner]
            )
        ),
        bridge,
        rviz_delayed,

        teleop_xterm
    ])