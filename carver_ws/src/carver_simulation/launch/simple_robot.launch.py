#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, ExecuteProcess
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    desc_pkg = get_package_share_directory('carver_description')
    sim_pkg = get_package_share_directory('carver_simulation')

    xacro_file = PathJoinSubstitution([
        FindPackageShare('carver_description'), 
        'urdf', 
        'simple_robot_velodyne.xacro'
    ])
    
    robot_description_content = Command([
        FindExecutable(name='xacro'), ' ', xacro_file,
        ' use_sim_time:=', use_sim_time
    ])
    robot_description = {'robot_description': robot_description_content}

    world_file = os.path.join(sim_pkg, 'worlds', 'custom_world_classic.world')
    
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]
        ),
        launch_arguments={'world': world_file, 'verbose': 'true'}.items()
    )

    state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': use_sim_time}]
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_entity',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'simple_robot',
            '-x', '-3.0',
            '-y', '0.0',
            '-z', '0.2'
        ]
    )

    jsb_spawner = Node(
        package='controller_manager',
        executable='spawner',
        name='spawner_joint_state_broadcaster',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
    )

    steering_spawner = Node(
        package='controller_manager',
        executable='spawner',
        name='spawner_steering_controller',
        arguments=['steering_controller', '--controller-manager', '/controller_manager'],
    )

    wheel_spawner = Node(
        package='controller_manager',
        executable='spawner',
        name='spawner_wheel_velocity_controller',
        arguments=['wheel_velocity_controller', '--controller-manager', '/controller_manager'],
    )

    rviz_config = os.path.join(sim_pkg, 'rviz', 'display_classic.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    odom_tf = Node(
        package='carver_simulation',
        executable='odom_tf_relay.py',
        name='odom_tf_relay',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    teleop_xterm = ExecuteProcess(
        cmd=['xterm', '-e', 'ros2', 'run', 'carver_simulation', 'simulation_teleop.py', '--ros-args', '-p', 'mode:=classic'],
        name='simple_robot_teleop_xterm',
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock'
        ),
        gazebo,
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
        rviz,
        odom_tf,
        teleop_xterm
    ])