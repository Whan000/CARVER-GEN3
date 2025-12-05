import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    carver_description_pkg = get_package_share_directory('carver_description')
    carver_bringup_pkg = get_package_share_directory('carver_bringup')
    # livox_pkg = get_package_share_directory('livox_ros_driver2')
    
    uros_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    carver_bringup_pkg,
                    "launch",
                    "uros.launch.py"
                )
            ]
        )
    )

    # livox_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         [
    #             os.path.join(
    #                 livox_pkg,
    #                 "launch_ROS2",
    #                 "rviz_MID360_launch.py"
    #             )
    #         ]
    #     )
    # )

    bno055_converter = Node(
            package='bno055_imu',
            executable='bno055_imu.py',
            name='bno055_imu',
            output='screen'
        )

    dummy_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    carver_description_pkg,
                    "launch",
                    "carver.launch.py"
                )
            ]
        )
    )

    motor_node = Node(
            package='carver_odrive',
            executable='carver_odrive.py',
            name='carver_odrive_node',
            output='screen',
            parameters=[
                {'control_mode': 'velocity'},
            ]
    )

    mode_node = Node(
            package='carver_manager',
            executable='carver_mode.py',
            name='carver_mode_node',
            output='screen',
    )

    steering_node = Node(
            package='carver_manager',
            executable='carver_manual_steering.py',
            name='carver_manual_steering',
            output='screen',
        )

    static_map_to_odom = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_map_to_odom',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
        )
    
    static_odom_to_base_link = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_odom_to_base_link',
            arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link']
        )

    return LaunchDescription([
        uros_launch,
        # livox_launch,
        bno055_converter,
        dummy_robot,
        motor_node,
        mode_node,
        steering_node,
        static_map_to_odom,
        static_odom_to_base_link,
    ])