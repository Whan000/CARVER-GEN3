import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    carver_description_pkg = get_package_share_directory('carver_description')
    carver_bringup_pkg = get_package_share_directory('carver_bringup')
    fast_lio_pkg = get_package_share_directory('fast_lio')
    livox_pkg = get_package_share_directory('livox_ros_driver2')
    
    uros_bno055 = Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            name='carver_bno055_agent',
            output='screen',
            arguments=[
                'serial',
                '-b', '2000000',
                '--dev', '/dev/carver_bno055'
            ]
        )

    livox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    livox_pkg,
                    "launch_ROS2",
                    "msg_MID360_launch.py"
                )
            ]
        )
    )

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

    manual_steering = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    carver_bringup_pkg,
                    "launch",
                    "manual_steering.launch.py"
                )
            ]
        )
    )

    lio_sam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    fast_lio_pkg,
                    "launch",
                    "mapping.launch.py"
                )
            ]
        )
    )

    delayed_lio_sam = TimerAction(
        period=10.0,
        actions=[lio_sam]
    )

    return LaunchDescription([
        uros_bno055,
        livox_launch,
        bno055_converter,
        dummy_robot,
        manual_steering,
        # delayed_lio_sam
    ])