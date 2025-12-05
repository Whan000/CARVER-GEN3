from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            name='carver_interface_agent',
            output='screen',
            arguments=[
                'serial',
                '-b', '2000000',
                '--dev', '/dev/ttyACM1'
            ]
        ),
        
        Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            name='carver_steering_agent',
            output='screen',
            arguments=[
                'serial',
                '-b', '2000000',
                '--dev', '/dev/ttyACM2'
            ]
        ),
        
        Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            name='carver_bno055_agent',
            output='screen',
            arguments=[
                'serial',
                '-b', '2000000',
                '--dev', '/dev/ttyACM0'
            ]
        ),
    ])