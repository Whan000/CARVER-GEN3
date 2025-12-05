from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    uros_interface = Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            name='carver_interface_agent',
            output='screen',
            arguments=[
                'serial',
                '-b', '2000000',
                '--dev', '/dev/carver_interface'
            ]
        )
    
    steering_node = Node(
            package='carver_manager',
            executable='carver_manual_steering.py',
            name='carver_manual_steering',
            output='screen',
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

    return LaunchDescription([
        uros_interface,
        motor_node,
        steering_node,
    ])