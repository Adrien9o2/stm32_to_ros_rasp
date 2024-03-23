from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='holonomic_twist_keyboard',
            namespace='robotbebou',
            executable='holonomic_twist_keyboard',
            output='screen',
            prefix = 'xterm -e',
            remappings=[
                ('cmd_vel', 'cmd/robot_speed_m_s_and_rad_s'),
            ]
        )
    ])
