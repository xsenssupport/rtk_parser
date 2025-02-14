from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'debug',
            default_value='false',
            description='Enable debug output'
        ),
        
        Node(
            package='rtk_parser',
            executable='rtk_parser_node',
            name='rtk_parser_node',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'debug': LaunchConfiguration('debug')
            }]
        )
    ])