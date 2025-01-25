from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('dev', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('baud', default_value='115200'),
        DeclareLaunchArgument('address', default_value='128'),
        DeclareLaunchArgument('max_speed', default_value='2.0'),
        DeclareLaunchArgument('ticks_per_meter', default_value='4342.2'),
        DeclareLaunchArgument('base_width', default_value='0.315'),
        DeclareLaunchArgument('run_diag', default_value='true'),

        Node(
            package='roboclaw_node',
            executable='roboclaw_node',
            name='roboclaw_node',
            parameters=[{
                'dev': LaunchConfiguration('dev'),
                'baud': LaunchConfiguration('baud'),
                'address': LaunchConfiguration('address'),
                'max_speed': LaunchConfiguration('max_speed'),
                'ticks_per_meter': LaunchConfiguration('ticks_per_meter'),
                'base_width': LaunchConfiguration('base_width'),
            }],
            condition=IfCondition(LaunchConfiguration('run_diag'))
        ),

        Node(
            package='diagnostic_aggregator',
            executable='aggregator_node',
            name='diagnostic_aggregator',
            parameters=[{
                'analyzers': {
                    'roboclaw': {
                        'type': 'diagnostic_aggregator/GenericAnalyzer',
                        'path': 'RoboClaw',
                        'timeout': 5.0,
                        'num_items': 0
                    }
                }
            }]
        ),
    ])