from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
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
            }],
            output='screen'
        ),
    ])