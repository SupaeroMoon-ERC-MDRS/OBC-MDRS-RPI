from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sim_mdrs',
            executable='remote_messages',
            name='talker_node'
        ),
        Node(
            package='sim_mdrs',
            executable='diff_controller',
            name='listener_node'
        ),
        Node(
            package='sim_mdrs',
            executable='roboclaw_node',
            name='listener_node'
        )
    ])
