from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sim_mdrs',
            executable='remote_messages',
            name='Remote_control_node'
        ),
        Node(
            package='sim_mdrs',
            executable='diff_controller',
            name='Differential_Controller_Node'
        ),
        Node(
            package='sim_mdrs',
            executable='roboclaw_node',
            name='Roboclaw_control_node'
        )
    ])
