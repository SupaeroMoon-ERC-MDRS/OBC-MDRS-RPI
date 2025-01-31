from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.event_handlers import OnProcessStart

import xacro
import os

def generate_launch_description():
    # Load robot description from URDF/Xacro
    pkg_share = os.path.join(get_package_share_directory('sim_mdrs'))
    xacro_file = os.path.join(pkg_share, 'urdf', 'arm_rviz.urdf.xacro')

    try:
        doc = xacro.parse(open(xacro_file))
        xacro.process_doc(doc)
        params = {'robot_description': doc.toxml()}
    except Exception as e:
        print(f"Error processing xacro file: {e}")
        return LaunchDescription([])

    # Define robot state publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    # Joint state publisher (if needed)
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
    )
    
    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_state_broadcaster'],
        output='screen'
    )

    load_arm_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'arm_controller'],
        output='screen'
    )

    # Launch RViz with a custom configuration file
    # rviz_config_file = os.path.join(pkg_share, 'rviz', 'arm_config.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
        # arguments=['-d', rviz_config_file]
    )

    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_node,
        RegisterEventHandler(
            OnProcessStart(
                target_action=robot_state_publisher_node,
                on_start=[load_joint_state_controller, load_arm_state_controller],
            )
        ),
        rviz_node
    ])

