from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
import xacro
import os


def generate_launch_description():
    # Declare the robot description file location
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
             )
    
    pkg_share = os.path.join(
        get_package_share_directory('sim_mdrs'))
    
    xacro_file = os.path.join(pkg_share,
                              'urdf',
                              'arm.urdf.xacro')
    
    try:
        doc = xacro.parse(open(xacro_file))
        xacro.process_doc(doc)
        params = {'robot_description': doc.toxml()}
    except Exception as e:
        print(f"Error processing xacro file: {e}")
        return LaunchDescription([])
    
    
    # Define the parameters to pass to the robot state publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    remote_control = Node(
        package='sim_mdrs',
        executable='remote_arm',
        output='screen'
    )

    # # Add a joint state publisher node
    # joint_state_publisher_node = Node(
    #     package='joint_state_publisher',
    #     executable='joint_state_publisher',
    #     name='joint_state_publisher',
    #     output='screen',
    # )

    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description', '-entity', 'arm', '-x', '0', '-y', '0', '-z', '0'],
                        output='screen')
    

    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_state_broadcaster'],
        output='screen'
    )

    load_arm_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'arm_controller'],
        output='screen'
    )

    # load_gripper_state_controller = ExecuteProcess(
    #     cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'gripper_controller'],
    #     output='screen'
    # )

    return LaunchDescription([
        gazebo,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[
                    load_joint_state_controller,
                    load_arm_state_controller,
                    # load_gripper_state_controller,
                ],
            )
        ),
        robot_state_publisher_node,
        spawn_entity,
        remote_control
    ])