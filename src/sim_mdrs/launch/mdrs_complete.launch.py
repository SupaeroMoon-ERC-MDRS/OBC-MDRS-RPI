import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

import xacro


def generate_launch_description():

    #################### Gazebo ##########################
    '''
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
             )
    
    osr_urdf_path = os.path.join(
        get_package_share_directory('sim_mdrs'))
    
    xacro_file = os.path.join(osr_urdf_path,
                              'urdf',
                              'osr.urdf.xacro')
    
    try:
        doc = xacro.parse(open(xacro_file))
        xacro.process_doc(doc)
        params = {'robot_description': doc.toxml()}
    except Exception as e:
        print(f"Error processing xacro file: {e}")
        return LaunchDescription([])
    '''
    #################### Nodes ##########################

    # node_robot_state_publisher = Node(
    #     package='robot_state_publisher',
    #     executable='robot_state_publisher',
    #     output='screen',
    #     parameters=[params]
    # )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
    )

    remote_arm = Node(
        package='sim_mdrs',
        executable='remote_arm',
        output='screen'
    )

    controller_spawn = Node(
        package='sim_mdrs',
        executable='diff_controller',
        output='screen'
    )

    # differential drive node
    diff_controller = Node(
        package='sim_mdrs',
        executable='diff_controller',
        output='screen'
    )

    # arm configuration node
    remote_messages = Node(
        package='sim_mdrs',
        executable='remote_messages',
        output='screen'
    )

    # gripper control node
    gripper_control = Node(
        package='sim_mdrs',
        executable='gripper',
        output='screen'
    )

    # joint angle node
    joint2motor = Node(
        package='sim_mdrs',
        executable='jointangle_to_motor',
        output='screen'
    )

    # roboclaw node
    roboclaw_controller = Node(
        package='sim_mdrs',
        executable='roboclaw_node',
        output='screen'
    )

    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'roo'],
                        output='screen')

    # TODO telemetry node
    # telemetry_node = Node(
    #     package='sim_mdrs',
    #     executable='telemetry_node',
    #     output='screen'
    # )

    #################### Processes ##########################
    
    # joint_state_controller
    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_state_broadcaster'],
        output='screen'
    )
    
    load_arm_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'arm_controller'],
        output='screen'
    )

    # wheel_velocity_controller
    rover_wheel_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'wheel_controller'],
        output='screen'
    )

    # servo_controller
    servo_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'servo_controller'],
        output='screen'
    )

    return LaunchDescription([
        # gazebo,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[
                    load_joint_state_controller,
                    load_arm_state_controller,
                    rover_wheel_controller,
                    servo_controller,
                ],
            )
        ),
        controller_spawn,
        robot_state_publisher_node,
        # node_robot_state_publisher,
        # spawn_entity,
        remote_messages,
        remote_arm,
        diff_controller,
        roboclaw_controller,
        joint2motor,
        gripper_control
        # TODO telemetry
    ])