from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import xacro
import os


def generate_launch_description():
    doc = xacro.process_file("../urdf/justMotor.urdf.xacro")
    robot_desc = doc.toxml()
    robot_description = {"robot_description": robot_desc}
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(get_package_share_directory("gazebo_ros"), "launch"),
                "/gazebo.launch.py",
            ]
        ),
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[robot_description],
    )

    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-topic", "robot_description", "-entity", "my_bot"],
        output="screen",
    )

    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    load_joint_trajectory_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'propellor_controller'],
        output='screen'
    )

    motor_controller = Node(
        package='alex_nodes',
        executable='motor_pubsub',
        name="motor_pubsub"
    )

    encoder_reader = Node(
        package='alex_nodes',
        executable='encoder_reader',
        name="encoder_reader"
    )

    user_interface = Node(
        package="alex_nodes",
        executable='motor_user_interface',
        name="motor_user_interface"
    )

    return LaunchDescription(
        [
            RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[load_joint_state_broadcaster],
            )
            ),
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=load_joint_state_broadcaster,
                    on_exit=[load_joint_trajectory_controller],
                )
            ),
            robot_state_publisher,
            gazebo,
            spawn_entity, 
            motor_controller, 
            encoder_reader, 
            user_interface
        ]
    )
