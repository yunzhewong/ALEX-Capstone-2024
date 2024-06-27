from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import xacro
import os


def generate_launch_description():
    doc = xacro.process_file("../urdf/twinMotor.urdf.xacro")
    robot_desc = doc.toxml()
    robot_description = {"robot_description": robot_desc}

    gazebo = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]
         ),launch_arguments={'extra_gazebo_args': '--ros-args --params-file ../config/gazebo_params.yaml' }.items()
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

    load_motor1 = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'bottom_motor_controller'],
        output='screen'
    )
    
    load_motor2 = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'top_motor_controller'],
        output='screen'
    )

    motor_controller = Node(
        package='alex_nodes',
        executable='motor_pubsub',
        name="motor_pubsub"
    )

    encoder_reader = Node(
        package='alex_nodes_cpp',
        executable='encoder_reader',
        name="encoder_reader"
    )

    user_interface = Node(
        package="alex_nodes",
        executable='command_redirector',
        name="command_redirector"
    )

    return LaunchDescription(
        [
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=spawn_entity,
                    on_exit=[load_joint_state_broadcaster, load_motor1, load_motor2],
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
