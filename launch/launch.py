from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetLaunchConfiguration
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
import xacro

def generate_launch_description():
    doc = xacro.process_file('../urdf/model.urdf.xacro')
    robot_desc = doc.toprettyxml(indent='  ')
    params = {'robot_description': robot_desc}

    return LaunchDescription([
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher'
        ),
        # Launch robot_state_publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',parameters=[params]
        ),
        # Launch rviz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            arguments=['-d', '../config/rviz.rviz'],
            output='screen'
        )
    ])
