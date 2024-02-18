from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import xacro
import os

def generate_launch_description():
    doc = xacro.process_file('../urdf/model.urdf.xacro')
    robot_desc = doc.toxml()
    params = {'robot_description': robot_desc, "use_sim_time": True}

    robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',parameters=[params]
        )
    
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
        )
    
    spawn_entity = Node(package='gazebo_ros', executable="spawn_entity.py", arguments=["-topic", "robot_description", '-entity', 'my_bot'], output="screen")

    return LaunchDescription([
        robot_state_publisher, gazebo, spawn_entity
    ])
