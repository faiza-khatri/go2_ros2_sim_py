import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

import xacro


def generate_launch_description():

    # Check if we're told to use sim time
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_ros2_control = LaunchConfiguration('use_ros2_control')
    name = 'robot1'
    namespace = '/robot1'

    # Process the URDF file
    pkg_path = os.path.join(get_package_share_directory('go1_description'))
    xacro_file = os.path.join(pkg_path,'xacro','robot.xacro')

    robot_desc = xacro.process_file(
            xacro_file,
            mappings={'robot_name': name}
        ).toxml() 
    # Create a robot_state_publisher node
    params = {'robot_description': robot_desc, 'use_sim_time': use_sim_time}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        namespace=namespace,
        parameters=[params],
        remappings=[
            ("/tf", f"{namespace}/tf"),
            ("/tf_static", f"{namespace}/tf_static"),
        ],
    )


    # Launch!
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),
        node_robot_state_publisher
    ])
