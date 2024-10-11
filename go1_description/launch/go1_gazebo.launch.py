#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

package_name = 'go1_description'
world_file = 'empty.sdf'

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # Пакет с Gazebo Ignition
    pkg_ros_ign_gazebo = get_package_share_directory('ros_ign_gazebo')
    
    # Пакет с описанием робота
    pkg_simulation = get_package_share_directory(package_name)
    
    # Путь к файлу Xacro
    xacro_path = os.path.join(pkg_simulation, 'urdf', 'go1.urdf.xacro')

    # Генерация описания робота
    robot_description_content = Command(['xacro ', xacro_path])
    robot_description = {"robot_description": robot_description_content, "use_sim_time": use_sim_time}
    
    # Путь к файлу мира для Gazebo Ignition
    world_path = os.path.join(pkg_simulation, 'world', world_file)

    # Запуск Gazebo Ignition
    start_ignition_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_ros_ign_gazebo, 'launch', 'ign_gazebo.launch.py')),
        launch_arguments={'ign_args': '-r ' + world_path}.items()
    )

    # Спавн робота в симуляции
    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-string", robot_description_content,
            "-name", "go1_description",
            "-allow_renaming", "true",
            "-z", "0.35",  # Указываем высоту спавна
        ],
        output="screen",
    )

    # Публикация состояния робота
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # Запуск контроллеров
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    spawn_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_group_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    # Ноды для управления роботом
    controller = Node(
        package='go1_controller',
        executable='robot_controller_gazebo.py',
        name='quadruped_controller',
        output='screen',
    )

    imu_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="ign_imu",
        arguments=["/imu_plugin/out@sensor_msgs/msg/Imu@ignition.msgs.IMU"]
    )

    cmd_vel_pub = Node(
        package='go1_controller',
        executable='cmd_vel_pub.py',
        name='cmd_vel_pub',
        output='screen',
    )

    return LaunchDescription([
        start_ignition_cmd,
        robot_state_publisher_node,
        spawn_entity,
        joint_state_broadcaster_spawner,
        spawn_controller,
        controller,
        imu_bridge,
        cmd_vel_pub
    ])
