#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os
import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
import xacro

from launch.conditions import IfCondition

package_name = 'go1_description'
world_file = 'empty.sdf'

def generate_launch_description():
    # Параметры для конфигурации
    use_sim_time = LaunchConfiguration("use_sim_time")
    namespace = '/robot1'
    name = 'robot1'

    # Аргументы запуска
    declare_namespace = DeclareLaunchArgument(
        'namespace',
        default_value='robot1',
        description='Namespace для запуска робота'
    )
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Использовать ли симулированное время'
    )
    use_localization = LaunchConfiguration("use_localization")

    # Пакет с Gazebo Ignition
    pkg_ros_ign_gazebo = get_package_share_directory('ros_ign_gazebo')
    
    # Пакет с описанием робота
    pkg_simulation = get_package_share_directory(package_name)
    
    # Путь к файлу Xacro
    xacro_path = os.path.join(pkg_simulation, 'xacro', 'robot.xacro')

    # Генерация описания робота
    robot_description_content = Command(['xacro ', xacro_path])

    robot_desc = xacro.process_file(
            xacro_path,
            mappings={'robot_name': name}
        ).toxml()   
    params = {'robot_description': robot_desc }
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
        namespace=namespace,
        arguments=[
            "-string", robot_description_content,
            "topic", f"{namespace}/go1_description",
            "-name", f"{namespace}/go1_description",
            "-allow_renaming", "true",
            "-z", "0.45",  # Указываем высоту спавна
        ],
        output="screen",
    )

    # Публикация состояния робота
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        namespace=namespace,
        parameters=[params],
        remappings=[
            ("/tf", f"{namespace}/tf"),
            ("/tf_static", f"{namespace}/tf_static"),
        ],
    )

    # Запуск контроллеров
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace=namespace,
        arguments=["joint_state_broadcaster", "--controller-manager", f"{namespace}/controller_manager"],
    )

    spawn_controller = Node(
        package="controller_manager",
        executable="spawner",
        namespace=namespace,
        arguments=["joint_group_controller", "--controller-manager", f"{namespace}/controller_manager"],
        output="screen",
    )

    # Ноды для управления роботом
    controller = Node(
        package='go1_controller',
        executable='robot_controller_gazebo.py',
        name='quadruped_controller',
        namespace=namespace,
        output='screen',
    )
    odom = Node(
        package='go1_controller',
        executable='QuadrupedOdometryNode.py',
        name='quadruped_odom',
        namespace=namespace,
        output='screen',
        remappings=[
            ("/tf", f"{namespace}/tf"),
            ("/tf_static", f"{namespace}/tf_static")],
        parameters=[{
            'verbose': False,
            'publish_rate': 100,
            'open_loop': False,
            'has_imu_heading': True,
            'is_gazebo': True,
            'thigh_length': 0.2,
            'base_width': 0.3,
            'encoder_resolution': 1,
            'steer_pos_multiplier': 1.0,
            'velocity_rolling_window_size': 1,
            'base_frame_id': 'base_footprint',
            'imu_topic': f'{namespace}/imu_plugin/out',
            'odom_frame_id': 'odom',
            'enable_odom_tf': True,
            'clock_topic': '/clock'
        }]
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="ign_bridge",
        namespace=namespace,
        arguments=[
            f"{namespace}/imu_plugin/out@sensor_msgs/msg/Imu@ignition.msgs.IMU",
            f"{namespace}/scan@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan"
        ]
    )

    robot_localization_node = Node(
        condition=launch.conditions.IfCondition(use_localization),
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        namespace=namespace,
        remappings=[
            ("/tf", f"{namespace}/tf"),
            ("/tf_static", f"{namespace}/tf_static")],
        parameters=[
            os.path.join(pkg_simulation, 'config', 'ekf.yaml'),
            {"use_sim_time": use_sim_time},

        ],
    )

    cmd_vel_pub = Node(
        package='go1_controller',
        executable='cmd_vel_pub.py',
        name='cmd_vel_pub',
        namespace=namespace,
        output='screen',
    )    
    rviz = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_simulation, 'launch', "rviz_launch.py")),
            launch_arguments={
                "namespace": namespace,
                "use_namespace": 'true',
                "rviz_config": os.path.join(pkg_simulation, 'config', "check_joint.rviz"),
            }.items()
        )

    return LaunchDescription([
        DeclareLaunchArgument(
                name="use_localization",
                default_value="True",
                description="Use EKF to estimagte odom->base_link transform from IMU + wheel odometry",
            ),
        DeclareLaunchArgument(
                name="use_sim_time",
                default_value="True",
                description="Flag to enable use_sim_time",
            ),
        declare_use_sim_time,
        start_ignition_cmd,
        robot_state_publisher_node,
        spawn_entity,
        joint_state_broadcaster_spawner,
        spawn_controller,
        controller,
        rviz,
        bridge,
        robot_localization_node,
        cmd_vel_pub,
        odom
    ])
