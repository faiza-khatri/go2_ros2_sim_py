#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch_ros.actions import Node
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit

def generate_launch_description():
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Использовать симуляционное время'
    )

    # Запуск ноды update_coordinate.py
    update_coordinate_process = ExecuteProcess(
        cmd=["/usr/bin/python3", "/root/ws/src/go1_sim/go1_gazebo/src/update_coordinate.py"],
        output='screen'
    )

    # Путь к launch файлу
    bot_start_launch_file = os.path.join(
        get_package_share_directory('go1_gazebo'),
        'launch',
        'launch.py'  # Используйте правильное имя launch файла с .py
    )

    # Включение launch файла после завершения ноды update_coordinate
    bot_start = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(bot_start_launch_file),
        launch_arguments={}.items()  # Здесь можно передавать аргументы запуска, если нужно
    )

    # Событие для запуска bot_start после завершения update_coordinate_node
    bot_start_event = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=update_coordinate_process,
            on_exit=[bot_start]
        )
    )

    return LaunchDescription([
        # Аргументы запуска
        use_sim_time,

        # Запуск ноды update_coordinate первой
        update_coordinate_process,

        # Запуск остальных действий после завершения update_coordinate
        bot_start_event
    ])
