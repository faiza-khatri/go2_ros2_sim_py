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
    xacro_path = os.path.join(pkg_simulation, 'xacro', 'robot.xacro')

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
            "-z", "0.45",  # Указываем высоту спавна
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
    odom = Node(
        package='go1_controller',
        executable='QuadrupedOdometryNode.py',
        name='quadruped_odom',
        output='screen',
             parameters=[{
                # Включить подробный вывод для отладки
                'verbose': True,

                # Частота публикации одометрии в Гц
                'publish_rate': 100,

                # Использовать открытый контур (без обратной связи от сенсоров)
                'open_loop': False,

                # Использовать ли IMU для определения направления
                'has_imu_heading': True,

                # Запуск в среде Gazebo
                'is_gazebo': True,

                # Длина бедра (в метрах)
                'thigh_length': 0.2,

                # Ширина базы (в метрах)
                'base_width': 0.3,

                # Разрешение энкодера (количества отсчетов на оборот)
                'encoder_resolution': 1,

                # Множитель для корректировки позиции поворота (если применимо)
                'steer_pos_multiplier': 1.0,

                # Размер окна для усреднения скорости (для сглаживания)
                'velocity_rolling_window_size': 1,

                # Имя базовой рамки робота
                'base_frame_id': 'base_footprint',

                # Тема для данных IMU
                'imu_topic': '/imu_plugin/out',

                # Имя рамки для одометрии
                'odom_frame_id': 'odom',

                # Включить трансформацию odom -> base_link
                'enable_odom_tf': True,

                # Тема для симулированного времени (используется в Gazebo)
                'clock_topic': '/clock'
            }]
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
    
    odometry_node = Node(
        package='quadruped_odometry',  # Имя пакета с нодой одометрии
        executable='odometry.py',          # Имя исполняемого файла (entry_point) из setup.py
        name='odometry',
        output='screen',
        parameters=[{
            "verbose": False,                          # Включить подробный вывод
            "publish_rate": 50,                        # Частота публикации одометрии (в Hz)
            "open_loop": False,                        # Открытый/закрытый цикл
            "has_imu_heading": True,                   # Использовать данные IMU для ориентации
            "is_gazebo": True,                         # Запуск в симуляции Gazebo
            "imu_topic": f"/imu_plugin/out",           # Топик для данных IMU
            "base_frame_id": "base_footprint",         # Frame ID для базового кадра
            "odom_frame_id": "odom",                    # Frame ID для одометрии
            "joints": [
                'FR_hip_joint', 'FR_thigh_joint', 'FR_calf_joint',
                'FL_hip_joint', 'FL_thigh_joint', 'FL_calf_joint',
                'RR_hip_joint', 'RR_thigh_joint', 'RR_calf_joint',
                'RL_hip_joint', 'RL_thigh_joint', 'RL_calf_joint',
            ],                                         # Имена суставов ног
            "clock_topic": "/clock",                    # Топик для синхронизации времени
            "enable_odom_tf": True,                     # Включить публикацию TF
        }]
    )

    return LaunchDescription([
        start_ignition_cmd,
        robot_state_publisher_node,
        spawn_entity,
        joint_state_broadcaster_spawner,
        spawn_controller,
        controller,
        imu_bridge,
        cmd_vel_pub,
        odom
        #odometry_node
    ])
