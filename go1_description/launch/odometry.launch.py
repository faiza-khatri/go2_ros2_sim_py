from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    odometry_node = Node(
        package='quadruped_odometry',  # Имя пакета с нодой одометрии
        executable='odometry.py',      # Имя исполняемого файла (entry_point) из setup.py
        name='odometry',
        output='screen',
        parameters=[{
            "verbose": False,                          # Включить подробный вывод
            "publish_rate": 50,                        # Частота публикации одометрии (в Hz)
            "open_loop": False,                        # Открытый/закрытый цикл
            "has_imu_heading": True,                   # Использовать данные IMU для ориентации
            "is_gazebo": True,                         # Запуск в симуляции Gazebo
            "imu_topic": "/imu_plugin/out",            # Топик для данных IMU
            "base_frame_id": "base_footprint",         # Frame ID для базового кадра
            "odom_frame_id": "odom",                   # Frame ID для одометрии
            "joints": [
                'FR_hip_joint', 'FR_thigh_joint', 'FR_calf_joint',
                'FL_hip_joint', 'FL_thigh_joint', 'FL_calf_joint',
                'RR_hip_joint', 'RR_thigh_joint', 'RR_calf_joint',
                'RL_hip_joint', 'RL_thigh_joint', 'RL_calf_joint',
            ],                                         # Имена суставов ног
            "clock_topic": "/clock",                   # Топик для синхронизации времени
            "enable_odom_tf": True                     # Включить публикацию TF
        }]
    )

    return LaunchDescription([odometry_node])