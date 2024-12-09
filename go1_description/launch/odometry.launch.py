from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    odometry_node = Node(
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

    return LaunchDescription([odometry_node])