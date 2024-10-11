cmake_minimum_required(VERSION 3.5)
project(go1_controller)

# Используем Python 3
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

# Указываем путь к скриптам и делаем их исполняемыми
install(
  DIRECTORY scripts/
  DESTINATION lib/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS  # Это позволяет сохранять разрешения на выполнение
)

# Добавляем каждый Python скрипт в качестве исполняемого файла
# Убедитесь, что каждый скрипт в директории scripts имеет shebang (#!/usr/bin/env python3)
install(PROGRAMS
  scripts/robot_controller_gazebo.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/cmd_vel_pub.py
  DESTINATION lib/${PROJECT_NAME}
)

# Устанавливаем папку с launch файлами
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Устанавливаем зависимости
ament_package()